#![no_std]
#![no_main]

use core::fmt::Write as FmtWrite;

use esp_backtrace as _;
use esp_hal::{
    config::WatchdogStatus,
    delay::{Delay, MicrosDurationU64},
    gpio::{Io, Level, Output},
    prelude::*,
    rng::Rng,
    timer::timg::TimerGroup,
};
use esp_wifi::{
    wifi::{
        utils::create_network_interface, AuthMethod, ClientConfiguration, Configuration,
        WifiStaDevice,
    },
    wifi_interface::WifiStack,
    EspWifiInitFor,
};
use heapless::String;
use smoltcp::{iface::SocketStorage, wire::IpAddress};

// ── Compile-time configuration ────────────────────────────────────────────────

/// WiFi SSID — set via `WIFI_SSID` environment variable at build time.
const SSID: &str = env!("WIFI_SSID");
/// WiFi passphrase — set via `WIFI_PASSWORD` environment variable at build time.
const PASSWORD: &str = env!("WIFI_PASSWORD");

/// MQTT broker IPv4 address (four octets).
const MQTT_BROKER_IP: [u8; 4] = [192, 168, 1, 100];
/// MQTT broker port.
const MQTT_BROKER_PORT: u16 = 1883;

/// MQTT client identifier.
const MQTT_CLIENT_ID: &[u8] = b"lumen-leaven";
/// MQTT topic for oven temperature reports (°F).
const MQTT_TOPIC_TEMP: &[u8] = b"lumen-leaven/temperature";
/// MQTT topic for WiFi RSSI reports (dBm).
const MQTT_TOPIC_RSSI: &[u8] = b"lumen-leaven/rssi";

/// Oven proofing setpoint (°F).
const TARGET_TEMP_F: f32 = 80.0;
/// Half-band around setpoint to reduce SSR chatter (°F).
const HYSTERESIS_F: f32 = 0.5;

// ── Static storage ────────────────────────────────────────────────────────────

// smoltcp socket storage: 1 slot for the DHCP client + 1 for the MQTT TCP socket.
static mut SOCKET_STORAGE: [SocketStorage<'static>; 2] = [
    SocketStorage::EMPTY,
    SocketStorage::EMPTY,
];

// TCP buffers for the MQTT socket.
static mut MQTT_RX_BUF: [u8; 512] = [0u8; 512];
static mut MQTT_TX_BUF: [u8; 512] = [0u8; 512];

// ── Helpers ───────────────────────────────────────────────────────────────────

/// Monotonic millisecond clock used by the smoltcp / WifiStack scheduler.
fn current_millis() -> u64 {
    esp_hal::time::now().duration_since_epoch().ticks() / 1_000
}

/// Attempt one full MQTT session: CONNECT → CONNACK → PUBLISH temp & RSSI → close.
///
/// Silently returns on any error so the main loop can retry next cycle.
fn mqtt_report(stack: &WifiStack<'static, WifiStaDevice>, temp_f: f32, rssi: i32) {
    use embedded_io::{Read, Write};

    // Acquire a fresh TCP socket backed by the static buffers.
    // SAFETY: `mqtt_report` is called only from the single-threaded main loop,
    // so at most one borrow of these static buffers exists at any time.
    let rx = unsafe { &mut MQTT_RX_BUF };
    let tx = unsafe { &mut MQTT_TX_BUF };
    let mut socket = stack.get_socket(rx, tx);

    let broker = IpAddress::Ipv4(smoltcp::wire::Ipv4Address(MQTT_BROKER_IP));
    if socket.open(broker, MQTT_BROKER_PORT).is_err() {
        log::warn!("MQTT: TCP connect failed");
        return;
    }

    // ── CONNECT ───────────────────────────────────────────────────────────────
    let payload_len = 2 + MQTT_CLIENT_ID.len(); // 2-byte length prefix + bytes
    let remaining_len = (10 + payload_len) as u8; // variable header (10) + payload
    let connect_pkt: [u8; 14] = [
        0x10,           // packet type = CONNECT
        remaining_len,
        0x00, 0x04, b'M', b'Q', b'T', b'T', // protocol name
        0x04,           // protocol level = 3.1.1
        0x02,           // connect flags: clean session
        0x00, 0x3C,     // keep-alive = 60 s
        0x00, MQTT_CLIENT_ID.len() as u8, // client-id length (MSB, LSB)
    ];
    if socket.write_all(&connect_pkt).is_err()
        || socket.write_all(MQTT_CLIENT_ID).is_err()
        || socket.flush().is_err()
    {
        log::warn!("MQTT: CONNECT send failed");
        return;
    }

    // ── CONNACK ───────────────────────────────────────────────────────────────
    // Read exactly 4 bytes: 0x20 0x02 <session_present> <return_code>.
    let mut buf = [0u8; 4];
    let mut pos = 0usize;
    for _ in 0..500 {
        match socket.read(&mut buf[pos..]) {
            Ok(n) => pos += n,
            Err(_) => break,
        }
        if pos >= 4 {
            break;
        }
    }
    if pos < 4 || buf[0] != 0x20 || buf[1] != 0x02 || buf[3] != 0x00 {
        log::warn!("MQTT: CONNACK missing or rejected (rc={})", buf[3]);
        return;
    }

    // ── PUBLISH temperature ───────────────────────────────────────────────────
    let mut temp_str: String<16> = String::new();
    write!(temp_str, "{:.1}", temp_f).ok();
    if publish(&mut socket, MQTT_TOPIC_TEMP, temp_str.as_bytes()).is_err() {
        log::warn!("MQTT: temperature publish failed");
    }

    // ── PUBLISH RSSI ──────────────────────────────────────────────────────────
    let mut rssi_str: String<8> = String::new();
    write!(rssi_str, "{}", rssi).ok();
    if publish(&mut socket, MQTT_TOPIC_RSSI, rssi_str.as_bytes()).is_err() {
        log::warn!("MQTT: RSSI publish failed");
    }

    socket.close();
}

/// Send an MQTT v3.1.1 PUBLISH at QoS 0 (no acknowledgement required).
///
/// Returns `Err(())` if the combined remaining-length would exceed 127 bytes
/// (i.e. would need multi-byte encoding — not implemented here).
fn publish<W>(w: &mut W, topic: &[u8], payload: &[u8]) -> Result<(), ()>
where
    W: embedded_io::Write,
{
    let remaining = 2 + topic.len() + payload.len();
    if remaining > 127 {
        return Err(());
    }
    let header = [
        0x30u8,                    // PUBLISH, QoS 0, no retain, no dup
        remaining as u8,
        (topic.len() >> 8) as u8,  // topic length MSB
        topic.len() as u8,         // topic length LSB
    ];
    w.write_all(&header).map_err(|_| ())?;
    w.write_all(topic).map_err(|_| ())?;
    w.write_all(payload).map_err(|_| ())?;
    w.flush().map_err(|_| ())?;
    Ok(())
}

/// Read the WiFi station RSSI via the underlying ESP-IDF ROM function.
///
/// Returns -99 dBm if the call fails (e.g. not currently associated).
fn read_wifi_rssi() -> i32 {
    extern "C" {
        // Provided by the WiFi ROM / static library linked in through esp-wifi-sys.
        fn esp_wifi_sta_get_rssi(rssi: *mut core::ffi::c_int) -> core::ffi::c_int;
    }
    let mut rssi: core::ffi::c_int = -99;
    // SAFETY: safe to call whenever the WiFi driver is initialised;
    // `rssi` is a valid out-pointer for exactly one `c_int`.
    unsafe { esp_wifi_sta_get_rssi(&mut rssi as *mut _) };
    rssi
}

/// Stub until a real sensor is wired (e.g. DS18B20, I²C TMP117, thermistor + ADC).
///
/// Returns a fixed value below the setpoint so the lamp path can be verified without a sensor.
fn read_oven_temp_fahrenheit_stub() -> f32 {
    79.0
}

// ── Entry point ───────────────────────────────────────────────────────────────

#[entry]
fn main() -> ! {
    // Initialise a 72 KiB heap required by the esp-wifi driver.
    esp_alloc::heap_allocator!(72 * 1024);

    let mut config = esp_hal::Config::default();
    config.watchdog.timg0 = WatchdogStatus::Enabled(MicrosDurationU64::millis(10_000));
    let peripherals = esp_hal::init(config);

    // TIMG0: wdt = hardware watchdog (already configured above),
    //        timer0 = periodic timer source for the esp-wifi scheduler.
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let mut wdt = timg0.wdt;

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    // GPIO1 → G3MB-202P DC+ input (active-high turns SSR / lamp on — verify your module).
    let mut ssr = Output::new(io.pins.gpio1, Level::Low);

    let delay = Delay::new();

    // ── WiFi initialisation ───────────────────────────────────────────────────

    let wifi_init = esp_wifi::init(
        EspWifiInitFor::Wifi,
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .expect("WiFi init failed");

    // Build the smoltcp network interface (includes a DHCP socket when the
    // `dhcpv4` feature is active, which it is in our Cargo.toml).
    // SAFETY: SOCKET_STORAGE is accessed only here in single-threaded main.
    let socket_storage = unsafe { &mut SOCKET_STORAGE };
    let (iface, wifi_device, mut wifi_controller, socket_set) =
        create_network_interface(&wifi_init, peripherals.WIFI, WifiStaDevice, socket_storage)
            .expect("Network interface creation failed");

    // WifiStack owns the smoltcp interface and provides blocking TCP helpers.
    // The `'static` lifetime comes from SOCKET_STORAGE being a `static mut`.
    let wifi_stack: WifiStack<'static, WifiStaDevice> =
        WifiStack::new(iface, wifi_device, socket_set, current_millis);

    // Configure and start the WiFi station.
    let client_config = Configuration::Client(ClientConfiguration {
        ssid: SSID.try_into().expect("SSID too long"),
        password: PASSWORD.try_into().expect("Password too long"),
        auth_method: AuthMethod::WPA2Personal,
        ..Default::default()
    });
    wifi_controller
        .set_configuration(&client_config)
        .expect("WiFi config failed");
    wifi_controller.start().expect("WiFi start failed");
    wifi_controller.connect().expect("WiFi connect initiated");

    // ── Wait for association + DHCP lease ─────────────────────────────────────
    log::info!("Waiting for WiFi and DHCP…");
    loop {
        wdt.feed();
        wifi_stack.work();
        if wifi_stack.is_iface_up() {
            let ip = wifi_stack
                .get_ip_info()
                .map(|i| i.ip)
                .unwrap_or(smoltcp::wire::Ipv4Address::UNSPECIFIED);
            log::info!("IP address: {}", ip);
            break;
        }
        delay.delay_millis(100);
    }

    // ── Main control + MQTT reporting loop ────────────────────────────────────
    loop {
        wdt.feed();

        let current_temp_f = read_oven_temp_fahrenheit_stub();

        if current_temp_f < TARGET_TEMP_F - HYSTERESIS_F {
            ssr.set_high();
        } else if current_temp_f > TARGET_TEMP_F + HYSTERESIS_F {
            ssr.set_low();
        }

        // Keep the smoltcp / WiFi stack ticking before the blocking MQTT call.
        wifi_stack.work();

        if wifi_stack.is_iface_up() {
            let rssi = read_wifi_rssi();
            log::info!("temp={:.1}°F  rssi={}dBm", current_temp_f, rssi);
            mqtt_report(&wifi_stack, current_temp_f, rssi);
        }

        delay.delay_millis(2_000);
    }
}
