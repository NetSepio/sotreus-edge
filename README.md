# Sotreus Edge firmware

Firmware for the **Heltec WiFi LoRa 32 V3** (ESP32-S3 + SX1262 + 0.96" OLED)
that turns it into a companion sensor for the Sotreus app. The Edge does the
Bluetooth and Wi-Fi listening that phones do poorly: no OS scan throttling,
no background limits, and 2.4 GHz probe requests that phones cannot see. It
keeps what it hears in its own flash, and the paired phone only has to handle
internet-connected feeds.

It runs in one of two modes, chosen from the app or the setup page:

* **Awareness.** Continuous BLE advertising capture and 2.4 GHz Wi-Fi
  management-frame capture (access points, probe requests, Remote ID beacon
  elements). Results are folded into per-device sightings and written to a
  3.8 MB on-device record log.
* **Mesh.** A Meshtastic-compatible LoRa node, controlled from the Sotreus
  app or the setup page: channel and private (PKI) direct messages, replies
  and reactions, roles and relaying, position, telemetry, traceroute,
  waypoints, neighbour info, store & forward, range test, paxcounter,
  detection sensor, serial bridge, notifications and quick messages.
  Awareness pauses while mesh mode runs, and records already stored are kept.
  Switching modes is instant and needs no reboot or reflash.

The Wi-Fi link is set separately. In `auto` the Edge joins a known network if
one is in range, and otherwise runs its own access point
(`Sotreus-Edge-XXXX`). Other choices are `ap`, `sta` and `off` (Bluetooth
only).

## Layout

```
main/
  main.c            boot order, mode switching, OTA confirmation
  edge_config.*     settings + device identity in NVS
  edge_log.*        256-byte record ring on the "obslog" partition
  edge_records.*    record body layouts + JSON views
  sense.*           sighting aggregation (BLE / AP / probe)
  edge_ble.*        NimBLE: GATT link, passkey pairing, observer scan
  edge_wifi.*       STA/AP link policy, promiscuous capture, channel hopping
  edge_http.*       HTTP API, WebSocket, setup page, OTA upload
  edge_api.*        JSON command dispatcher shared by BLE / HTTP / WS
  edge_events.*     live NDJSON event fan-out
  ui.*, ssd1306.*   OLED screens, PRG button, LED
  mesh/             SX1262 driver, Meshtastic-compatible mesh, protobuf, region math,
                    X25519 / AES-CCM for PKI direct messages
  web/index.html    setup page served by the Edge
docs/protocol.md    app <-> Edge protocol, record format, Meshtastic scope, Context feeds
tests/host/         host unit tests for the pure-C modules
tools/mock_edge.py  fake Edge for developing the setup page
tools/context-relay Cloudflare Worker the browser can use for three Context feeds
```

### Flash layout (8 MB)

| Partition | Size | Holds |
|---|---|---|
| `nvs` | 24 KB | settings, Wi-Fi networks, bonds, PIN, mesh key pair |
| `ota_0`, `ota_1` | 2 MB each | firmware (about 1.5 MB now) |
| `obslog` | 3.7 MB | about 15,100 records of 256 bytes |
| `meshdb` | 64 KB | nodes, messages and waypoints (two 32 KB copies) |
| `coredump` | 128 KB | the last crash, for `idf.py coredump-info` |

After a firmware update that changes this table, flash with `idf.py flash`
(not `app-flash`) so the new table is written. Records in `obslog` keep
their place.

## Build and flash

Requires ESP-IDF v6.0.x (`esp32s3` target). The `cjson` and `mdns`
components come from the component manager.

```bash
source ~/.espressif/tools/activate_idf_v6.0.1.sh
```

```bash
idf.py build
```

Connect the board over USB-C (it shows up as `/dev/cu.usbserial-*` on macOS),
then:

```bash
idf.py -p /dev/cu.usbserial-0001 flash monitor
```

Erase the flash first if the board previously ran Meshtastic or other
firmware:

```bash
idf.py -p /dev/cu.usbserial-0001 erase-flash flash monitor
```

If the port is not detected, hold **PRG**, tap **RST**, release **PRG**, then
retry.

Host tests (no board needed):

```bash
make -C tests/host test
```

## First use

1. **Power on.** The display shows the status screen. Short-press **PRG**
   to cycle through STATUS → LINK → ACCESS → LORA.
2. **Pair a phone over Bluetooth.** Pairing is open until the first phone
   bonds. Later, hold **PRG** for 3 s to open it for 2 minutes. The 6-digit
   code appears on the display.
3. **Or use the setup page.** With no known Wi-Fi, the Edge runs
   `Sotreus-Edge-XXXX`; the password is on the LINK screen. Join it, and
   the setup page opens by itself as a sign-in sheet (captive portal). If it
   doesn't, open `http://192.168.4.1`. Enter the 8-digit PIN from the
   ACCESS screen. After you log in, the device stops prompting and treats
   the network as "no internet", so it keeps using mobile data for the
   internet. On a shared network, use the address on the LINK screen or
   `http://sotreus-xxxx.local`.
4. **Set the LoRa region** under Settings before using mesh mode. Until a
   region is set, the radio never transmits.
5. **Clock.** When the Edge has joined a network it takes network time
   (SNTP, `pool.ntp.org`). Otherwise a paired phone sends the time on
   connect, or logging in to the setup page sets it. The browser also passes
   its time zone, so the display shows local time. Until a clock is known,
   records carry uptime and are flagged as such.

| PRG button | Action |
|---|---|
| short press | next screen / wake display |
| hold 3-8 s | open Bluetooth pairing for 2 minutes |
| hold 10 s | factory reset (warning from 5 s; release to cancel) |

## Setup page

The Edge serves its own web app in the Sotreus "Night Instrument" theme,
with Geist and Instrument Serif fonts bundled on the device. Tabs follow the
mode:

- **Overview** shows the current mode. Switching modes always asks for
  confirmation, and asks for the LoRa region first if none is set.
- **Observations** appears in Awareness mode only; **Mesh** in Mesh mode
  only. Settings show only the current mode's options.
- **Context** appears only when the Edge has joined a network in station
  mode and the browser can reach the internet. It mirrors the app's Context
  screen:
  - an OpenFreeMap dark map
  - Environment: air quality (Open-Meteo, PREDICTED), earthquakes (USGS),
    natural events (NASA EONET), disaster alerts (GDACS) and official
    warnings (NWS or MeteoAlarm, copied in the issuer's words)
  - Aircraft (ADS-B from adsb.lol, broadcast by the aircraft, not sensed)
  - Satellites (CelesTrak orbits, passes predicted in the browser)

  Every feed is fetched by the browser, never by the Edge, so Context costs
  the Edge no memory or radio time. Sources are set under Settings › Context
  sources, with a "What leaves this browser" list. The area is typed, picked
  on the map, or set from the browser's IP address (GeoJS), and is stored
  rounded to 0.1°. MapLibre GL JS, uPlot and satellite.js are served from the
  Edge itself.

  adsb.lol, GDACS and MeteoAlarm do not let a web page read them (no CORS
  headers, or a bot check). The browser reaches them through a relay you
  set: [tools/context-relay](tools/context-relay) is a small Cloudflare
  Worker with a fixed allow-list. With no relay set, those three stay off.

## Privacy choices

* Observations stay on the Edge until a bonded phone or an authenticated
  setup-page session reads them. The Edge uses no cloud service. Its only
  outbound connections are made while it has joined a network:
  * network time (SNTP to `pool.ntp.org`)

  The setup page's Context feeds are fetched by the browser, not the Edge.
* The Edge does not broadcast stable hardware identifiers:
  * The Bluetooth address rotates (resolvable private address), and the name
    is advertised only while pairing is open.
  * The Wi-Fi station uses a random per-device MAC instead of the factory
    MAC, and the access point picks a new random MAC on every boot.
  * Wi-Fi scans are passive, so the Edge sends no probe requests of its own.
  * The mesh node number is random.
* Network names that nearby devices probe for are stored only as salted
  hashes, unless you turn on "keep probed network names in clear".
* Signal strength is recorded as proximity evidence only, never as distance
  or direction.
* A factory reset erases settings, Wi-Fi passwords, Bluetooth bonds, device
  identity and every record.

Known gap: NVS (Wi-Fi passwords, bonds, PIN) is not encrypted, because that
needs flash encryption. Enable flash encryption and secure boot for
production builds; both burn eFuses.

## Meshtastic compatibility

The mesh layer speaks the Meshtastic 2.x over-the-air protocol: the same
region slots, presets, header, channel encryption and channel hash, and
Meshtastic 2.5 public-key (PKI) direct messages. It exchanges messages with
Meshtastic nodes on a matching channel, including the public default
`LongFast`, and channel links (`meshtastic.org/e/#…`) can be shared both
ways. Use it from the Sotreus app or the Mesh tab of the setup page; the
Meshtastic phone app cannot connect. MQTT, remote admin, remote hardware,
the store & forward server, audio and ATAK are left out on purpose.
[docs/protocol.md](docs/protocol.md) section 7 lists exactly what is covered.

Sotreus Edge is not affiliated with or endorsed by Meshtastic®, a registered
trademark of Meshtastic LLC.

## Serial console

The USB serial port takes the same JSON commands as Bluetooth and HTTP, one
per line, at 115200 baud. Responses come back as lines starting with
`@edge `. Owning the USB cable is treated as owning the device, so no PIN is
asked. Examples:

```
{"cmd":"status"}
{"cmd":"diag"}
{"cmd":"set_mode","mode":"mesh"}
{"cmd":"records","from":0,"limit":5}
```

`diag` reports free heap, the largest free block, and each task's minimum
free stack. On macOS, opening the port resets the board, so send commands in
the same session that opened it.

## Hardware status (Heltec V3, board rev 3.2)

Verified on a board:

* Boot, SX1262 detection, OLED, battery measurement (rev 3.2, enable level
  high, auto-detected)
* Awareness: BLE, access points, probe requests
* Sighting records: written with uptime time, then wall time once the clock
  is set
* Live switching awareness ↔ mesh, and the AP fallback with channel hopping
* Nearby Wi-Fi scan from the setup page: completes in about 12 s with
  20+ networks, in both modes
* Captive-portal DNS answers (host-tested). The DNS responder starts on the
  access point.
* Free heap in awareness mode with Wi-Fi joined: about 76 KB (low point about
  54 KB). Mesh mode with its node, message and waypoint tables: about 60 KB.
* Setup page served gzipped from the Edge. MapLibre (211 KB) loads in about
  2 s over Wi-Fi.
* `sotreus-96ff.local` resolves; `_sotreus._tcp` is browsable.
* Mesh over the serial API: settings round-trip (role, relay mode, quick
  messages, fixed position), channel link export and re-import, sending
  text, a reaction and a traceroute, and messages kept across a restart.
* With USB power and no battery, battery details stay hidden.
* Network time sets the clock about 15 s after joining a network (within
  0.2 s of the Mac's clock).
* The OLED shows local time using the browser's time zone.
* LoRa with region IN (LongFast, 865.875 MHz, 22 dBm): the radio tunes,
  transmits its node info, and returns to receive. Noise floor on the
  channel is about -108 dBm.

Not yet verified:

* Bluetooth pairing and sync with a phone
* The setup page and HTTP/WebSocket API from a browser, including the
  captive-portal sheet on iOS and Android, and the channel-by-channel scan
  while a phone is connected to the access point
* Anything that needs another Meshtastic node in range: receiving and
  decoding, PKI direct messages both ways, ACKs, traceroute replies, store &
  forward, and a stock node listing the Edge. No LoRa traffic was in range
  during testing. The crypto and protobuf code passes known-answer tests on
  the host (RFC 7748, FIPS-197, RFC 3610 vectors and Meshtastic channel
  links).
* The Context relay on Cloudflare itself. Its code was run locally under
  Node against the live feeds.
* OTA upload

## License

GPL-3.0. See [LICENSE](LICENSE). Made by NetSepio LLC (https://netsepio.com), product site https://sotreus.com.

Bundled third-party files (main/web/vendor, main/web/fonts), with their
licences in [licenses/](licenses):

| File | Version | Licence | Source, integrity |
|---|---|---|---|
| maplibre-gl.js / .css | 4.7.1 | BSD-3-Clause | npm `maplibre-gl@4.7.1`, registry sha512 verified; `sourceMappingURL` line removed |
| uPlot.iife.min.js / .min.css | 1.6.31 | MIT | npm `uplot@1.6.31`, registry sha512 verified |
| satellite.min.js | 5.0.0 | MIT | npm `satellite.js@5.0.0`, registry sha512 verified |
| Geist, Geist Mono, Instrument Serif | from the Sotreus app | SIL OFL 1.1 | subset to Latin and converted to WOFF2 with fonttools `pyftsubset` |

Develop the setup page without a board using `python3 tools/mock_edge.py`
(PIN 12345678, http://127.0.0.1:8080). It serves the same files, a stateful
fake API with sample mesh nodes and messages, and binary records in the
firmware's layout.
