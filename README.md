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
  3.4 MB on-device record log.
* **Mesh.** A Meshtastic-compatible LoRa node, controlled from the Sotreus
  app, the setup page, or a Meshtastic app over the air (remote admin):
  channel and private (PKI) direct messages, replies and reactions, roles and
  relaying, position, telemetry, traceroute, waypoints, neighbour info,
  store & forward (client and flash-backed server), range test, paxcounter,
  detection sensor, serial bridge, notifications, quick messages, remote
  admin, remote hardware (GPIO), MQTT with TLS, and ATAK (TAK positions and
  GeoChat from the Meshtastic ATAK plugin, and the `TAK_TRACKER` role).

Each mode boots on its own. Switching modes saves the setting and restarts
the Edge (about 15 seconds): Mesh mode never loads the sighting pipeline,
and Awareness mode puts the LoRa radio to sleep and frees the mesh task.
Records already stored are kept.

The Wi-Fi link is set separately. In `auto` the Edge joins a known network if
one is in range, and otherwise runs its own access point
(`Sotreus-Edge-XXXX`). Other choices are `ap`, `sta` and `off` (Bluetooth
only).

## Supported boards

| Board | Status |
|---|---|
| **Heltec WiFi LoRa 32 V3** (ESP32-S3FN8, 8 MB flash, SX1262, 0.96" OLED) | Supported. Tested on board rev 3.2. |
| Other ESP32 dev boards, including other Heltec models | Not supported yet |

This firmware is built only for the Heltec WiFi LoRa 32 V3. Its pin map
(`main/board.h`), LoRa radio driver and 8 MB flash layout are made for that
board, so don't flash it onto any other board. Heltec sells the V3 in two
frequency versions: 863–928 MHz (for example `US` or `EU_868`) and
470–510 MHz (`CN`). Get the one that covers your LoRa region.

Support for more dev boards is planned for later releases. Each board will
get its own firmware build: you choose your dev board, and the build made for
it is the one flashed to its ESP32. The Edge already reports its board in
`info.hw` (`heltec_wifi_lora_32_v3`), so the app and the setup page can tell
boards apart.

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
| `ota_0`, `ota_1` | 2 MB each | firmware (about 1.7 MB now) |
| `obslog` | 3.4 MB | about 14,000 records of 256 bytes |
| `sfdb` | 256 KB | store & forward server history, about 1,000 messages |
| `meshdb` | 64 KB | nodes, messages and waypoints (two 32 KB copies) |
| `coredump` | 128 KB | the last crash, for `idf.py coredump-info` |

After a firmware update that changes this table, flash with `idf.py flash`
(not `app-flash`) so the new table is written. Records in `obslog` keep
their place.

## Build and flash

These steps are for the Heltec WiFi LoRa 32 V3 only (see
[Supported boards](#supported-boards)). Requires ESP-IDF v6.0.x (`esp32s3` target). The `cjson` and `mdns`
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

1. **Power on.** The display shows the status screen, with the mode
   (`AWARE` or `MESH`) in its header. Short-press **PRG** to cycle through
   STATUS → LINK → ACCESS in Awareness mode, or STATUS → LORA → LINK →
   ACCESS in Mesh mode. While the Edge restarts into another mode, the
   display shows "Switching to MESH" (or AWARE).
2. **Pair a phone over Bluetooth.** Pairing is open until the first phone
   bonds. Later, hold **PRG** until the display shows "Pairing open"
   (3 s), then let go: pairing stays open for 2 minutes. The 6-digit code
   appears on the display.
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
| hold 3 s | open Bluetooth pairing for 2 minutes (opens while still held; let go) |
| hold 15 s | factory reset (countdown from 10 s; let go to cancel) |

## Setup page

The Edge serves its own web app in the Sotreus "Night Instrument" theme,
with Geist and Instrument Serif fonts bundled on the device. Tabs follow the
mode:

- Every section keeps itself current: live events arrive over a WebSocket,
  and while a section is visible it also polls the Edge (status every 15 s
  without live events, the Mesh tab every 20 s, Wi-Fi every 30 s, new records
  every 20 s). A silent WebSocket is reopened, a tab coming back into view
  refreshes at once, and the Overview says "Stale" when the Edge has not
  answered for 45 s. Settings forms are not refreshed under you.
- **Overview** shows the current mode. Switching modes always asks for
  confirmation, asks for the LoRa region first if none is set, and, before
  the restart into Mesh, asks who may administer the Edge over the air.
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
  setup-page session reads them. While the Sotreus app is connected over
  Bluetooth and has asked for it, new records go straight to the phone and
  are stored on the Edge only if the phone does not confirm them (see
  docs/protocol.md, "Phone hand-off"), which also saves flash wear. Outbound connections, only while the Edge
  has joined a network:
  * network time (SNTP to `pool.ntp.org`)
  * in Mesh mode, the MQTT broker you configure (off by default)

  The setup page's Context feeds are fetched by the browser, not the Edge, so
  Awareness scanning is never slowed by them.
* Remote admin is open to any node by default (the owner's choice). The
  setup page shows a warning until an admin key is set. Even when open, the
  Wi-Fi, security, MQTT, channel and remote hardware settings and factory
  resets need an admin key, the private key and Wi-Fi password are never
  sent, and every change made over the air is written to the record log.
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
ways. A Meshtastic app on another node can administer it remotely (with
Meshtastic's session keys), and it can act as an MQTT gateway and a store &
forward server. [docs/protocol.md](docs/protocol.md) section 7 lists exactly
what is covered.

Regions, modem presets and device roles match the stable Meshtastic release
(2.7): every sub-GHz region (up to `BR_902`), the presets up to `LONG_TURBO`,
and the `CLIENT_BASE` role, with 2.7's relay rules. A preset wider than the
region's band falls back to `LONG_FAST`, as in Meshtastic, and the status
says so.

Not included:

* **The Meshtastic phone API** (Bluetooth / TCP `ToRadio`/`FromRadio`), so the
  Meshtastic app cannot connect to the Edge directly. The Sotreus app and the
  setup page are its clients. Remote admin from a Meshtastic app on another
  node works.
* **Audio.** The settings exist (and can be set remotely), but voice is
  neither sent nor played: the Heltec V3 has no microphone, speaker or I2S
  codec, and the codec2 library is not built in yet.
* **Radio settings only in the Meshtastic 2.8 pre-releases:** the `LITE`,
  `NARROW`, `TINY` and `MEDIUM_TURBO` presets and the `EU_866`, `EU_N_868` and
  amateur-radio (ITU) regions. They follow once 2.8 is a stable release.
* **Newer Meshtastic features:** packet signatures, key verification, status
  messages, mesh beacons, paging, critical alerts, and telemetry other than
  device metrics. Section 7 of the protocol lists the rest.

### ATAK

ATAK (Android Team Awareness Kit) is a map app used by emergency services,
search-and-rescue teams and the military. Meshtastic's ATAK plugin sends team
positions and GeoChat over LoRa. The Edge works with it the way Meshtastic
firmware does:

* It reads TAK packets from both plugin versions (v1 `TAKPacket` on port 72,
  v2 `TAKPacketV2` on port 78) and lists ATAK users with callsign, team,
  role, reported position and battery; GeoChat appears in Messages as
  `[TAK]`. Packets the plugin compressed (v1 unishox2 strings, v2 zstd) are
  relayed unchanged; their positions are shown when readable (v1), their text
  is not.
* With the role `TAK_TRACKER` ("TAK position beacon") it reports its position
  as an uncompressed TAK v2 position (flags byte `0xFF`), like Meshtastic,
  with the team colour and role set under Settings › ATAK (Meshtastic
  `TAKConfig`, also settable by remote admin).
* From the setup page (and later the Sotreus app) it can send TAK GeoChat to
  all chat rooms and its TAK position on demand.

What the Edge cannot do is run ATAK itself: an ATAK phone still needs the
Meshtastic Android app (phone API above) or, later, the Sotreus app to
translate between ATAK and the mesh.

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
* Switching awareness ↔ mesh restarts into the other mode (the sighting task
  is absent in Mesh, the mesh task in Awareness), and the AP fallback with
  channel hopping
* Nearby Wi-Fi scan from the setup page: completes in about 12 s with
  20+ networks, in both modes
* Captive-portal DNS answers (host-tested). The DNS responder starts on the
  access point.
* Free heap with Wi-Fi joined: Awareness about 80 KB (low point about 65 KB).
  Mesh with its tables, store & forward and an MQTT connection: about 58 KB.
  During the TLS handshake to `mqtt.meshtastic.org:8883` the low point was
  33 KB.
* MQTT: connect, subscribe, uplink of the Edge's own packet (still encrypted)
  and downlink of a broker message into the mesh, against a local test broker;
  TLS connection to `mqtt.meshtastic.org:8883`.
* Store & forward server: sent and downlinked channel text kept in flash.
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
* Mesh radio settings, receive only (transmit off): IN `LONG_TURBO` tunes to
  865.750 MHz; `LONG_TURBO` in `EU_868` falls back to `LongFast` at
  869.525 MHz and `status.mesh.preset_too_wide` names it; the `CLIENT_BASE`
  role is taken and an unknown role is refused.
* Bluetooth pairing, live hand-off and sync with a phone (Solana Seeker,
  Android 16): records arrive live, unconfirmed ones are written to flash and
  come back by `sync`, and a planned restart loses none.

Not yet verified:

* The setup page and HTTP/WebSocket API from a browser, including the
  captive-portal sheet on iOS and Android, and the channel-by-channel scan
  while a phone is connected to the access point
* Anything that needs another Meshtastic node in range: receiving and
  decoding, PKI direct messages both ways, ACKs, traceroute replies, store &
  forward requests, remote admin and remote hardware from a Meshtastic app,
  relaying by role (`ROUTER`, `ROUTER_LATE`, `CLIENT_BASE`) and hop limits
  kept between favourite routers, and a stock node listing the Edge. No LoRa
  traffic was in range during testing. The crypto and protobuf code passes known-answer tests on
  the host (RFC 7748, FIPS-197, RFC 3610 vectors, Meshtastic channel
  links, and AdminMessage / ServiceEnvelope / StoreAndForward round trips).
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
