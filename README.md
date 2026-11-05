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
* **Mesh.** A Meshtastic-compatible LoRa node: it sends and receives channel
  and direct text, node info and ACKs, and relays packets for other nodes.
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
  mesh/             SX1262 driver, Meshtastic-compatible mesh, protobuf, region math
  web/index.html    setup page served by the Edge
docs/protocol.md    app <-> Edge protocol, record format, Meshtastic scope
tests/host/         host unit tests for the pure-C modules
```

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
   `Sotreus-Edge-XXXX`; the password is on the LINK screen. Join it, open
   `http://192.168.4.1`, and enter the 8-digit PIN from the ACCESS screen.
   On a shared network, use the address on the LINK screen or
   `http://sotreus-edge-xxxx.local`.
4. **Set the LoRa region** under Settings before using mesh mode. Until a
   region is set, the radio never transmits.
5. **Set the clock.** The phone sends the time on connect. Logging in to
   the setup page also sets it if nothing else has. Until then, records carry
   uptime and are flagged as such.

| PRG button | Action |
|---|---|
| short press | next screen / wake display |
| hold 3-8 s | open Bluetooth pairing for 2 minutes |
| hold 10 s | factory reset (warning from 5 s; release to cancel) |

## Privacy choices

* Observations stay on the Edge until a bonded phone or an authenticated
  setup-page session reads them. The Edge makes no outbound connections and
  uses no cloud service.
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

The mesh layer speaks the Meshtastic 2.x over-the-air protocol. It uses the
same region slots, presets, header, AES-CTR channel encryption and channel
hash, so it exchanges messages with Meshtastic nodes on a matching channel,
including the public default `LongFast`. It is **not** the full Meshtastic
firmware: the Meshtastic phone app cannot connect to it, and PKI DMs,
store-and-forward, MQTT and the admin/telemetry modules are not included.
Send and read messages from the Sotreus app, or from the Mesh tab of the
setup page. [docs/protocol.md](docs/protocol.md) section 7 lists exactly what
is covered.

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
* Free heap in awareness mode: about 55 KB (low point about 49 KB). Mesh
  mode: about 84 KB.

Not yet verified:

* Bluetooth pairing and sync with a phone
* The setup page and HTTP/WebSocket API from a browser
* LoRa transmit/receive and on-air interop with a stock Meshtastic node.
  This needs a LoRa region set.
* OTA upload

## License

GPL-3.0. See [LICENSE](LICENSE).
