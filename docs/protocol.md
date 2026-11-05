# Sotreus Edge protocol (version 1)

How the Sotreus app (or any client) talks to a Sotreus Edge. Everything here
is local: Bluetooth LE to a bonded phone, or HTTP/WebSocket on the Edge's own
Wi-Fi network or a shared LAN. The Edge makes no outbound internet connections.

`info.protocol` carries the version. Additive changes (new fields, commands,
record types) keep the version. Breaking changes bump it.

## 1. Transports

### Bluetooth LE (primary, used for pairing)

| | UUID | Properties |
|---|---|---|
| Service | `5e0e0001-5307-4e75-8d9a-ed9e5e0e0001` | |
| info | `5e0e0002-5307-4e75-8d9a-ed9e5e0e0001` | read: static device JSON (same as the `info` command) |
| control | `5e0e0003-5307-4e75-8d9a-ed9e5e0e0001` | write / write without response: NDJSON commands |
| events | `5e0e0004-5307-4e75-8d9a-ed9e5e0e0001` | notify: NDJSON events and command responses |
| records | `5e0e0005-5307-4e75-8d9a-ed9e5e0e0001` | notify: raw 256-byte records during a `sync` |

* **Security.** Every characteristic needs an encrypted, MITM-authenticated
  link: LE Secure Connections, passkey entry, with the 6-digit code on the
  Edge's display. New pairings are accepted only while pairing is open: on
  first use until a phone bonds, after holding PRG for 3 s, or after the
  `pairing_open` command. Notifications go only to secured links.
* **Advertising.** The Edge advertises the service UUID. Its name
  (`Sotreus Edge XXXX`, with the random suffix shown on the display) is
  advertised only while pairing is open. It uses a resolvable private address
  that rotates, so bonded phones should reconnect by bond, not by address.
* **Framing.** `control` and `events` carry a byte stream of UTF-8 JSON
  objects, each ending in `\n`. Chunks follow the ATT MTU. Concatenate the
  chunks and split on `\n`. Request MTU 517; the Edge prefers 512.
* **Records.** `records` carries a byte stream of back-to-back 256-byte
  records (section 4). Reassemble by counting bytes.

### USB serial console

115200 baud, one JSON command per line. Responses are printed as
`@edge <json>`, mixed in with the firmware log. The serial console also
takes `diag`, which returns heap and per-task stack headroom.

### HTTP and WebSocket

* `POST /api/login` with `{"pin":"12345678","unix_ms":<optional>}` returns
  `{"ok":true,"token":"<32 hex>"}`. The PIN is on the Edge's ACCESS screen
  and in the `access` command. Five wrong PINs lock login for 60 s. If the
  Edge's clock is unset, `unix_ms` sets it (source `browser`).
* `POST /api/cmd` with `Authorization: Bearer <token>` and one command
  object returns the response object.
* `GET /api/records?from=<seq>&limit=<n≤5000>&fmt=json|bin` (Bearer)
  returns NDJSON record views (`json`) or raw 256-byte records (`bin`).
  Headers `X-Edge-Head` and `X-Edge-Oldest` give the stored range.
* `GET /ws?token=<token>` is a WebSocket that takes commands in and sends
  events out (text frames, NDJSON).
* `POST /api/ota` (Bearer) takes a firmware `.bin` as the body, with an
  optional `X-SHA256` header. The Edge checks the hash and image, switches
  slots and restarts. A new image that does not run cleanly for 30 s rolls
  back.
* Sessions live in RAM. They end at reboot, on `POST /api/logout`, or when
  the PIN changes.

## 2. Commands

A request is `{"cmd":"<name>", "id":<optional, echoed>, ...}`. A response is
`{"ev":"resp","id":...,"ok":true,...}` or `{"ev":"resp","ok":false,"error":"..."}`.

| cmd | Parameters | Result |
|---|---|---|
| `info` | | `info` (section 5) |
| `status` | | `status`: uptime, clock, battery, link, ble, awareness, log, mesh |
| `get_config` | | `config` (section 6) |
| `set_config` | `config`: partial config object | new `config`; applies without reboot |
| `set_mode` | `mode`: `awareness` \| `mesh` | new `config` |
| `set_time` | `unix_ms`, optional `source` | sets wall time; the phone should send it on every connect |
| `wifi_list` / `wifi_scan` | | `known` (no passwords), `nearby` (last scan); `wifi_scan` also starts a scan |
| `wifi_add` | `ssid`, `password` (empty = open) | |
| `wifi_remove` | `ssid` | |
| `access` | | `pin`, `ap_ssid`, `ap_password` |
| `new_pin` | | `pin`; ends every HTTP session |
| `sync` | `from`, optional `limit` | BLE only. Response has `oldest`, `head`; then records stream on `records`, then a `sync_done` event |
| `records` | `from`, `limit ≤ 20` | `records` (JSON views), `next` |
| `log_erase` | `confirm: true` | erases every record in the background (sequence numbers never repeat) |
| `watch` | `kind`: `ble` \| `wifi`, `addr`, `seconds ≤ 600` | streams `rssi` events for one device (proximity check) |
| `unwatch` | | |
| `mesh_send` | `text` (1-200 bytes), `to` (`broadcast` \| `!aabbccdd` \| number), `channel`, `want_ack` | `packet_id`; delivery arrives as `mesh_tx` events |
| `mesh_nodes` / `mesh_messages` / `mesh_status` | | node table / last 24 messages / radio state and counters |
| `mesh_new_identity` | | new random `node_id` |
| `regions` | | region and preset names |
| `pairing_open` | `seconds` (default 120) | |
| `ble_forget_all` | `confirm: true` | removes every bond and reopens pairing |
| `reboot` | | |
| `factory_reset` | `confirm: true` | erases settings, networks, bonds, identity and records, then reboots |

## 3. Events

| ev | When | Fields |
|---|---|---|
| `resp` | reply to a command | see above |
| `status` | every 10 s while a client listens | `status` |
| `obs` | a record was written | `rec` (record view, section 4) |
| `seen` | a device was first heard in a new sighting window | `kind`, `addr`, `rssi`, `label?`, `uptime_ms` |
| `rssi` | during `watch`, at most 4 Hz | `kind`, `addr`, `rssi`, `channel?`, `uptime_ms`, `unix_ms?` |
| `mode` | mode changed | `mode`, `awareness_paused` |
| `sync_done` | end of a BLE sync | `ok`, `sent`, `next`, `head`, `more` |
| `mesh_rx` | text addressed to us or the channel | `msg` |
| `mesh_tx` | our message progressed | `id`, `status`: `sent` \| `delivered` \| `relayed` \| `no_ack` \| `no_relay_heard` \| `failed` (`reason`) |
| `mesh_node` | node info received | `node` |

Live events are best effort. If a client is slow, the Edge drops events and
counts them (`status.events_dropped`). The record log plus `sync` is the
reliable path, so de-duplicate by `seq`.

## 4. Records

Records live in a flash ring (≈ 15,600 slots, oldest overwritten first).
`(info.device_id, seq)` is the stable Edge record ID. `seq` grows by one per
record and never repeats on a device, including across log erases.

All multi-byte fields are little-endian.

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | `seq` |
| 4 | 1 | `type`: 1 BLE, 2 Wi-Fi AP, 3 Wi-Fi probe, 4 mesh text, 5 Edge event |
| 5 | 1 | `flags`: 0x01 TIME_VALID, 0x02 CONTINUES, 0x04 TRUNCATED |
| 6 | 2 | `boot`: boot counter (low 16 bits) |
| 8 | 8 | `first_ms` (int64) |
| 16 | 8 | `last_ms` (int64) |
| 24 | 2 | `body_len` |
| 26 | 2 | `format` = 1 |
| 28 | 224 | body |
| 252 | 4 | CRC-32 of bytes 0-251, same value as zlib `crc32()` or `java.util.zip.CRC32` |

**Time.** With TIME_VALID set, `first_ms` and `last_ms` are Unix ms. Without
it, they are milliseconds since boot number `boot`, because the Edge had no
wall time yet. Keep these records labelled "time not known" unless the app
can anchor that boot itself, and never present them as fresh.

**Sightings.** Types 1-3 summarise one device over a window: written when it
has been quiet for `close_after_s` (default 60 s), or every `max_window_s`
(default 300 s) while still heard. In the second case CONTINUES is set and
the next window follows. RSSI is what the Edge heard: proximity evidence,
not distance or direction.

### Type 1: BLE advertiser (origin `EdgeBle`, provenance SENSED)

| Offset | Size | Field |
|---|---|---|
| 0 | 6 | address, display order (MSB first) |
| 6 | 1 | address type: 0 public, 1 random |
| 7 | 1 | last PDU: 0 ADV_IND, 1 ADV_DIRECT_IND, 2 ADV_SCAN_IND, 3 ADV_NONCONN_IND |
| 8 | 3 | RSSI last / max / min (int8, dBm) |
| 11 | 1 | flags: 0x02 has scan response, 0x04 connectable |
| 12 | 4 | advertising reports in the window |
| 16 | 1 | adv length |
| 17 | 1 | scan-response length |
| 18 | 31 | last advertising data (raw AD structures) |
| 49 | 31 | last scan response |

The raw AD bytes go to the phone so its signature catalogue can classify
them. The Edge does not classify.

### Type 2: Wi-Fi access point (origin `EdgeWifi`, SENSED)

| Offset | Size | Field |
|---|---|---|
| 0 | 6 | BSSID |
| 6 | 1 | channel |
| 7 | 3 | RSSI last / max / min |
| 10 | 1 | auth: 0 open, 1 WEP, 2 WPA, 3 WPA2, 4 WPA3, 5 WPA2/WPA3, 6 enterprise, 7 OWE, 255 unknown |
| 11 | 1 | flags: 0x01 hidden, 0x02 probe response, 0x04 Remote ID element, 0x08 scan list only |
| 12 | 4 | frames heard |
| 16 | 2 | beacon interval (TU) |
| 18 | 1 | SSID length |
| 19 | 32 | SSID bytes |
| 51 | 1 | Remote ID element length |
| 52 | 120 | Remote ID vendor element (OUI `FA:0B:BC` onward), truncated |

### Type 3: Wi-Fi probe request (origin `EdgeWifi`, SENSED)

| Offset | Size | Field |
|---|---|---|
| 0 | 6 | source MAC |
| 6 | 1 | channel |
| 7 | 3 | RSSI last / max / min |
| 10 | 1 | flags: 0x01 randomised MAC, 0x02 wildcard probes seen, 0x04 names stored in clear, 0x08 more names than stored |
| 11 | 1 | number of distinct probed names (≤ 4) |
| 12 | 4 | frames heard |
| 16 | 16 | 4 × FNV-1a-32 of `probe_salt ‖ SSID` |
| 32 | 132 | 4 × (len u8, SSID[32]); only when `awareness.probe_names` is on |

By default the Edge stores only salted hashes of the network names a nearby
device asks for. The app gets `info.probe_salt`, so it can hash names it
already knows (for example the owner's own networks) and compare.

### Type 4: mesh text (origin `EdgeLora`, provenance NETWORK)

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | from node number |
| 4 | 4 | to node number (`0xffffffff` = channel) |
| 8 | 4 | packet id |
| 12 | 1 | local channel index |
| 13 | 1 | hop_limit (low nibble), hop_start (high nibble) |
| 14 | 1 | RSSI (int8) |
| 15 | 1 | SNR × 4 (int8) |
| 16 | 1 | 0 received, 1 sent |
| 17 | 1 | text length |
| 18 | 206 | UTF-8 text (TRUNCATED flag if longer) |

### Type 5: Edge event (origin `Edge`)

| Offset | Size | Field |
|---|---|---|
| 0 | 2 | code: 1 boot, 2 mode, 3 time set, 4 log erased, 5 config, 6 firmware |
| 2 | 1 | detail length |
| 3 | 221 | detail text |

## 5. `info`

Static for a boot, so it is safe for long BLE reads:

```json
{"protocol":1,"device_id":"9f3c…","name":"Sotreus Edge","suffix":"1A2B","fw":"0.1.0",
 "idf":"v6.0.1","hw":"heltec_wifi_lora_32_v3","boot":12,"record_size":256,"record_format":1,
 "probe_salt":"…","caps":{"ble_observation":true,"wifi24_observation":true,
 "wifi_probe_observation":true,"gnss":false,"local_storage":true,"firmware_switching":false,
 "meshtastic_mode":true,"lora":true,"display":true,"battery":true,"ota_upload":true}}
```

`firmware_switching` is false because mesh mode is built in. It is not a
second firmware.

## 6. Config

```json
{
  "mode": "awareness",
  "link": "auto",
  "awareness": {"ble": true, "ble_active": true, "wifi_aps": true, "wifi_probes": true,
                "probe_names": false, "rssi_floor": -100, "close_after_s": 60, "max_window_s": 300},
  "display": {"timeout_s": 60},
  "mesh": {"region": "UNSET", "preset": "LONG_FAST", "hop_limit": 3, "rebroadcast": "all",
           "tx_power": 0, "channel_num": 0, "node_id": "!5ac1e0d2",
           "long_name": "Sotreus Edge 1A2B", "short_name": "SE2B", "nodeinfo_interval_s": 10800,
           "channels": [{"name": "", "psk": "default"}]}
}
```

* `link` is one of `auto`, `ap`, `sta` or `off`.
* `mesh.region` is a Meshtastic region name, or `UNSET`. While it is
  `UNSET`, the radio never transmits.
* `mesh.preset` is one of `LONG_FAST`, `LONG_SLOW`, `LONG_MODERATE`,
  `MEDIUM_SLOW`, `MEDIUM_FAST`, `SHORT_SLOW`, `SHORT_FAST` or `SHORT_TURBO`.
* A channel `psk` is `default` (Meshtastic `AQ==`), `none`, `simpleN`, or
  `base64:<16 or 32 bytes>`. An empty primary channel name means the preset
  name (`LongFast`), which matches Meshtastic's public default channel.

## 7. Meshtastic compatibility

On air the Edge matches Meshtastic 2.x: region frequency slots (djb2 of the
channel name), sync word 0x2B, 16-symbol preamble, the 16-byte header,
AES-CTR with the channel key and nonce `packet_id(u64 LE) ‖ from(u32 LE) ‖ 0`,
and the channel hash. It sends and receives text (port 1). It sends node info
(port 4) and answers node-info requests. It decodes positions (port 3) and
routing ACK/NAK (port 5). It ACKs direct messages, retries its own direct
messages up to 3 times, and relays with Meshtastic's SNR-weighted contention
window.

It does **not** implement the following:

* the Meshtastic phone API, so the Meshtastic app cannot connect to it
* PKI direct messages. Its node info carries no public key, so other nodes
  fall back to channel encryption for messages to it.
* store-and-forward, MQTT, telemetry, range test, traceroute replies and
  remote admin
* next-hop routing (it floods)

It sends `bitfield = 0`, so its packets are never marked OK for MQTT upload.
Its node number is random, not derived from a hardware address.
