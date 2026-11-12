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
  It advertises every 100-150 ms while pairing is open and for 30 s after
  boot or after a link drops, otherwise about once a second. A phone that
  reconnects with a direct (not background) connect inside that window is
  back within seconds.
* **Link-layer packets.** 3 s after a paired phone's link is encrypted, the
  Edge limits the packets it sends to 27 bytes, once per link. With 251-byte
  packets an Android 9 phone (Huawei, Bluetooth 4.2) lost every link within
  two minutes. The limit lowers throughput and needs about 5 times as many
  packets per record, so keep polling light. A phone that handles 251-byte
  packets can say so with `ble_link` (needs `caps.ble_full_size`). The Edge
  remembers that for the bonded phone and leaves its links at 251. Send it
  after the handshake on every link while the setting is on. If it arrives
  before the 3 s step, the current link keeps 251 too; otherwise the next
  link does. The Edge never changes a live link's packet size for it.
* **Framing.** `control` and `events` carry a byte stream of UTF-8 JSON
  objects, each ending in `\n`. Chunks follow the ATT MTU. Concatenate the
  chunks and split on `\n`. Request MTU 517; the Edge prefers 512.
* **Records.** `records` carries a byte stream of back-to-back 256-byte
  records (section 4). Reassemble by counting bytes.

### Captive portal (access point only)

The access point's DHCP hands out `192.168.4.1` as the DNS server. Until a
device logs in, every name resolves to the Edge, and unknown paths
(including the OS connectivity checks) redirect to `http://192.168.4.1/`.
After a successful `/api/login` from that device's address, its DNS queries
get NXDOMAIN, so the OS treats the network as having no internet and keeps
its other connection (mobile data) for internet traffic. The app should
expect this and bind its sockets to the Edge's Wi-Fi network (Android
`Network.bindSocket` / `ConnectivityManager.bindProcessToNetwork`).

### mDNS

On a shared network the Edge answers as `sotreus-<suffix>.local` (for
example `sotreus-96ff.local`). It advertises `_http._tcp` and
`_sotreus._tcp` on port 80 with TXT records `vendor=NetSepio LLC`,
`product=Sotreus Edge`, `url=https://sotreus.com`, `model`, `fw`, `api=1`,
`path=/`. The app can browse `_sotreus._tcp` to find Edges on the LAN.

### USB serial console

115200 baud, one JSON command per line. Responses are printed as
`@edge <json>`, mixed in with the firmware log. The serial console also
takes `diag`, which returns heap and per-task stack headroom. For link
tests, `{"cmd":"diag","ble_tx_octets":N}` sets the link-layer payload the
Edge limits new paired links to (27-251; 0 means 251) until the next
restart. Live links and phones that asked for full-size packets
(`ble_link`) are left alone. The console logs connection parameter,
PHY and data-length changes.

### HTTP and WebSocket

* `POST /api/login` with `{"pin":"12345678","unix_ms":<optional>,"tz_offset_min":<optional>}` returns
  `{"ok":true,"token":"<32 hex>"}`. The PIN is on the Edge's ACCESS screen
  and in the `access` command. Five wrong PINs lock login for 60 s. If the
  Edge's clock is unset, `unix_ms` sets it (source `browser`).
* `POST /api/cmd` with `Authorization: Bearer <token>` and one command
  object returns the response object.
* `GET /api/records?from=<seq>&limit=<n≤5000>&fmt=json|bin` (Bearer)
  returns NDJSON record views (`json`) or raw 256-byte records (`bin`).
  Headers `X-Edge-Head` and `X-Edge-Oldest` give the stored range. The setup
  page uses `bin` and decodes records in the browser (section 4), so the
  Edge spends no memory building JSON for them.
* `GET /ws?token=<token>` is a WebSocket that takes commands in and sends
  events out as NDJSON text frames. A new record goes out as a binary frame
  holding the raw 256-byte record.
* The Edge makes no internet requests for the page's Context tab. The
  browser fetches every feed itself (section 8).
* `GET /assets/<file>`: the map, chart and orbit libraries and the fonts,
  versioned and gzip-encoded, cached for a year.
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
| `status` | | `status`: uptime, clock, battery (`present`, `source`: `battery` \| `usb` \| `unknown`; `mv` and `percent_approx` only when present), link, ble, awareness, log, mesh. `present` needs about 5 minutes of steady readings (a charger with no cell behind it wanders). Until then `source` is `usb` if the readings already wander, else `unknown` |
| `get_config` | | `config` (section 6) |
| `set_config` | `config`: partial config object | new `config`; applies without a restart, except a mode change |
| `set_mode` | `mode`: `awareness` \| `mesh` | new `config`; the Edge restarts into that mode about 2.5 s later (HTTP sessions end) |
| `set_time` | `unix_ms`, optional `source`, optional `tz_offset_min` | sets wall time (the phone should send it on every connect) and the local time zone shown on the display |
| `wifi_list` / `wifi_scan` | | `known` (no passwords), `nearby` (networks heard in the last 5 min: scans plus sniffed beacons, with `age_s`), `scanning`; `wifi_scan` also starts a scan. Poll `wifi_list` until `scanning` is false (5-20 s). While a client uses the Edge's access point, the scan goes one channel at a time so the client stays connected. |
| `wifi_add` | `ssid`, `password` (empty = open) | |
| `wifi_remove` | `ssid` | |
| `access` | | `pin`, `ap_ssid`, `ap_password` |
| `new_pin` | | `pin`; ends every HTTP session |
| `sync` | `from`, optional `limit` | BLE only. Response has `oldest`, `head`; then records stream on `records`, then a `sync_done` event |
| `sync_confirm` | `seq` | the phone has safely stored every record up to and including `seq` (after its database commit). They are no longer served and their flash is erased in the background (about 1 s per 1,000 records). Result: `deleted_through` (capped at the newest stored seq). `status.log.deleted_after_sync` / `deleting` report progress. |
| `stream_start` | | Bluetooth only, after subscribing to the records characteristic. From now on every new record is sent on the records characteristic (raw 256 bytes, as in `sync`) instead of being stored. Result: `stored_head`, the newest stored seq, so the app can `sync` what was stored before, and `first_seq`, the seq of the first handed-off record. Sent while a hand-off runs, it restarts it (unconfirmed records are stored first). Refused with `the Edge is restarting` once a planned restart has begun. See "Phone hand-off" below. |
| `stream_ack` | `seq` | the app has every record up to and including `seq` |
| `stream_stop` | | ends the hand-off; records not yet confirmed are stored |
| `records` | `from`, `limit ≤ 20` | `records` (JSON views), `next` |
| `log_erase` | `confirm: true` | erases every record in the background (sequence numbers never repeat) |
| `watch` | `kind`: `ble` \| `wifi`, `addr`, `seconds ≤ 600` | streams `rssi` events for one device (proximity check) |
| `unwatch` | | |
| `live` | optional `offset` (default 0), optional `limit` (1-100, default 100) | `rows`: open sightings as `[kind, addr, rssi_last, rssi_max, ago_ms, count]`, strongest `rssi_last` first; `kind` is `ble`, `wifi_ap` or `wifi_probe`; `ago_ms` is how long ago the Edge last heard it, measured on the Edge. Also `total`, `next` (offset of the next page, `null` on the last), `uptime_ms`. Error while awareness is not running (Mesh mode). Needs `caps.live_sightings`. |
| `mesh_send` | `text` (1-200 bytes), `to` (`broadcast` \| `!aabbccdd` \| number), `channel`, `want_ack`, optional `reply_id`, optional `reaction: true` (the text is one emoji reacting to `reply_id`) | `packet_id`; delivery arrives as `mesh_tx` events. Direct messages to a node whose key is known go PKI-encrypted. |
| `mesh_canned` | `index` | sends that quick message on the primary channel; `packet_id` |
| `mesh_traceroute` | `to`, `channel` | `packet_id`; the route arrives as a `mesh_traceroute` event |
| `mesh_request_position` | `to`, `channel` | asks the node for its position |
| `mesh_waypoint` | `lat`, `lon`, `name`, optional `description`, `expire_unix_s`, `icon`, `id` (to update), `channel` | `waypoint_id` |
| `mesh_waypoints` | | `waypoints` received or sent (up to 16) |
| `mesh_store_forward` | optional `router`, `channel` | asks a store & forward router for missed channel messages; error if none was heard |
| `mesh_send_nodeinfo` | | sends node info now |
| `mesh_set_position` | `lat`, `lon`, optional `alt` | the position used in `client` position mode (from the phone) |
| `mesh_node` | `id`, optional `favorite`, optional `ignored` | ignoring also adds the node to `mesh.ignore` in config |
| `mesh_channel_url` | optional `url`, optional `replace` (default true) | `url`: the channels as a Meshtastic link (`https://meshtastic.org/e/#…`). With `url`, applies a link first: `replace` swaps all channels and the LoRa preset, region and hop limit; otherwise its channels are added. |
| `mesh_tak` | | `tak`: ATAK users heard (callsign, uid, team, role, battery, plugin `v1`/`v2`, `compressed`, `reported_position`, `ago_s`) |
| `mesh_tak_chat` | `text` (1-200 bytes), `channel` | sends TAK GeoChat to all chat rooms (TAKPacketV2, port 78) |
| `mesh_tak_position` | `channel` | sends our position as a TAK position report; error when there is no position or the channel's precision is 0 |
| `mesh_nodes` / `mesh_messages` / `mesh_status` | | node table (up to 64, with key, favourite, ignored, device and environment telemetry, neighbours, reported position) / last 32 messages (with `pki`, `reply_id`, `reaction`) / radio state, role, public key, channel use and own airtime, polite hold, store & forward routers heard, paxcounter counts, `admin` (mode, requests, refused), `store_forward_server` (stored, capacity, who it is sending to), `mqtt` (state, broker, uplinked, downlinked, last error), counters |
| `mesh_nodes` with paging | optional `offset` (default 0), optional `limit` (1-64, default 64) | With either parameter: one page of `nodes` in table order, plus `total` and `next` (offset of the next page, `null` on the last), as `live` pages. Without both, the whole table exactly as above. A node first heard between two pages can shift later pages by one. Needs `caps.mesh_nodes_paging`. |
| `mesh_new_identity` | | new random `node_id` and a new key pair |
| `regions` | | region and preset names |
| `ctx_get` / `ctx_set` | `ctx`: object (≤ 2 KB) | Context-page settings, stored on the Edge as an opaque JSON object owned by the web app (area rounded to 0.1°, enabled sources) |
| `diag` | | free heap, largest free block, per-task minimum free stack |
| `pairing_open` | `seconds` (default 120) | |
| `ble_link` | optional `full_size`: bool | Bluetooth only, from a paired phone. With `full_size`, stores whether this phone's links keep 251-byte link-layer packets instead of the 27-byte limit (see "Link-layer packets"); without it, only reads. Result: `full_size` (this phone's setting) and `this_link`: `full_size` \| `limited` (what the current link has, or will have once its 3 s step runs). Removing the phone's bond also clears its setting. Needs `caps.ble_full_size`. |
| `ble_forget_all` | `confirm: true` | removes every bond and reopens pairing |
| `reboot` | | |
| `factory_reset` | `confirm: true` | erases settings, networks, bonds, identity and records, then reboots |

## 3. Events

| ev | When | Fields |
|---|---|---|
| `resp` | reply to a command | see above |
| `status` | every 10 s while a client listens | `status` |
| `obs` | a record was written | WebSocket: a binary frame with the raw record. Bluetooth: `seq`, `type`; read it with `sync` (not sent during a hand-off, when the record itself arrives on the records characteristic). |
| `seen` | a device was first heard in a new sighting window | `kind`, `addr`, `rssi`, `label?`, `uptime_ms`, `body?`: base64 of the record body this window would be written with now (section 4 layout for `kind`, trailing zero bytes trimmed; pad with zeros to decode). Lets the app classify a device when it is first heard. |
| `rssi` | during `watch`, at most 4 Hz | `kind`, `addr`, `rssi`, `channel?`, `uptime_ms`, `unix_ms?` |
| `mode` | a mode change was saved | `mode`, `restarting: true` |
| `stream` | the phone hand-off started or stopped | `state` (`started` / `stopped`), `reason` |
| `mesh_admin` | a remote admin request arrived | `from`, `request` (AdminMessage field), `outcome` (`answered`, `applied`, `refused`, …) |
| `mesh_tak` | a TAK packet arrived | `from` |
| `mesh_rhw` | a remote hardware request arrived | `from`, `request` (HardwareMessage type), `outcome` |
| `sync_done` | end of a BLE sync | `ok`, `sent`, `next`, `head`, `more` |
| `mesh_rx` | text addressed to us or the channel | `msg` |
| `mesh_tx` | our message progressed | `id`, `status`: `sent` \| `delivered` \| `relayed` \| `no_ack` \| `no_relay_heard` \| `failed` (`reason`, Meshtastic routing error) |
| `mesh_node` | node info, position or telemetry received | `node` |
| `mesh_traceroute` | a route check came back | `id`, `to`, `route`, `snr_towards`, `route_back`, `snr_back` (ids, dB; `null` = unknown) |
| `mesh_waypoint` | a waypoint was received | `waypoint` |

Live events are best effort. If a client is slow, the Edge drops events and
counts them (`status.events_dropped`). The record log plus `sync` is the
reliable path, so de-duplicate by `seq`.

## 4. Records

Records live in a flash ring (≈ 15,100 slots on the 3.7 MB `obslog`
partition, oldest overwritten first).
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

**Delete after sync.** The record log is a buffer, not an archive: after a
sync the app sends `sync_confirm` with the newest seq it has committed, and
the Edge erases those records. Sectors (16 records) whose records are all
confirmed are erased; when that includes the sector being written, new
records start in the next sector. The confirmed seq is kept in NVS, so
confirmed records are never served again, even if a restart interrupts the
erase. `status.log.count` counts records actually stored (sequence numbers
have gaps), so use it instead of `head - oldest + 1`.

**Phone hand-off.** While a bonded phone has sent `stream_start`, new records
go to it instead of flash. Each still gets the next `seq`, so `(device_id,
seq)` stays unique and records the phone already has never come back under
another number. The phone confirms with `stream_ack` (cumulative); the Edge
holds up to 32 unconfirmed records in RAM and writes them to flash, oldest
first, when that buffer fills, when a waiting record goes 30 s without a
confirmation (counted from the later of the last `stream_ack` and the moment
it started waiting), or when
the link drops, the app unsubscribes or sends `stream_stop`. Stored `seq`
values therefore still only increase, with gaps where records went to the
phone. A phone should de-duplicate by `seq` (a record confirmed late may also
be stored). Sequence numbers survive restarts: the Edge keeps a floor in NVS,
moved 1,024 ahead each time numbering reaches it. After a restart, numbering
resumes at that floor, so up to 1,024 numbers are skipped, never reused. A
planned restart (`reboot`, a mode change, an update) first writes the
unconfirmed handed-off records to flash and ends the hand-off (`stream` event
`stopped`, reason `Edge restarting`), so `sync` them after reconnecting. About
150 ms later it ends each Bluetooth link (reason 0x13), so the phone sees the
drop at once. A crash, a watchdog reset or a power loss loses those records.
Every restart loses the sightings still open in RAM.

**Records lost at a restart.** `status.log.restart_loss` counts records a
restart lost: numbered, but neither stored nor confirmed with `stream_ack`.
After a planned restart that is at most a record caught mid-write.

```json
"restart_loss": {"since_boot": 57, "records": 3,
                 "ranges": [{"boot": 58, "from": 30650, "to": 30652}]}
```

- `ranges`: the latest 8 restarts that lost records, oldest first. Each holds
  the seqs `from`..`to` numbered during boot `boot` (`info.boot`). Records
  sent but not yet confirmed fall in a range too, so the phone may already
  hold some: count only the seqs it doesn't have. Numbers skipped at the floor
  are never in a range.
- `records`: the total since `since_boot`, including ranges no longer listed.
- `since_boot`: the count lives in RAM that survives a software restart, a
  crash or a watchdog reset, but not a power loss or the reset pin (flashing
  uses it). After those it starts again at the new boot. If `since_boot` is
  later than the last boot the phone saw, the loss at the restart into
  `since_boot` is not known.
- Firmware without the count leaves `restart_loss` out.

The `stream_start` result fixes the boundary exactly. Every record numbered
below `first_seq` is either stored (up to `stored_head`; `sync` it) or was
confirmed by a phone in an earlier hand-off. Numbers between `stored_head`
and `first_seq` are usually gaps, not missing records: they went to a phone
earlier, or were skipped at a restart. From `first_seq` on, handed-off
records arrive one after another (`first_seq`, `first_seq + 1`, ...). A missing
number means the RAM buffer was full and that record went to flash: stop
the hand-off, `sync`, and start again. Do not expect `stored_head + 1` as the
first live record. Sending `stream_start` while a hand-off runs restarts it
with a new `first_seq`, after writing the unconfirmed records to flash.

**Bad packets.** Rarely, the Edge's Bluetooth controller garbles a packet on
the air: about once per 70,000 packets in phone tests, with or without the
27-byte limit. Usually the phone sees the bad packet and drops the link
(Android reports a MIC failure, 0x3D). Once in those tests a record arrived
with a valid link check but a wrong CRC. Check every record's CRC. On a bad
one, drop it, stop the hand-off, `sync`, and start again. The Edge still has
that record, because it was never confirmed.

**Full tables.** The Edge keeps open sightings in RAM tables (128 Bluetooth,
48 access points, 48 probing devices). When a table is full a newcomer still
gets a slot: the sighting with the weakest strongest signal (oldest on a tie)
is pushed out. A pushed-out sighting heard once or for under 5 s is only
counted; once a minute an event record (code 7) gives the counts, and
`status.awareness.brief_counted` keeps the totals since boot.

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

### Type 6: Edge position (reserved)

Reserved for Edges with a GNSS receiver (`caps.gnss`); this firmware never writes it. One fix at
`first_ms`:

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | latitude × 10⁷ (int32) |
| 4 | 4 | longitude × 10⁷ (int32) |
| 8 | 4 | altitude, WGS84, mm (int32; `INT32_MIN` = unknown) |
| 12 | 2 | horizontal accuracy, 0.1 m (`0xFFFF` = unknown) |
| 14 | 2 | vertical accuracy, 0.1 m (`0xFFFF` = unknown) |
| 16 | 1 | satellites used |
| 17 | 1 | fix type: 0 none, 2 2D, 3 3D |
| 18 | 2 | speed, cm/s (`0xFFFF` = unknown) |

### Type 5: Edge event (origin `Edge`)

| Offset | Size | Field |
|---|---|---|
| 0 | 2 | code: 1 boot, 2 mode, 3 time set, 4 log erased, 5 config, 6 firmware, 7 brief sightings counted |
| 2 | 1 | detail length |
| 3 | 221 | detail text |

## 5. `info`

Static for a boot, so it is safe for long BLE reads:

```json
{"protocol":1,"device_id":"9f3c…","name":"Sotreus Edge","vendor":"NetSepio LLC",
 "vendor_url":"https://netsepio.com","product_url":"https://sotreus.com","mdns_host":"sotreus-1a2b.local","suffix":"1A2B","fw":"0.3.0",
 "idf":"v6.0.1","hw":"heltec_wifi_lora_32_v3","boot":12,"record_size":256,"record_format":1,
 "probe_salt":"…","caps":{"ble_observation":true,"wifi24_observation":true,
 "wifi_probe_observation":true,"gnss":false,"local_storage":true,"firmware_switching":false,
 "meshtastic_mode":true,"lora":true,"display":true,"battery":true,"ota_upload":true,
 "live_sightings":true,"placement":true,"ble_full_size":true,"mesh_nodes_paging":true}}
```

`firmware_switching` is false because mesh mode is built in. It is not a
second firmware.

## 6. Config

```json
{
  "mode": "awareness",
  "link": "auto",
  "placement": {"mode": "stationary", "lat": 12.9716, "lon": 77.5946, "accuracy_m": 8,
                "set_unix_ms": 1762400000000, "source": "phone"},
  "awareness": {"ble": true, "ble_active": true, "wifi_aps": true, "wifi_probes": true,
                "probe_names": false, "rssi_floor": -100, "close_after_s": 60, "max_window_s": 300},
  "display": {"timeout_s": 60},
  "mesh": {"region": "UNSET", "preset": "LONG_FAST", "hop_limit": 3, "rebroadcast": "all",
           "tx_power": 0, "channel_num": 0, "node_id": "!5ac1e0d2",
           "long_name": "Sotreus Edge 1A2B", "short_name": "SE2B", "nodeinfo_interval_s": 10800,
           "channels": [{"name": "", "psk": "default", "position_precision": 13}],
           "role": "CLIENT", "licensed": false,
           "position": {"mode": "off", "interval_s": 900},
           "telemetry_interval_s": 1800,
           "neighbor_info": {"enabled": false, "interval_s": 21600},
           "range_test_interval_s": 0,
           "notify": {"mode": "off", "gpio": -1},
           "detection": {"enabled": false, "gpio": -1, "active_high": false, "min_interval_s": 45, "name": ""},
           "paxcounter": {"enabled": false, "interval_s": 900},
           "serial": {"enabled": false, "rx": -1, "tx": -1, "baud": 115200},
           "canned": ["OK", "On my way", "Need assistance", "All fine here"],
           "lora": {"freq_offset_hz": 0, "override_freq_mhz": 0, "tx_enabled": true, "override_duty_cycle": false,
                    "rx_boosted": true, "ignore_mqtt": false, "ok_to_mqtt": false},
           "ignore": [],
           "admin": {"enabled": true, "mode": "open", "admin_channel": false, "keys": []},
           "remote_hardware": {"enabled": false, "allow_undefined_pins": false,
                               "pins": [{"gpio": 47, "name": "relay", "type": "write"}]},
           "store_forward": {"server": false, "heartbeat": true, "return_max": 25, "return_window_min": 240},
           "mqtt": {"enabled": false, "address": "mqtt.meshtastic.org", "username": "meshdev", "password_set": true,
                    "encryption": true, "json": false, "tls": false, "root": "", "proxy_to_client": false,
                    "map": {"enabled": false, "interval_s": 3600, "precision": 14, "location": false}},
           "tak": {"team": "default", "role": "default"},
           "audio": {"enabled": false, "ptt_gpio": -1, "bitrate": "default",
                     "i2s_ws": -1, "i2s_sd": -1, "i2s_din": -1, "i2s_sck": -1}}
}
```

Each channel also has `uplink` and `downlink` (MQTT). The MQTT password is
write-only: send `password` to change it; reads show `password_set`.

* `link` is one of `auto`, `ap`, `sta` or `off`.
* `placement` says how the owner uses the Edge: `travel` (carried; the phone tags sightings
  with its own location while they are together) or `stationary` (left in one place at
  `lat`/`lon` ± `accuracy_m`, set from the phone (`source: phone`), a map (`map`) or a GNSS
  receiver (`gnss`)). The Edge only stores it, in its own NVS key `place`: it does not use it,
  show it, or send it over the mesh (the mesh position is `mesh.position`). In `set_config`,
  `lat`, `lon` and `accuracy_m` go together; `"lat": null` clears the position. Without a
  position the object has only `mode`. Needs `caps.placement`.
* `mesh.region` is a Meshtastic region name, or `UNSET`. While it is
  `UNSET`, the radio never transmits. The regions match Meshtastic 2.7 on
  sub-GHz: `US`, `EU_433`, `EU_868`, `CN`, `JP`, `ANZ`, `KR`, `TW`, `RU`, `IN`,
  `NZ_865`, `TH`, `UA_433`, `UA_868`, `MY_433`, `MY_919`, `SG_923`, `PH_433`,
  `PH_868`, `PH_915`, `ANZ_433`, `KZ_433`, `KZ_863`, `NP_865` and `BR_902`
  (`regions` lists them). `LORA_24` needs a 2.4 GHz radio.
* `mesh.preset` is one of `LONG_FAST`, `LONG_SLOW`, `LONG_MODERATE`,
  `LONG_TURBO`, `MEDIUM_SLOW`, `MEDIUM_FAST`, `SHORT_SLOW`, `SHORT_FAST` or
  `SHORT_TURBO`. A preset wider than the region's band (the 500 kHz
  `LONG_TURBO` and `SHORT_TURBO` in `EU_868`) is kept as set but the radio uses
  `LONG_FAST`, as Meshtastic does; `status.mesh` (and `mesh_status`) then
  name the set preset in `preset_too_wide`, and `preset` is the one in use.
* A channel `psk` is `default` (Meshtastic `AQ==`), `none`, `simpleN`, or
  `base64:<16 or 32 bytes>`. An empty primary channel name means the preset
  name (`LongFast`), which matches Meshtastic's public default channel. Up to
  8 channels.
* `position_precision` (0-32 bits) is how precisely a channel gets this
  node's position: 0 never, 32 exact, 13 about ±2.9 km (the default on the
  default-key channel).
* `rebroadcast` is `all`, `all_skip_decoding`, `local_only`, `known_only`,
  `none` or `core_portnums_only`. `role` is a Meshtastic role name
  (`CLIENT`, `CLIENT_MUTE`, `CLIENT_HIDDEN`, `CLIENT_BASE`, `ROUTER`,
  `ROUTER_LATE`, `REPEATER`, `TRACKER`, `TAK`, `TAK_TRACKER`, `SENSOR`,
  `LOST_AND_FOUND`). Relaying follows Meshtastic 2.7: `ROUTER` relays early
  and always; `ROUTER_LATE` relays in the client window and always, moving to
  the late window when another node relays first; `CLIENT_BASE` does the same
  for packets from or to a favourite and relays the rest like `CLIENT`. A
  `ROUTER`, `ROUTER_LATE` or `CLIENT_BASE` keeps the hop limit when the node
  that just relayed is a favourite with one of those roles (never on the first
  hop). A contact added by remote admin becomes a favourite, except on a
  `CLIENT_BASE`.
* `position.mode` is `off`, `fixed` (with `lat`, `lon`, `alt`) or `client`
  (the phone hands it over with `mesh_set_position`). The Edge has no GNSS.
* `licensed: true` turns off all encryption (amateur radio rules); the long
  name should then be the call sign.
* Settings saved by firmware before config version 2 are migrated on boot.
* `admin.mode` is `open` (any node; the default the owner chose, with a
  warning on the setup page) or `keys` (only PKI messages from one of up to 3
  listed public keys). `admin_channel: true` also accepts Meshtastic's legacy
  channel named `admin`. Remote hardware follows the same rule and also
  accepts the legacy channel named `gpio`. GPIOs the board uses are refused.
* The module settings (`admin`, `remote_hardware`, `store_forward`, `mqtt`,
  `audio`, channel `uplink`/`downlink`) are stored apart from the main
  config (NVS blob `meshmod`).

## 7. Meshtastic compatibility

On air the Edge matches Meshtastic 2.x: region frequency slots (djb2 of the
channel name), sync word 0x2B, 16-symbol preamble, the 16-byte header with
next-hop and relay-node bytes, AES-CTR with the channel key and nonce
`packet_id(u64 LE) ‖ from(u32 LE) ‖ 0`, and the channel hash. Other nodes see
firmware level `2.5.0.5e0e` in its device metadata.

Covered:

* **Messages.** Text on up to 8 channels, direct messages with ACK and up to
  3 retries (the last one floods), replies and emoji reactions, quick
  (canned) messages from the page or the PRG button.
* **PKI.** An X25519 key pair is made on first boot and sent in node info.
  Direct messages to and from nodes whose key is known use
  `SHA-256(X25519)` keys and AES-CCM (8-byte tag, 4-byte extra nonce).
* **Routing.** Managed flooding with Meshtastic's SNR-weighted contention
  window, next-hop learning from ACKs, every 2.7 role (including
  `CLIENT_BASE`) and rebroadcast mode, hop limits kept between favourite
  routers, the
  ignore list, duty-cycle limits and a polite hold while channel use is above
  25 % or own airtime is above half the limit.
* **Node info, position and telemetry.** Node info with key and role,
  position (fixed or from the phone, truncated per channel), device metrics,
  and decoding of others' device and environment telemetry.
* **Remote admin (port 6).** Get and set owner, channels, config (device,
  position, power, network, display, LoRa, Bluetooth, security, session key)
  and module config (MQTT, serial, external notification, store & forward,
  range test, telemetry, canned messages, audio, remote hardware, neighbour
  info, ambient lighting, detection sensor, paxcounter), canned messages,
  device metadata, connection status, remote hardware pins, ham mode, fixed
  position, time, favourites, ignored nodes, node removal, shared contacts,
  node-database reset, reboot, factory resets. Meshtastic 2.5 session
  passkeys are required for every change (valid 300 s, refreshed every
  150 s). Who may send is set by `admin.mode` (section 6). The private key and
  the Wi-Fi password are never sent; the MQTT password and channel keys only
  to a listed admin key. Network, security, MQTT, channel and remote hardware
  changes, ham mode and factory resets always need a listed key (or the
  admin channel). Changes apply 3 s after the reply, without a restart.
* **Remote hardware (port 2).** Read, write and watch GPIOs listed in
  settings (or any free GPIO when allowed); board pins are always refused.
* **Store & forward.** Client (asks a router for history) and server: text on
  readable channels, and the Edge's own channel messages, are kept in the
  256 KB `sfdb` flash ring (about 1,000 messages, survives restarts). The
  server answers `CLIENT_HISTORY` (and a direct "SF" text), `CLIENT_PING`,
  `CLIENT_STATS` and `CLIENT_ABORT`, replays up to `return_max` messages from
  the last `return_window_min` minutes one every 5 s under their original
  sender and id, answers `ROUTER_BUSY` while serving someone else, and sends a
  heartbeat every 15 minutes.
* **MQTT.** While the Edge has joined Wi-Fi in Mesh mode: Meshtastic topics
  `<root>/2/e/<channel>/<!gateway>` with `ServiceEnvelope`, uplink of packets
  it sends and hears on uplink channels (others' only when they set OK to
  MQTT), downlink into the mesh marked `via_mqtt` (hop limit capped to 1 on
  the public broker), PKI topic, optional JSON uplink (not to the public
  broker) and JSON `sendtext` downlink, map reports on `<root>/2/map/`, TLS
  with the common CA bundle. Root defaults to `msh/<region>`.
* **Modules.** Neighbour info, waypoints, range test, paxcounter (counts
  only), detection sensor (GPIO), serial text bridge (UART1), external
  notification (GPIO or LED), lost-and-found beacon.
* **ATAK.** TAKPacket (port 72) and TAKPacketV2 (port 78, flags byte, `0xFF`
  = uncompressed) are decoded for the ATAK user list and GeoChat; compressed
  ones are relayed but not read. `TAK_TRACKER` sends its position as a TAK v2
  position report (CoT `a-f-G-U-C`, how `h-e` for a fixed position, `m-g`
  for one from the phone, uid `!<node id>`) at channel 0's position
  precision, with `TAKConfig` team and role (defaults Cyan, Team Member).
* **Channel links.** Export and import of `https://meshtastic.org/e/#…`,
  including MQTT uplink/downlink flags.
* **Persistence.** Nodes, messages and waypoints are saved to the `meshdb`
  partition every 15 minutes and before a restart.

Not covered:

* The Meshtastic phone API (`ToRadio`/`FromRadio` over Bluetooth or TCP), so
  the Meshtastic app cannot connect directly; remote admin from another node
  works.
* Audio: settings only, until a microphone, speaker, I2S codec and the
  codec2 library are added. Received voice frames are counted.
* Running ATAK itself: an ATAK phone still needs the Meshtastic Android app
  (phone API) or the Sotreus app to bridge it to the mesh. Compressed TAK
  packets (unishox2 / zstd) are relayed but not decompressed.
* Custom LoRa modem parameters (bandwidth, spreading factor, coding rate
  outside the presets), OTA over the mesh, files, displays and sensors the
  Heltec V3 does not have.
* Radio settings that exist only in the Meshtastic 2.8 pre-releases: the
  `LITE_*`, `NARROW_*`, `TINY_*` and `MEDIUM_TURBO` presets, the `EU_866`,
  `EU_N_868` and ITU amateur-radio regions, and 2.8's per-region preset lists.
* Newer Meshtastic features: XEdDSA packet signatures and the signature
  policy, key verification (port 12), status messages (port 36), mesh beacons
  (37), paging (38), critical alerts (port 11), ack proofs, waypoint geofence
  alerts, and telemetry other than device metrics (air quality, power,
  health, local stats). The status message and traffic management module
  settings are accepted but have no effect. No `PKI_UNKNOWN_PUBKEY` error is
  sent when a direct message cannot be decrypted.
* Partly covered: the serial bridge has text mode only; notifications drive
  the LED or one GPIO (no buzzer, vibration or ringtone); no power saving,
  smart position broadcast, external GPS or I2C sensors; no NTP server
  choice, static IP, syslog or mesh over local Wi-Fi; no settings backup or
  restore; "mute node" is accepted but has no effect.

Its node number is random, not derived from a hardware address.

## 8. Context feeds (setup page)

The Context tab runs entirely in the browser. The Edge stores only the
page's settings (`ctx_set`) and makes no internet requests for it.

| Feed | Fetched from | What it is sent |
|---|---|---|
| Map tiles | OpenFreeMap | the tiles in view |
| Earthquakes | USGS | nothing (world list, filtered in the browser) |
| Natural events | NASA EONET | nothing (world list) |
| Air quality | Open-Meteo | a 0.5° or 0.1° cell centre |
| Satellites | CelesTrak | group names |
| Warnings, United States | NWS | the state code |
| Approximate area | GeoJS | the browser's IP address, only on request |
| Aircraft | adsb.lol, through the relay | a 0.5° grid point and a radius |
| Disaster alerts | GDACS, through the relay | nothing (world list) |
| Warnings, Europe | MeteoAlarm, through the relay | the country name |

adsb.lol, GDACS and MeteoAlarm do not let a web page read them (no CORS
headers, or a bot check). The browser reaches them through a relay you set,
a small Cloudflare Worker in `tools/context-relay/`. With no relay set,
those cards explain why they are empty and nothing is sent. The browser can
read the device's own location only over HTTPS, which the Edge does not
serve, so the area is typed, picked on the map, or taken from the IP address.
