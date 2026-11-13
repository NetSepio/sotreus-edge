#!/usr/bin/env python3
"""Mock Sotreus Edge for developing the setup page without a board.

Serves main/web/index.html and the bundled assets, and answers the HTTP API
with plausible, stateful data, including binary records (/api/records?fmt=bin)
in the firmware's 256-byte layout and the full Mesh command set. Context
sources are fetched by the browser itself, as on the real Edge.

    python3 tools/mock_edge.py [--port 8080] [--link connected|ap] [--mode awareness|mesh]

Login PIN: 12345678. Listens on 127.0.0.1 only.
"""
import argparse
import json
import secrets
import struct
import time
import urllib.parse
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent / "main" / "web"
ASSETS = {
    "maplibre-gl-4.7.1.js": ("vendor/maplibre-gl-4.7.1.js", "text/javascript"),
    "maplibre-gl-4.7.1.css": ("vendor/maplibre-gl-4.7.1.css", "text/css"),
    "uplot-1.6.31.min.js": ("vendor/uplot-1.6.31.min.js", "text/javascript"),
    "uplot-1.6.31.min.css": ("vendor/uplot-1.6.31.min.css", "text/css"),
    "satellite-5.0.0.min.js": ("vendor/satellite-5.0.0.min.js", "text/javascript"),
    "geist-regular.woff2": ("fonts/geist_regular.woff2", "font/woff2"),
    "geist-medium.woff2": ("fonts/geist_medium.woff2", "font/woff2"),
    "geist-mono-regular.woff2": ("fonts/geist_mono_regular.woff2", "font/woff2"),
    "instrument-serif-regular.woff2": ("fonts/instrument_serif_regular.woff2", "font/woff2"),
}
PIN = "12345678"
START = time.time()
TOKENS = set()
STATE = {
    "ctx": {},
    "config": {
        "mode": "awareness", "link": "auto",
        "placement": {"mode": "travel"},
        "awareness": {"ble": True, "ble_active": True, "wifi_aps": True, "wifi_probes": True, "probe_names": False,
                      "rssi_floor": -100, "close_after_s": 60, "max_window_s": 300},
        "display": {"timeout_s": 60},
        "mesh": {"region": "UNSET", "preset": "LONG_FAST", "hop_limit": 3, "rebroadcast": "all", "tx_power": 0,
                 "channel_num": 0, "node_id": "!5f3bb481", "long_name": "Sotreus Edge 96FF", "short_name": "SEFF",
                 "nodeinfo_interval_s": 10800, "channels": [{"name": "", "psk": "default", "position_precision": 13, "uplink": False, "downlink": False}],
                 "role": "CLIENT", "licensed": False, "position": {"mode": "off", "interval_s": 900}, "telemetry_interval_s": 1800,
                 "neighbor_info": {"enabled": False, "interval_s": 21600}, "range_test_interval_s": 0,
                 "notify": {"mode": "off", "gpio": -1},
                 "detection": {"enabled": False, "gpio": -1, "active_high": False, "min_interval_s": 45, "name": ""},
                 "paxcounter": {"enabled": False, "interval_s": 900},
                 "serial": {"enabled": False, "rx": -1, "tx": -1, "baud": 115200},
                 "canned": ["OK", "On my way", "Need assistance", "All fine here"],
                 "lora": {"freq_offset_hz": 0, "override_freq_mhz": 0, "tx_enabled": True, "override_duty_cycle": False,
                          "rx_boosted": True, "ignore_mqtt": False, "ok_to_mqtt": False},
                 "ignore": [],
                 "admin": {"enabled": True, "mode": "open", "admin_channel": False, "keys": []},
                 "remote_hardware": {"enabled": False, "allow_undefined_pins": False, "pins": []},
                 "store_forward": {"server": False, "heartbeat": True, "return_max": 25, "return_window_min": 240},
                 "mqtt": {"enabled": False, "address": "mqtt.meshtastic.org", "username": "meshdev", "password_set": True,
                          "encryption": True, "json": False, "tls": False, "root": "", "proxy_to_client": False,
                          "map": {"enabled": False, "interval_s": 3600, "precision": 14, "location": False}},
                 "tak": {"team": "default", "role": "default"},
                 "audio": {"enabled": False, "ptt_gpio": -1, "bitrate": "default", "i2s_ws": -1, "i2s_sd": -1, "i2s_din": -1, "i2s_sck": -1}},
    },
    "known": [],
    "nodes": [
        {"id": "!a1b2c3d4", "num": 2712847316, "long_name": "Ridge relay", "short_name": "RDG", "hw_model": 43, "role": "ROUTER",
         "has_public_key": True, "public_key": "q83vEjRWeJq83vEjRWeJq83vEjRWeJq83vEjRWeJq80=", "favorite": False, "ignored": False, "last_heard_ago_s": 40, "rssi": -96, "snr": 4.5, "hops_away": 0,
         "device": {"battery_level": 101, "channel_util": 6.2, "air_util_tx": 0.8, "uptime_s": 86400},
         "neighbors": [{"id": "!0c0ffee1", "snr": 6.25}]},
        {"id": "!0c0ffee1", "num": 202375137, "long_name": "Valley base", "short_name": "VLY", "hw_model": 9, "role": "CLIENT",
         "has_public_key": False, "favorite": True, "ignored": False, "last_heard_ago_s": 600, "rssi": -118, "snr": -7.25, "hops_away": 1,
         "reported_position": {"lat": 12.95, "lon": 77.6, "alt_m": 900, "precision_bits": 13, "provenance": "NETWORK"},
         "environment": {"temperature_c": 24.5, "humidity_pct": 61, "pressure_hpa": 912}},
    ],
    "messages": [
        {"id": 11, "from": "!a1b2c3d4", "to": "broadcast", "channel": 0, "dir": "received", "provenance": "NETWORK", "pki": False,
         "ago_s": 300, "rssi": -96, "snr": 4.5, "text": "Hello from the ridge"},
        {"id": 12, "from": "!0c0ffee1", "to": "!5f3bb481", "channel": 0, "dir": "received", "provenance": "NETWORK", "pki": True,
         "ago_s": 120, "rssi": -118, "snr": -7.25, "text": "Are you on the hill today?"},
        {"id": 13, "from": "!0c0ffee1", "to": "broadcast", "channel": 0, "dir": "received", "provenance": "NETWORK", "pki": False,
         "ago_s": 100, "rssi": -118, "snr": -7.25, "reply_id": 11, "reaction": True, "text": "\U0001F44D"},
    ],
    "waypoints": [{"id": 77, "from": "!0c0ffee1", "name": "Water point", "description": "Tap by the gate", "lat": 12.9512, "lon": 77.6021,
                   "expire_unix_s": int(time.time()) + 86400, "icon": 0, "unix_s": int(time.time()) - 900, "provenance": "NETWORK"}],
    "next_id": 1000,
    "tak": [{"node": "!0c0ffee1", "callsign": "VALLEY-2", "uid": "ANDROID-5f2c", "team": "Cyan", "role": "Team Lead", "battery": 64,
             "plugin": "v2", "compressed": False, "ago_s": 95,
             "reported_position": {"lat": 12.9512, "lon": 77.6033, "alt_m": 905, "speed_mps": 0, "course_deg": 0, "provenance": "NETWORK"}},
            {"node": "!a1b2c3d4", "callsign": "", "uid": "", "team": "", "role": "", "plugin": "v2", "compressed": True, "ago_s": 300}],
}
REGIONS = ["US", "EU_433", "EU_868", "CN", "JP", "ANZ", "KR", "TW", "RU", "IN", "NZ_865", "TH", "UA_433", "UA_868",
           "MY_433", "MY_919", "SG_923", "PH_433", "PH_868", "PH_915", "ANZ_433", "KZ_433", "KZ_863", "NP_865", "BR_902"]
PRESETS = ["LONG_FAST", "LONG_SLOW", "MEDIUM_SLOW", "MEDIUM_FAST", "SHORT_SLOW", "SHORT_FAST", "LONG_MODERATE",
           "SHORT_TURBO", "LONG_TURBO"]


def deep_merge(dst, src):
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            deep_merge(dst[k], v)
        else:
            dst[k] = v


def status(args):
    cfg = STATE["config"]
    mesh_on = cfg["mode"] == "mesh"
    sta = args.link == "connected"
    up = int((time.time() - START) * 1000)
    region = None if cfg["mesh"]["region"] == "UNSET" else cfg["mesh"]["region"]
    m = {"radio_present": True, "running": mesh_on, "node_id": cfg["mesh"]["node_id"], "node_num": 1597748353,
         "region": region, "preset": cfg["mesh"]["preset"], "nodes": 2 if mesh_on else 0,
         "counters": {"rx": 5, "tx": 2, "relayed": 1, "rx_bad": 0, "rx_other_channel": 0, "rx_duplicate": 0,
                      "relay_skipped": 0, "tx_dropped": 0, "preambles": 7, "headers": 6},
         "channels": [{"index": 0, "name": "LongFast", "hash": 8, "encrypted": True}],
         "role": cfg["mesh"]["role"], "public_key": "6TvroVZE2KjHkif1q0e3S1dYc2nq3nYyD2Oc4pGXr0I=",
         "channel_util_pct": 4.3, "air_util_tx_pct": 0.12, "duty_cycle_pct": 10, "polite_hold": False, "store_forward_routers": 1,
         "admin": {"enabled": cfg["mesh"]["admin"]["enabled"], "mode": cfg["mesh"]["admin"]["mode"], "keys": len(cfg["mesh"]["admin"]["keys"]), "requests": 2, "refused": 1},
         "store_forward_server": {"enabled": cfg["mesh"]["store_forward"]["server"], "storage": True, "stored": 12, "capacity": 1008, "history_requests": 1},
         "mqtt": {"state": "connected" if cfg["mesh"]["mqtt"]["enabled"] else "off", "broker": cfg["mesh"]["mqtt"]["address"], "tls": cfg["mesh"]["mqtt"]["tls"],
                  "uplinked": 14, "downlinked": 3, "dropped": 0, "connects": 1}}
    if region:
        m["freq_mhz"] = 865.875
        if mesh_on:
            m["radio"] = {"chip_mode": "rx", "rssi_now": -108}
    return {
        "uptime_ms": up, "time_valid": True, "time_source": "browser", "unix_ms": int(time.time() * 1000),
        "mode": cfg["mode"], "battery": {"present": False, "source": "usb", "mv": 4180},
        "heap_free": 55400, "heap_min": 48900,
        "link": {"mode": cfg["link"], "state": "connected" if sta else "ap",
                 "ssid": "HomeNet" if sta else "Sotreus-Edge-96FF", "ip": "192.168.1.42" if sta else "192.168.4.1",
                 "stations": 0 if sta else 1, "channel": 6, "sniffing": not mesh_on, "hopping": False, "rssi": -58},
        "ble": {"connections": 0, "secured": 0, "bonds": 1, "pairing_open": False, "scanning": not mesh_on},
        "awareness": {"running": not mesh_on, "open_sightings": {"ble": 0 if mesh_on else 31, "wifi_ap": 0 if mesh_on else 24,
                      "wifi_probe": 0 if mesh_on else 9}, "ble_reports": 2400, "wifi_frames": 400, "records": 170, "queue_drops": 0},
        "log": {"head": 230, "oldest": 1, "count": 230, "capacity": 15616, "erasing": False, "write_failures": 0},
        "mesh": m, "events_dropped": 0,
    }


def record(seq):
    now = int(time.time() * 1000)
    kinds = [
        {"type": "ble", "origin": "EdgeBle", "addr": "4c:9a:21:0e:77:%02x" % (seq % 256), "addr_type": "random",
         "name": "Fitness band", "rssi": {"last": -71, "max": -62, "min": -80}, "count": 140, "adv": "0201061aff4c00"},
        {"type": "wifi_ap", "origin": "EdgeWifi", "bssid": "24:2f:d0:bd:0f:%02x" % (seq % 256), "ssid": "SquareOne",
         "channel": 6, "auth": "wpa2", "rssi": {"last": -72, "max": -70, "min": -76}, "count": 30},
        {"type": "wifi_probe", "origin": "EdgeWifi", "mac": "ca:2a:f0:e6:b6:%02x" % (seq % 256), "randomized": True,
         "channel": 1, "rssi": {"last": -52, "max": -44, "min": -60}, "count": 4, "ssid_hashes": ["9a1b2c3d"]},
    ]
    r = dict(kinds[seq % 3])
    r.update({"seq": seq, "provenance": "SENSED", "time_valid": True, "first_unix_ms": now - 120000 - seq * 1000,
              "last_unix_ms": now - 60000 - seq * 1000, "boot": 9})
    return r


AUTH_CODE = {"open": 0, "wep": 1, "wpa": 2, "wpa2": 3, "wpa3": 4, "wpa2_wpa3": 5, "enterprise": 6, "owe": 7}
REC_TYPE = {"ble": 1, "wifi_ap": 2, "wifi_probe": 3}


def mac_bytes(m):
    return bytes(int(x, 16) for x in m.split(":"))


def record_bin(seq):
    """The same synthetic record in the firmware's 256-byte layout (see edge_records.h)."""
    r = record(seq)
    rs = r["rssi"]
    rssi = struct.pack("<bbb", rs["last"], rs["max"], rs["min"])
    body = bytearray(228)
    if r["type"] == "ble":
        adv = bytes.fromhex("020106")
        name = r["name"].encode()
        adv += bytes([len(name) + 1, 0x09]) + name
        body[0:6] = mac_bytes(r["addr"])
        body[6] = 1
        body[8:11] = rssi
        body[11] = 4
        body[12:16] = struct.pack("<I", r["count"])
        body[16] = len(adv)
        body[18:18 + len(adv)] = adv
    elif r["type"] == "wifi_ap":
        ssid = r["ssid"].encode()
        body[0:6] = mac_bytes(r["bssid"])
        body[6] = r["channel"]
        body[7:10] = rssi
        body[10] = AUTH_CODE[r["auth"]]
        body[12:16] = struct.pack("<I", r["count"])
        body[16:18] = struct.pack("<H", 100)
        body[18] = len(ssid)
        body[19:19 + len(ssid)] = ssid
    else:
        body[0:6] = mac_bytes(r["mac"])
        body[6] = r["channel"]
        body[7:10] = rssi
        body[10] = 1
        body[11] = len(r["ssid_hashes"])
        body[12:16] = struct.pack("<I", r["count"])
        for i, h in enumerate(r["ssid_hashes"]):
            body[16 + 4 * i:20 + 4 * i] = struct.pack("<I", int(h, 16))
    body = bytes(body[:224])
    body_len = {"ble": 80, "wifi_ap": 172, "wifi_probe": 164}[r["type"]]
    head = struct.pack("<IBBHqqHH", seq, REC_TYPE[r["type"]], 1, r["boot"], r["first_unix_ms"], r["last_unix_ms"],
                       body_len, 1)
    rec = head + body
    return rec + struct.pack("<I", zlib.crc32(rec) & 0xFFFFFFFF)


def live_rows(offset, limit):
    """Open sightings for the `live` command, strongest first."""
    rows = []
    for seq in range(200, 230):
        r = record(seq)
        addr = (r.get("addr") or r.get("bssid") or r.get("mac")).upper()
        rows.append([r["type"], addr, r["rssi"]["last"], r["rssi"]["max"], (seq * 1700) % 58000, r["count"]])
    rows.sort(key=lambda x: -x[2])
    limit = max(1, min(int(limit), 100))
    page = rows[offset:offset + limit]
    nxt = offset + limit if offset + limit < len(rows) else None
    return page, len(rows), nxt


def node_by_id(nid):
    return next((n for n in STATE["nodes"] if n["id"] == nid), None)


def handle_cmd(req, args):
    c = req.get("cmd")
    cfg = STATE["config"]
    ok = {"ev": "resp", "ok": True}
    if c == "info":
        return {**ok, "info": {"protocol": 1, "device_id": "9f3c00aa11bb22cc", "name": "Sotreus Edge", "vendor": "NetSepio LLC",
                               "vendor_url": "https://netsepio.com", "product_url": "https://sotreus.com",
                               "mdns_host": "sotreus-96ff.local", "suffix": "96FF", "fw": "0.3.1", "idf": "v6.0.1",
                               "hw": "heltec_wifi_lora_32_v3", "boot": 9, "record_size": 256, "record_format": 1,
                               "probe_salt": "00",
                               "caps": {"ble_observation": True, "wifi24_observation": True, "wifi_probe_observation": True,
                                        "gnss": False, "local_storage": True, "firmware_switching": False,
                                        "meshtastic_mode": True, "lora": True, "display": True, "battery": True,
                                        "ota_upload": True, "live_sightings": True, "placement": True,
                                        "ble_full_size": True, "mesh_nodes_paging": True,
                                        "mesh_node_remove": True,
                                        "mesh_target_fields": True}}}
    if c == "status":
        return {**ok, "status": status(args)}
    if c == "live":
        if cfg["mode"] != "awareness":
            return {"ev": "resp", "ok": False, "error": "not listening (Mesh mode or stopped)"}
        page, total, nxt = live_rows(int(req.get("offset", 0)), req.get("limit", 100))
        return {**ok, "rows": page, "total": total, "next": nxt, "uptime_ms": int((time.time() - START) * 1000)}
    if c == "get_config":
        return {**ok, "config": cfg}
    if c in ("set_config", "set_mode"):
        patch = req if c == "set_mode" else req.get("config", {})
        patch = {k: v for k, v in patch.items() if k not in ("cmd", "id")}
        if patch.get("mode") == "mesh" and cfg["mesh"]["region"] == "UNSET" and not (patch.get("mesh") or {}).get("region"):
            pass  # the firmware allows it (radio stays silent); the page asks for a region first
        mq = ((patch.get("mesh") or {}).get("mqtt") or {})
        if "password" in mq:
            mq["password_set"] = bool(mq.pop("password"))
        deep_merge(cfg, patch)
        return {**ok, "config": cfg}
    if c == "ctx_get":
        return {**ok, "ctx": STATE["ctx"]}
    if c == "ctx_set":
        STATE["ctx"] = req.get("ctx", {})
        return ok
    if c == "regions":
        return {**ok, "regions": REGIONS, "presets": PRESETS}
    if c in ("wifi_list", "wifi_scan"):
        near = [{"ssid": "SquareOne", "rssi": -61, "channel": 6, "auth": "wpa2", "age_s": 3, "known": False},
                {"ssid": "HomeNet", "rssi": -58, "channel": 11, "auth": "wpa2_wpa3", "age_s": 1, "known": True}]
        return {**ok, "known": [{"ssid": s, "has_password": True} for s in STATE["known"]], "nearby": near, "scanning": False}
    if c == "wifi_add":
        STATE["known"].append(req.get("ssid"))
        return ok
    if c == "wifi_remove":
        STATE["known"] = [s for s in STATE["known"] if s != req.get("ssid")]
        return ok
    if c == "access":
        return {**ok, "pin": PIN, "ap_ssid": "Sotreus-Edge-96FF", "ap_password": "k7mq2xr9wd4p"}
    if c == "records":
        frm = int(req.get("from", 0))
        n = min(int(req.get("limit", 20)), 20)
        recs = [record(s) for s in range(max(frm, 1), min(frm + n, 231))]
        return {**ok, "records": recs, "next": (recs[-1]["seq"] + 1) if recs else frm}
    mesh_on = cfg["mode"] == "mesh" and cfg["mesh"]["region"] != "UNSET"
    if c == "mesh_status":
        return {**ok, "mesh": status(args)["mesh"]}
    if c == "mesh_nodes":
        if "offset" not in req and "limit" not in req:
            return {**ok, "nodes": STATE["nodes"]}
        off = max(int(req.get("offset", 0)), 0)
        lim = int(req.get("limit", 64))
        lim = lim if 1 <= lim <= 64 else 64
        nodes = STATE["nodes"]
        return {**ok, "nodes": nodes[off:off + lim], "total": len(nodes),
                "next": off + lim if off + lim < len(nodes) else None}
    if c == "mesh_messages":
        return {**ok, "messages": STATE["messages"]}
    if c == "sync_confirm":
        return {**ok, "deleted_through": int(req.get("seq", 0))}
    if c == "mesh_tak":
        return {**ok, "tak": STATE["tak"]}
    if c in ("mesh_tak_chat", "mesh_tak_position") and mesh_on:
        if c == "mesh_tak_chat":
            STATE["next_id"] += 1
            STATE["messages"].append({"id": STATE["next_id"], "from": cfg["mesh"]["node_id"], "to": "broadcast", "channel": 0, "dir": "sent",
                                      "provenance": "NETWORK", "pki": False, "ago_s": 0, "status": "sent",
                                      "text": "[TAK] " + cfg["mesh"]["long_name"] + ": " + req.get("text", "")})
        return ok
    if c == "mesh_waypoints":
        return {**ok, "waypoints": STATE["waypoints"]}
    if c.startswith("mesh_") and c not in ("mesh_node", "mesh_channel_url", "mesh_new_identity") and not mesh_on:
        return {"ev": "resp", "ok": False, "error": "mesh is not running or the LoRa region is not set"}
    if c == "mesh_send":
        STATE["next_id"] += 1
        m = {"id": STATE["next_id"], "from": cfg["mesh"]["node_id"], "to": req.get("to", "broadcast"), "channel": req.get("channel", 0),
             "dir": "sent", "provenance": "NETWORK", "pki": req.get("to", "broadcast") != "broadcast" and bool((node_by_id(req.get("to")) or {}).get("has_public_key")),
             "ago_s": 0, "status": "sent", "text": req.get("text", "")}
        if req.get("reply_id"):
            m["reply_id"] = req["reply_id"]
        if req.get("reaction"):
            m["reaction"] = True
        STATE["messages"].append(m)
        return {**ok, "packet_id": m["id"]}
    if c == "mesh_canned":
        return {**ok, "packet_id": 1235}
    if c in ("mesh_traceroute", "mesh_request_position", "mesh_send_nodeinfo", "mesh_set_position", "mesh_store_forward"):
        return {**ok, "packet_id": 1236} if c == "mesh_traceroute" else ok
    if c == "mesh_waypoint":
        wid = int(req["waypoint_id"] if "waypoint_id" in req else req.get("id", 0))  # `id` only for older clients
        STATE["waypoints"] = [w for w in STATE["waypoints"] if not wid or w["id"] != wid]
        if not wid:
            STATE["next_id"] += 1
            wid = STATE["next_id"]
        STATE["waypoints"].append({"id": wid, "from": cfg["mesh"]["node_id"], "name": req.get("name", "Waypoint"),
                                   "description": req.get("description", ""), "lat": req["lat"], "lon": req["lon"],
                                   "expire_unix_s": req.get("expire_unix_s", 0), "icon": 0, "unix_s": int(time.time()), "provenance": "NETWORK"})
        return {**ok, "waypoint_id": wid}
    if c == "mesh_node":
        nid = req["node"] if "node" in req else req.get("id")  # `id` only for older clients
        n = node_by_id(nid)
        if req.get("remove") is True:
            if not mesh_on:
                return {"ev": "resp", "ok": False, "error": "mesh is not running"}
            if nid == cfg["mesh"]["node_id"]:
                return {"ev": "resp", "ok": False, "error": "this is the Edge's own node"}
            if not n:
                return {"ev": "resp", "ok": False, "error": "node not found"}
            STATE["nodes"].remove(n)
            n = None
        if req.get("favorite") is not None and n:
            n["favorite"] = bool(req["favorite"])
        if req.get("ignored") is not None:
            ig = [x for x in cfg["mesh"]["ignore"] if x != nid]
            if req["ignored"]:
                ig.append(nid)
            cfg["mesh"]["ignore"] = ig
            if n:
                n["ignored"] = bool(req["ignored"])
        return ok
    if c == "mesh_channel_url":
        if req.get("url"):
            if "#" not in req["url"]:
                return {"ev": "resp", "ok": False, "error": "the link could not be read"}
        return {**ok, "url": "https://meshtastic.org/e/#CgcSAQE6AggNEggIATgKQANIAQ"}
    if c in ("pairing_open", "reboot", "log_erase", "ble_forget_all", "factory_reset", "unwatch", "watch", "set_time"):
        return ok
    if c == "new_pin":
        return {**ok, "pin": PIN}
    if c == "mesh_new_identity":
        return {**ok, "node_id": "!0badcafe"}
    return {"ev": "resp", "ok": False, "error": "unknown cmd"}


class Handler(BaseHTTPRequestHandler):
    server_version = "MockEdge/1"

    def log_message(self, fmt, *a):
        print("%s %s" % (self.command, self.path))

    def send(self, code, body, ctype="application/json", extra=None):
        data = body if isinstance(body, bytes) else json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(data)

    def authed(self):
        h = self.headers.get("Authorization", "")
        return h.startswith("Bearer ") and h[7:] in TOKENS

    def body(self):
        n = int(self.headers.get("Content-Length", 0))
        return json.loads(self.rfile.read(n) or b"{}")

    def do_GET(self):
        u = urllib.parse.urlparse(self.path)
        if u.path in ("/", "/index.html"):
            return self.send(200, (ROOT / "index.html").read_bytes(), "text/html; charset=utf-8")
        if u.path.startswith("/assets/"):
            a = ASSETS.get(u.path[8:])
            if not a:
                return self.send(404, {"error": "not found"})
            return self.send(200, (ROOT / a[0]).read_bytes(), a[1])
        if u.path == "/api/records":
            if not self.authed():
                return self.send(401, {"ok": False})
            q = urllib.parse.parse_qs(u.query)
            frm = max(int((q.get("from") or ["1"])[0]), 1)
            limit = min(int((q.get("limit") or ["200"])[0]), 512)
            seqs = range(frm, min(frm + limit, 231))
            if (q.get("fmt") or [""])[0] == "bin":
                return self.send(200, b"".join(record_bin(x) for x in seqs), "application/octet-stream")
            lines = "\n".join(json.dumps(record(x)) for x in seqs) + "\n"
            return self.send(200, lines.encode(), "application/x-ndjson")
        self.send(404, {"error": "not found"})

    def do_POST(self):
        u = urllib.parse.urlparse(self.path)
        if u.path == "/api/login":
            b = self.body()
            if b.get("pin") != PIN:
                return self.send(401, {"ok": False, "error": "wrong PIN"})
            t = secrets.token_hex(16)
            TOKENS.add(t)
            return self.send(200, {"ok": True, "token": t})
        if u.path == "/api/logout":
            return self.send(204, b"")
        if u.path == "/api/cmd":
            if not self.authed():
                return self.send(401, {"ok": False, "error": "log in first"})
            req = self.body()
            resp = handle_cmd(req, self.server.args)
            if "id" in req:
                resp["id"] = req["id"]
            return self.send(200, resp)
        self.send(404, {"error": "not found"})


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--link", choices=["connected", "ap"], default="connected")
    p.add_argument("--mode", choices=["awareness", "mesh"], default="awareness")
    args = p.parse_args()
    STATE["config"]["mode"] = args.mode
    srv = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    srv.args = args
    print("Mock Edge on http://127.0.0.1:%d (PIN %s, link %s)" % (args.port, PIN, args.link))
    srv.serve_forever()


if __name__ == "__main__":
    main()
