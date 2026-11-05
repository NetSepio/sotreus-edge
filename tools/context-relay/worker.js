// Sotreus Context relay: a Cloudflare Worker.
//
// The Sotreus Edge setup page is served over plain HTTP from the Edge, and its
// Context tab fetches public feeds straight from the browser. Three useful
// feeds send no CORS headers (or challenge browsers), so a web page cannot read
// them: adsb.lol (aircraft), MeteoAlarm (European warnings) and GDACS (global
// disaster alerts). This Worker fetches only those, from a fixed allow-list,
// and returns them with CORS headers.
//
// Privacy:
//   - Aircraft requests carry a point rounded here to a 0.5° grid (about 55 km),
//     whatever the page sends. MeteoAlarm requests carry a country name. GDACS
//     requests carry nothing about the user.
//   - No cookies, no request bodies, no logging of query strings by this code.
//     Responses are cached at the edge by the rounded upstream URL only.
//   - Like any web service, Cloudflare sees the caller's IP address.
//
// Deploy: `npx wrangler deploy` in this folder (see README.md). The Edge page
// then takes the Worker's https URL under Settings › Context sources › Relay.

const UA = "SotreusContextRelay/1.0 (+https://sotreus.com)";
const MAX_BYTES = 4 * 1024 * 1024;

const METEOALARM = new Set([
  "austria", "belgium", "bosnia-herzegovina", "bulgaria", "croatia", "cyprus", "czechia", "denmark", "estonia",
  "finland", "france", "germany", "greece", "hungary", "iceland", "ireland", "israel", "italy", "latvia",
  "lithuania", "luxembourg", "malta", "moldova", "montenegro", "netherlands", "norway", "poland", "portugal",
  "republic-of-north-macedonia", "romania", "serbia", "slovakia", "slovenia", "spain", "sweden", "switzerland",
  "ukraine", "united-kingdom",
]);

const GDACS_BASE = "https://www.gdacs.org/gdacsapi/api/events/geteventlist/SEARCH?";
const GDACS = {
  major: GDACS_BASE + "eventlist=EQ;TC;FL;VO;WF;DR&alertlevel=orange;red",
  minor: GDACS_BASE + "eventlist=TC;FL;VO;WF&alertlevel=green",
};

class BadRequest extends Error {}

function num(q, k, lo, hi) {
  const v = Number(q.get(k));
  if (!Number.isFinite(v) || v < lo || v > hi) throw new BadRequest(k + " must be a number from " + lo + " to " + hi);
  return v;
}

/** Centre of the 0.5° cell holding v. */
function cell(v) {
  return (Math.floor(v / 0.5) * 0.5 + 0.25).toFixed(2);
}

const ROUTES = {
  "/aircraft": (q) => {
    const lat = cell(num(q, "lat", -90, 90));
    const lon = cell(num(q, "lon", -180, 180));
    const nm = Math.round(num(q, "radius_nm", 1, 100));
    return { url: `https://api.adsb.lol/v2/point/${lat}/${lon}/${nm}`, ttl: 15, type: "application/json" };
  },
  "/meteoalarm": (q) => {
    const c = q.get("country") || "";
    if (!METEOALARM.has(c)) throw new BadRequest("unknown country");
    return { url: "https://feeds.meteoalarm.org/api/v1/warnings/feeds-" + c, ttl: 600, type: "application/json" };
  },
  "/gdacs": (q) => {
    const url = GDACS[q.get("list") || ""];
    if (!url) throw new BadRequest("list must be major or minor");
    return { url, ttl: 900, type: "application/json" };
  },
};

const CORS = {
  "Access-Control-Allow-Origin": "*",
  "Access-Control-Allow-Methods": "GET, OPTIONS",
  "Access-Control-Max-Age": "86400",
};

function json(status, obj) {
  return new Response(JSON.stringify(obj), {
    status,
    headers: { ...CORS, "Content-Type": "application/json", "Cache-Control": "no-store" },
  });
}

export default {
  async fetch(req, env, ctx) {
    if (req.method === "OPTIONS") return new Response(null, { status: 204, headers: CORS });
    if (req.method !== "GET") return json(405, { ok: false, error: "GET only" });
    const u = new URL(req.url);
    if (u.pathname === "/" || u.pathname === "/health") return json(200, { ok: true, routes: Object.keys(ROUTES) });
    const route = ROUTES[u.pathname];
    if (!route) return json(404, { ok: false, error: "unknown route" });

    let up;
    try {
      up = route(u.searchParams);
    } catch (e) {
      return json(400, { ok: false, error: e instanceof BadRequest ? e.message : "bad request" });
    }

    const cache = caches.default;
    const key = new Request(up.url);
    let res = await cache.match(key);
    if (!res) {
      let r;
      try {
        r = await fetch(up.url, { headers: { "User-Agent": UA, Accept: "application/json" } });
      } catch (e) {
        return json(502, { ok: false, error: "could not reach the source" });
      }
      if (!r.ok) return json(502, { ok: false, error: "the source answered HTTP " + r.status });
      if (/html/i.test(r.headers.get("content-type") || "")) return json(502, { ok: false, error: "the source did not return data" });
      const body = await r.arrayBuffer();
      if (body.byteLength > MAX_BYTES) return json(502, { ok: false, error: "the source sent too much data" });
      res = new Response(body, { headers: { "Content-Type": up.type, "Cache-Control": "public, max-age=" + up.ttl } });
      ctx.waitUntil(cache.put(key, res.clone()));
    }
    const out = new Response(res.body, res);
    for (const [k, v] of Object.entries(CORS)) out.headers.set(k, v);
    out.headers.set("X-Content-Type-Options", "nosniff");
    out.headers.set("Referrer-Policy", "no-referrer");
    return out;
  },
};
