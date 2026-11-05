# Sotreus Context relay

The Edge's setup page fetches Context feeds from the browser, never from the
Edge. Most feeds allow that. Three do not let a web page read them:

| Feed | Why a browser cannot read it directly |
|------|---------------------------------------|
| Aircraft (adsb.lol, ADS-B) | no `Access-Control-Allow-Origin` header |
| MeteoAlarm (European official warnings) | no `Access-Control-Allow-Origin` header |
| GDACS (global disaster alerts) | answers browsers with a bot check page |

`worker.js` is a small Cloudflare Worker that fetches only those three, from a
fixed allow-list, and adds CORS headers. The page calls it from the browser
when a relay URL is set under **Settings › Context sources › Relay**. With no
relay set, those three cards say why they are unavailable and nothing is sent.

## What the relay sees

- Aircraft: a point the Worker rounds to a 0.5° grid (about 55 km) and a radius
  up to 100 nautical miles. The page filters the result to your radius.
- MeteoAlarm: the country name you picked.
- GDACS: nothing about you; it fetches the world list.
- Cloudflare sees the caller's IP address, as any web service does. The Worker
  sets no cookies and does not log query strings.

## Deploy

```sh
cd tools/context-relay
npx wrangler@3 login
npx wrangler@3 deploy
```

Wrangler prints the Worker's URL, such as
`https://sotreus-context-relay.<account>.workers.dev`. Enter it on the Edge page.
Check it with `curl https://…/health`.

Credits shown on the page: aircraft data © adsb.lol contributors (ODbL);
warnings © EUMETNET MeteoAlarm and the issuing national services; GDACS, a
cooperation of the United Nations and the European Commission.
