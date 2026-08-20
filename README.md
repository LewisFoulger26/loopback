# LoopBack

Round-trip run planner. Set your start point, tell it how far you want to run, and it generates a road-following loop that brings you back to exactly where you started (within your chosen tolerance), then guides you round it with sat-nav-style turn-by-turn voice directions.

## Features

- **Loop generation to a target distance** with ±5/10/15% tolerance — works in cities and rural areas alike (tested in central Manchester, the Yorkshire Dales and the Peak District).
- **Avoid roads**: toggle ⛔ Avoid mode and tap any road you don't want to run down; the generator produces loops that dodge those spots (hard exclusion with an ORS key, score-and-re-roll without one). Tap a marker again to remove it.
- **Turn-by-turn voice navigation** (en-GB speech): pre-announcements ("In 150 metres, turn left…"), distance to next turn, progress bar, km done / to go.
- **Automatic re-routing**: drift off the loop and it fetches a connector route back (orange dashed line), guides you along it, then resumes the main directions at the right turn.
- **Simulate mode**: previews the whole guided run with a phantom runner, no GPS needed.

## Routing engines

| | Default | With free ORS key |
|---|---|---|
| Engine | Public [OSRM](http://project-osrm.org/) demo server | [OpenRouteService](https://openrouteservice.org) |
| Profile | General road network (no dedicated pedestrian profile) | **foot-walking** — proper paths and pavements |
| Loop method | Waypoint circle, iteratively rescaled | Native `round_trip` generator |
| Avoid roads | Candidate scoring + re-roll (best effort) | Hard `avoid_polygons` exclusion |
| Key needed | None | Free sign-up, paste into ⚙ Settings (stored in localStorage only) |

## Running it

Any static file server works. From this folder:

```bash
python -m http.server 5173
```

Then open http://localhost:5173.

> GPS (`geolocation`) only works in a **secure context** — `localhost` or HTTPS. To use it on your phone mid-run, host it somewhere with HTTPS (GitHub Pages is the zero-effort option) and open it in your phone's browser. Without GPS you can still plan routes by tapping the map, and preview the guidance with **🧪 Simulate**.

## Usage

1. **📍 My location** (or tap the map) to set your start/finish point.
2. Optionally toggle **⛔ Avoid roads** and tap streets to exclude.
3. Enter a distance in km and a tolerance, then **✦ Generate route**. **↻ Try another** re-rolls in a different direction.
4. **▶ Start run** — follow the blue line; it speaks each turn as you approach, re-routes you if you stray, and announces completion back at your start point.

## Known limits

- The public OSRM demo is a shared fair-use service and routes on the general road network — occasionally a loop may use a road you'd rather not run on (that's what Avoid mode and the ORS key are for).
- The ORS integration follows their published v2 directions API but has only been exercised without a key locally — first run with a real key may need a tweak.
