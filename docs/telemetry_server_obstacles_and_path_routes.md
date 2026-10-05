# Telemetry Server: Obstacles & Planned Path Routes

The obstacle-authoring and pathfinding features add two new groups of routes to the
telemetry server (`https://vt-autoboat-telemetry.uk`). The server implementation lives
outside this repository, so this document specifies the contract the boat (telemetry
node) and the ground station expect.

Everything else about the route convention is unchanged from the existing
`waypoints/*` routes: routes end in `/{instance_id}` and are created per telemetry
instance.

## Encoding convention

The server currently accepts two body shapes:

| Payload | Client call | Body on the wire |
| --- | --- | --- |
| `list` (e.g. waypoints) | `requests.post(url, json=payload)` | raw JSON array |
| `dict` (e.g. autopilot params) | `requests.post(url, json=json.dumps(payload))` | **double-encoded**: a JSON string whose value is the JSON object |

The obstacle document is a GeoJSON `FeatureCollection`, which is a `dict`, so it follows
the **double-encoded** convention. The planned path is a `list`, so it is sent as a **raw
JSON array**.

## Obstacles

### `POST obstacles/set/{instance_id}`

Stores the obstacle polygon set for the instance.

- **Body**: a JSON string (double-encoded) whose value is a GeoJSON document:
  ```json
  "{\"type\":\"FeatureCollection\",\"features\":[{\"type\":\"Feature\",\"geometry\":{\"type\":\"Polygon\",\"coordinates\":[[[lon,lat],[lon,lat],...]]},\"properties\":{}}]}"
  ```
  (i.e. the outer quotes are part of the body.)
- **Server should**: JSON-decode the body once to obtain the GeoJSON object, validate that
  `type` is `FeatureCollection` or `Feature`, and store it for the instance.
- **Response**: any `2xx`.
- **Sent by**: ground station `send_obstacles()`.

> Recommendation: store the *decoded* GeoJSON object so the `get*` routes below can return
> it as a normal JSON object.

### `GET obstacles/get/{instance_id}`

Returns the stored obstacle GeoJSON for the instance.

- **Response**: the GeoJSON object, e.g.
  ```json
  {"type":"FeatureCollection","features":[...]}
  ```
- **Consumed by**: ground station `pull_obstacles()` (expects a `dict` whose `type` is
  `FeatureCollection` or `Feature`).

### `GET obstacles/get_new/{instance_id}`

Same as `obstacles/get/{instance_id}`, but with "only return when it changed since the last
call" semantics, mirroring `waypoints/get_new`.

- **Consumed by**: the telemetry node's `update_obstacles_from_telemetry()` poll. It
  compares the returned JSON against the previous response and only republishes when it
  differs.
- **Note**: returning the current value on every call is acceptable (the node de-duplicates
  client side); returning `null`/`{}`/HTTP non-2xx when nothing changed is also fine.

### `POST obstacles/test/{instance_id}` (optional)

Mirror of `waypoints/test/{instance_id}` for connectivity checks.

## Planned Path

### `POST path/set/{instance_id}`

Stores the obstacle-avoiding path the boat is currently following.

- **Body**: a raw JSON array of `[latitude, longitude]` points:
  ```json
  [[37.0,-80.0],[37.001,-80.001],...]
  ```
- **Response**: any `2xx`.
- **Sent by**: the telemetry node's `planned_path_callback()` (on every new
  `/waypoint_path`).

### `GET path/get/{instance_id}`

Returns the stored planned path.

- **Response**: a raw JSON array of `[latitude, longitude]` points (same shape as
  waypoints).
- **Consumed by**: ground station (`PlannedPathThreadRouter.RemoteFetcherThread`, route
  key `get_planned_path`), which draws it as a read-only polyline.

### `POST path/test/{instance_id}` (optional)

Mirror of `waypoints/test/{instance_id}` for connectivity checks.

## Graceful degradation

Both new route groups are polled/posted with **non-retrying**, time-limited requests on the
boat side (`_get_raw_response_without_retry` / `_send_raw_data_without_retry` in
`telemetry_node.py`). If the routes do not exist yet, the telemetry node logs the failure
and continues; the rest of telemetry (waypoints, boat status, autopilot parameters) is
unaffected. On the ground station, `send_obstacles()`/`pull_obstacles()` log an error and
leave the UI unchanged.

## Route keys (client side)

Added to `ground_station/src/utils/constants.py`:

- `get_obstacles` → `obstacles/get/`
- `get_new_obstacles` → `obstacles/get_new/`
- `set_obstacles` → `obstacles/set/`
- `test_obstacles` → `obstacles/test/`
- `get_planned_path` → `path/get/`
- `get_new_planned_path` → `path/get_new/`
- `set_planned_path` → `path/set/`
- `test_planned_path` → `path/test/`

New endpoint keys are read from `app_data/git_ignore/app_state.json`, which is deleted on
ground station exit (see `run.sh`), so a normal run picks them up automatically.
