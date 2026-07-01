# mattbot_capture

Robot-side image capture with a local spool and background upload queue, plus continuous pose logging to SQLite chunks. Callers save frames on demand via ROS services; uploaders POST completed sessions and pose chunks to a central ingest service.

**Prerequisite:** set `ROBOT_ID` before launch (same as `mattbot_dds`).

---

## Quick start

```bash
# Standalone
roslaunch mattbot_capture capture.launch

# Or via bringup (off by default)
roslaunch mattbot_bringup short.launch capture:=true capture_ingest_ip:=192.168.50.2 capture_ingest_port:=8080
```

---

## Capture API

| Interface | Type | Description |
|-----------|------|-------------|
| `/capture/start_session` | service | Start a session; optional auto-sampling via `sample_hz` |
| `/capture/stop_session` | service | Finalize session (`ready_for_upload`) |
| `/capture/save_frame` | service | Save one frame (creates one-shot session if none active) |
| `/capture/save_frame` | topic (`std_msgs/Empty`) | Fire-and-forget save |

### Examples

```bash
# Timed session at 1 Hz
rosservice call /capture/start_session "trigger: 'patrol'
session_id: ''
sample_hz: 1.0"
sleep 30
rosservice call /capture/stop_session "{}"

# Single frame with extra metadata
rosservice call /capture/save_frame "metadata_json: '{\"note\": \"person near door\"}'"
```

From Python:

```python
from mattbot_capture.srv import CaptureStartSession, CaptureStopSession, CaptureSaveFrame

rospy.wait_for_service("/capture/start_session")
start = rospy.ServiceProxy("/capture/start_session", CaptureStartSession)
start("patrol", "", 1.0)

rospy.ServiceProxy("/capture/save_frame", CaptureSaveFrame)(
    '{"detections": [{"class": "person", "conf": 0.9}]}'
)

rospy.ServiceProxy("/capture/stop_session", CaptureStopSession)()
```

---

## Spool layout

```
{spool_dir}/robot_{ROBOT_ID}/{session_id}/
  manifest.json
  frame_{sec}_{nsec}.jpg
  frame_{sec}_{nsec}_ir.jpg      # when IR enabled (capture:=true + enable_ir on camera)
  frame_{sec}_{nsec}_depth.png   # when capture_depth enabled (depth always on for OSOD)
```

Each `manifest.json` lists frames with ROS time, wall time, pose, detections, and optional `extra` fields.

### IR pairing (Astra camera)

When `capture:=true` at bringup, `enable_ir` is set on the camera driver (`astra.launch` or `astra_pro_plus.launch`) and `capture_node` saves a companion IR JPEG for each RGB frame (latest cached IR at save tick — not hardware-synced).

| File | `extra` |
|------|---------|
| `frame_{sec}_{nsec}.jpg` | pose, detections (unchanged) |
| `frame_{sec}_{nsec}_ir.jpg` | `modality: "ir"`, `rgb_frame_id`, `content_type: "image/jpeg"` |

Example IR frame entry:

```json
{
  "frame_id": "frame_1719240645_123456789_ir",
  "filename": "frame_1719240645_123456789_ir.jpg",
  "ros_time": {"sec": 1719240645, "nsec": 123456789},
  "detections": [],
  "extra": {
    "modality": "ir",
    "content_type": "image/jpeg",
    "rgb_frame_id": "frame_1719240645_123456789"
  }
}
```

Central ingest: pair IR to RGB via `extra.rgb_frame_id` or matching `ros_time` + `_ir.jpg` suffix.

### Depth pairing

Depth is always published for OSOD (`enable_depth` stays true on the camera). When `capture_depth` is enabled, `capture_node` saves a companion **lossless PNG** per RGB frame (latest cached depth at save tick — not hardware-synced). Values are uint16 millimeters (`16UC1`).

| File | `extra` |
|------|---------|
| `frame_{sec}_{nsec}_depth.png` | `modality: "depth"`, `rgb_frame_id`, `content_type: "image/png"`, `depth_encoding`, `depth_units` |

Example depth frame entry:

```json
{
  "frame_id": "frame_1719240645_123456789_depth",
  "filename": "frame_1719240645_123456789_depth.png",
  "ros_time": {"sec": 1719240645, "nsec": 123456789},
  "detections": [],
  "extra": {
    "modality": "depth",
    "content_type": "image/png",
    "depth_encoding": "16UC1",
    "depth_units": "millimeters",
    "rgb_frame_id": "frame_1719240645_123456789"
  }
}
```

Central ingest: pair depth to RGB via `extra.rgb_frame_id` or `_depth.png` suffix. Sessions may contain up to **3 files per capture tick** (RGB + IR + depth).

**Audio sessions** (wakeword utterances from `mattbot_record`) use the same layout with `utterance.wav` and `trigger: "wakeword"`. Transcript is in `frames[0].extra.transcript`. The uploader sends WAV files as `audio/wav`.

---

## Auto-capture

When capture is enabled, `capture_auto_trigger` runs two independent session triggers:

### Navigation

Watches `/robot_mode` from `localize_and_navigate.py` and records for each mission.

- **Start:** first transition into a capture-active mode (default: 3, 4, 5, 6, 7, 10, 11)
- **Stop:** transition to any other mode (e.g. IDLE)
- **Sample rate:** 0.5 Hz (`nav_sample_hz`)

### Person detection

Watches `/detected_objects` from `mattbot_image_detection` while the robot is **not** navigating.

- **Start:** first frame with `class_name=="person"` above `person_min_confidence`
- **Continue:** while person remains visible
- **Tail:** keep recording `person_tail_seconds` (default 5s) after person disappears
- **Stop:** tail expires (re-detect during tail cancels stop and continues)
- **Sample rate:** 1.0 Hz (`person_sample_hz`)

Nav takes priority: an active person session is stopped when a navigation mission starts.

**Frame sync caveat:** `capture_node` saves the latest camera frame and latest detections independently; bboxes in metadata may not pixel-align with every JPEG. Continuous session sampling mitigates this.

**Disable:** `auto_capture:=false` (all auto triggers), `person_capture:=false` (person only), or `capture:=false` in bringup (entire stack).

Manual `/capture/*` services remain available. If a manual session is already active, auto-capture skips start and logs once.

### capture_auto_trigger

| Param | Default | Description |
|-------|---------|-------------|
| `~auto_capture_enabled` | `true` | Enable automatic session start/stop |
| `~nav_sample_hz` | `0.5` | Frame rate during navigation sessions |
| `~nav_capture_modes` | `[3,4,5,6,7,10,11]` | `/robot_mode` values that trigger capture |
| `~robot_mode_topic` | `/robot_mode` | Navigation state topic |
| `~person_capture_enabled` | `true` | Enable person-detection sessions |
| `~person_sample_hz` | `1.0` | Frame rate during person sessions |
| `~person_tail_seconds` | `5.0` | Record after last person detection |
| `~person_min_confidence` | `0.6` | Min detection `probability` for person |
| `~detections_topic` | `/detected_objects` | Detection input topic |

---

## Parameters

### capture_node

| Param | Default | Description |
|-------|---------|-------------|
| `~spool_dir` | `/workspace/catkin_ws/data/capture_spool` | Local buffer root (persists via catkin_ws bind mount in Docker) |
| `~jpeg_quality` | `90` | JPEG quality |
| `~max_spool_bytes` | `5368709120` | Refuse saves when spool exceeds this |
| `~image_topic` | `/camera/color/image_raw` | Camera input |
| `~capture_ir` | `true` | Save IR companion JPEG when IR messages available |
| `~ir_image_topic` | `/camera/ir/image_raw` | IR input (requires `enable_ir:=capture` at bringup) |
| `~capture_depth` | `true` | Save depth companion PNG when depth messages available |
| `~depth_image_topic` | `/camera/depth/image_raw` | Depth input (always on for OSOD; no bringup gating) |
| `~pose_topic` | `/amcl_pose` | Pose fallback if TF unavailable |
| `~detections_topic` | `/detected_objects` | Latest detections cached on save |
| `~map_frame` / `~base_frame` | `map` / `base_link` | TF lookup for pose |

### capture_uploader

| Param | Default | Description |
|-------|---------|-------------|
| `~ingest_ip` | `192.168.50.2` | Central machine IP (hostname also works) |
| `~ingest_port` | `8080` | Central ingest port (`/api/v1/upload` path is fixed) |
| `~ingest_scheme` | `http` | URL scheme (rarely changed) |
| `~api_key` | `""` | Optional `X-Api-Key` header |
| `~poll_interval_s` | `30` | Scan interval |
| `~request_timeout_s` | `120` | HTTP timeout per batch |
| `~check_health` | `true` | Ping `/health` before upload |

---

## Pose logging

When `capture:=true`, `pose_logger` and `pose_uploader` run alongside the image capture stack. Pose is read locally from TF (`map` → `base_link`, `/amcl_pose` fallback) — not from DDS — so trajectories survive central server downtime.

### Sampling

| Condition | Rate |
|-----------|------|
| Moving (`robot_mode != 0`) | **2 Hz** |
| Idle (`robot_mode == 0`) | ~1/min |
| Any state | Force sample if gap exceeds 30 s |
| Significant motion while throttled | Immediate if moved > 0.1 m or turned > ~5° |

Invalid TF periods are stored with `valid=0` so localization gaps are visible.

### Pose spool layout

```
{pose_spool_dir}/robot_{ROBOT_ID}/
  active.sqlite
  active.meta.json
  chunk_2025-06-26T14-00-00Z.sqlite
  chunk_2025-06-26T14-00-00Z.meta.json
```

Chunks rotate hourly (default). Sealed chunks have `status: "ready_for_upload"`. `pose_uploader` POSTs them to `/api/v1/pose_upload` when the central ingest service is reachable.

Each SQLite row: `wall_time`, `ros_time`, local `x/y/theta/frame`, optional fleet `ref_x/ref_y/ref_theta`, `is_static`, `valid`.

### pose_logger parameters

| Param | Default | Description |
|-------|---------|-------------|
| `~pose_spool_dir` | `/workspace/catkin_ws/data/pose_spool` | Local buffer root |
| `~moving_sample_hz` | `2.0` | Sample rate when moving |
| `~static_sample_hz` | `0.0167` | Sample rate when idle (~1/min) |
| `~max_gap_s` | `30.0` | Force sample if exceeded |
| `~min_distance_m` | `0.1` | Immediate save on motion |
| `~min_angle_rad` | `0.09` | Immediate save on rotation (~5 deg) |
| `~chunk_hours` | `1.0` | Rotate SQLite chunk |
| `~max_pose_spool_bytes` | `5368709120` | Spool cap (512 MB) |
| `~map_frame` / `~base_frame` | `map` / `base_link` | TF frames |
| `~pose_topic` | `/amcl_pose` | Pose fallback |
| `~robot_mode_topic` | `/robot_mode` | Static detection |

### pose_uploader parameters

Same ingest connection params as `capture_uploader` (`~ingest_ip`, `~ingest_port`, `~api_key`, `~poll_interval_s`, etc.). Upload path is fixed at `/api/v1/pose_upload`.

Pose trajectories complement image capture: join on the central server by matching `wall_time` to frame poses in capture manifests.

---

## Detection logging

When `capture:=true`, `detection_logger` and `detection_uploader` run alongside the image capture stack. Subscribes to `/detected_objects` (`DetectedObjectArray` from OSOD) and writes a SQLite row when a message contains objects above `detection_min_confidence` — independent of JPEG/PNG saves. No rows are written when nothing is detected.

Use `detection_sample_hz` to cap how often rows are written while objects are visible (default **1.0 Hz**). Set to `0` to log every qualifying message with no rate limit.

### Detection spool layout

```
{detection_spool_dir}/robot_{ROBOT_ID}/
  active.sqlite
  active.meta.json
  chunk_2025-06-26T14-00-00Z.sqlite
  chunk_2025-06-26T14-00-00Z.meta.json
```

Each SQLite row in `detection_snapshots`: `wall_time`, `ros_time`, robot pose at sample, `object_count`, `objects_json` (same schema as capture manifest `detections[]`).

Object entry schema:

```json
{
  "class_name": "person",
  "probability": 0.92,
  "pose": {"x": 12.3, "y": 4.5, "z": 0.0},
  "width": 0.33,
  "bbox": [x1, y1, x2, y2]
}
```

Chunks rotate hourly (default). Sealed chunks upload via `POST /api/v1/detection_upload`.

### detection_logger parameters

| Param | Default | Description |
|-------|---------|-------------|
| `~detection_spool_dir` | `/workspace/catkin_ws/data/detection_spool` | Local buffer root |
| `~sample_hz` | `1.0` | Max log rate when objects present (`detection_sample_hz` launch arg; `0` = unlimited) |
| `~chunk_hours` | `1.0` | Rotate SQLite chunk |
| `~max_detection_spool_bytes` | `5368709120` | Spool cap |
| `~detections_topic` | `/detected_objects` | Input from image detection |
| `~min_confidence` | `0.0` | Min `probability` to include in spool (`detection_min_confidence` launch arg) |
| `~map_frame` / `~base_frame` / `~pose_topic` | same as pose_logger | Robot pose context per snapshot |

### detection_uploader parameters

Same ingest connection params as `capture_uploader`. Upload path fixed at `/api/v1/detection_upload`.

Join detection snapshots to image captures on the central server by overlapping `wall_time` / `ros_time`.

---

## Central machine specification

Implement separately on a central PC/NAS. Robots never connect to the database directly.

### Components

1. **File storage** — e.g. `/data/captures/` on NAS or local disk
2. **PostgreSQL** — metadata catalog (images stay on disk)
3. **Ingest service** — FastAPI (or similar) HTTP API

### Storage layout

```
/data/captures/
  robot_2/
    2025-06-18/
      {session_id}/
        manifest.json
        frame_*.jpg
```

### PostgreSQL schema

```sql
CREATE TABLE robots (
  id          INTEGER PRIMARY KEY,
  name        TEXT,
  created_at  TIMESTAMPTZ DEFAULT now()
);

CREATE TABLE sessions (
  id           UUID PRIMARY KEY,
  robot_id     INTEGER NOT NULL REFERENCES robots(id),
  trigger      TEXT NOT NULL,
  started_at   TIMESTAMPTZ NOT NULL,
  ended_at     TIMESTAMPTZ,
  status       TEXT NOT NULL DEFAULT 'complete',
  frame_count  INTEGER NOT NULL DEFAULT 0,
  storage_path TEXT NOT NULL,
  uploaded_at  TIMESTAMPTZ DEFAULT now()
);

CREATE TABLE captures (
  id            BIGSERIAL PRIMARY KEY,
  session_id    UUID NOT NULL REFERENCES sessions(id),
  robot_id      INTEGER NOT NULL REFERENCES robots(id),
  frame_id      TEXT NOT NULL,
  filename      TEXT NOT NULL,
  storage_path  TEXT NOT NULL,
  ros_time_sec  BIGINT NOT NULL,
  ros_time_nsec INTEGER NOT NULL,
  wall_time     TIMESTAMPTZ,
  pose_x        DOUBLE PRECISION,
  pose_y        DOUBLE PRECISION,
  pose_theta    DOUBLE PRECISION,
  detections    JSONB DEFAULT '[]',
  extra         JSONB DEFAULT '{}',
  sha256        TEXT,
  UNIQUE (session_id, frame_id)
);

CREATE INDEX idx_captures_robot_time ON captures (robot_id, wall_time);
CREATE INDEX idx_captures_detections ON captures USING GIN (detections);
```

Register robots on first upload: `INSERT INTO robots (id) VALUES ($1) ON CONFLICT DO NOTHING`.

### Ingest API

#### `GET /health`

Returns `200` with body `{"status": "ok"}`. The robot uploader uses this before uploading.

#### `POST /api/v1/upload`

**Headers:**

- `X-Robot-Id` — must match `manifest.robot_id`
- `X-Api-Key` — optional; reject with `401` if required and missing

**Body:** `multipart/form-data`

- `manifest` — JSON file (`manifest.json`)
- `files` — one or more file parts (JPEG or WAV); filenames must match `frames[].filename`

**Processing:**

1. Parse manifest; require `schema_version`, `status == "ready_for_upload"`.
2. Verify header robot id matches manifest.
3. Storage path: `robot_{id}/{YYYY-MM-DD}/{session_id}/` under `STORAGE_ROOT`.
4. Write image/audio files and manifest; reject path traversal in filenames.
5. In one DB transaction: upsert `sessions`; insert `captures` from `frames[]`.
6. Use `ON CONFLICT (session_id, frame_id) DO NOTHING` for idempotent retries.

**IR frames:** identify via `filename` suffix `_ir.jpg` or `frames[].extra.modality == "ir"`. Link to RGB via `extra.rgb_frame_id`. Store both under the same session directory.

**Depth frames:** identify via `_depth.png` suffix or `frames[].extra.modality == "depth"`. Link to RGB via `extra.rgb_frame_id`. PNG holds uint16 depth in millimeters (`16UC1`); upload as `image/png`.

**Response `201`:**

```json
{"ok": true, "session_id": "...", "files_accepted": 12, "storage_path": "robot_2/2025-06-18/..."}
```

**Errors:** `400` bad manifest, `401` auth, `409` file mismatch, `507` disk full.

#### `POST /api/v1/pose_upload`

**Headers:** same as image upload (`X-Robot-Id`, optional `X-Api-Key`).

**Body:** `multipart/form-data`

- `meta` — JSON file (`chunk_*.meta.json`)
- `chunk` — SQLite file (`chunk_*.sqlite`)

**Processing:**

1. Parse meta; require `schema_version`, `status == "ready_for_upload"`.
2. Verify header robot id matches meta.
3. Read rows from SQLite `poses` table; bulk-insert into `robot_poses`.
4. Use `ON CONFLICT (robot_id, wall_time) DO NOTHING` for idempotent retries.

**Response `201`:**

```json
{"ok": true, "chunk_id": "chunk_2025-06-26T14-00-00Z", "rows_accepted": 7200}
```

#### `POST /api/v1/detection_upload`

**Headers:** same as image upload (`X-Robot-Id`, optional `X-Api-Key`).

**Body:** `multipart/form-data`

- `meta` — JSON file (`chunk_*.meta.json`)
- `chunk` — SQLite file (`chunk_*.sqlite`)

**Processing:**

1. Parse meta; require `schema_version`, `status == "ready_for_upload"`.
2. Verify header robot id matches meta.
3. Read rows from SQLite `detection_snapshots` table; bulk-insert into `detection_snapshots` (PostgreSQL).
4. Parse `objects_json` into JSONB column `objects`.
5. Use idempotent upsert on `(robot_id, wall_time, chunk_id)` or similar.

**Response `201`:**

```json
{"ok": true, "chunk_id": "chunk_2025-06-26T14-00-00Z", "rows_accepted": 3600}
```

### PostgreSQL schema (pose chunks)

```sql
CREATE TABLE robot_poses (
  id            BIGSERIAL PRIMARY KEY,
  robot_id      INTEGER NOT NULL,
  wall_time     TIMESTAMPTZ NOT NULL,
  ros_time_sec  BIGINT,
  ros_time_nsec INTEGER,
  x             DOUBLE PRECISION,
  y             DOUBLE PRECISION,
  theta         DOUBLE PRECISION,
  frame         TEXT,
  ref_x         DOUBLE PRECISION,
  ref_y         DOUBLE PRECISION,
  ref_theta     DOUBLE PRECISION,
  is_static     BOOLEAN NOT NULL,
  valid         BOOLEAN NOT NULL,
  chunk_id      TEXT,
  uploaded_at   TIMESTAMPTZ DEFAULT now(),
  UNIQUE (robot_id, wall_time)
);

CREATE INDEX idx_robot_poses_robot_time ON robot_poses (robot_id, wall_time);
```

### PostgreSQL schema (detection chunks)

```sql
CREATE TABLE detection_snapshots (
  id              BIGSERIAL PRIMARY KEY,
  robot_id        INTEGER NOT NULL,
  chunk_id        TEXT NOT NULL,
  wall_time       TIMESTAMPTZ NOT NULL,
  ros_time_sec    BIGINT,
  ros_time_nsec   INTEGER,
  robot_x         DOUBLE PRECISION,
  robot_y         DOUBLE PRECISION,
  robot_theta     DOUBLE PRECISION,
  robot_frame     TEXT,
  robot_valid     BOOLEAN NOT NULL,
  object_count    INTEGER NOT NULL,
  objects         JSONB NOT NULL,
  uploaded_at     TIMESTAMPTZ DEFAULT now(),
  UNIQUE (robot_id, wall_time, chunk_id)
);

CREATE INDEX idx_detection_snapshots_robot_time ON detection_snapshots (robot_id, wall_time);
CREATE INDEX idx_detection_objects ON detection_snapshots USING GIN (objects);
```

### Deployment sketch

```yaml
services:
  postgres:
    image: postgres:16
    environment:
      POSTGRES_DB: robot_capture
      POSTGRES_USER: capture
      POSTGRES_PASSWORD: changeme
    volumes: [pgdata:/var/lib/postgresql/data]

  ingest:
    build: ./ingest
    ports: ["8080:8080"]
    environment:
      DATABASE_URL: postgresql://capture:changeme@postgres/robot_capture
      STORAGE_ROOT: /data/captures
      API_KEYS: ""
    volumes:
      - /data/captures:/data/captures

volumes:
  pgdata:
```

Set each robot's `~ingest_ip` and `~ingest_port` to match the central ingest service (default port 8080).

---

## License

MIT
