# mattbot_capture

Robot-side image capture with a local spool and background upload queue. Callers save frames on demand via ROS services; a separate uploader POSTs completed sessions to a central ingest service.

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
  frame_{sec}_{nsec}_ir.jpg   # when IR enabled (capture:=true + enable_ir on camera)
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

Central ingest: pair IR to RGB via `extra.rgb_frame_id` or matching `ros_time` + `_ir.jpg` suffix. Sessions may contain 2× JPEG frames per capture tick.

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

**Response `201`:**

```json
{"ok": true, "session_id": "...", "files_accepted": 12, "storage_path": "robot_2/2025-06-18/..."}
```

**Errors:** `400` bad manifest, `401` auth, `409` file mismatch, `507` disk full.

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
