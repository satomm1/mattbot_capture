"""Build central ingest URLs from host and port."""

UPLOAD_PATH = "/api/v1/upload"
POSE_UPLOAD_PATH = "/api/v1/pose_upload"
DETECTION_UPLOAD_PATH = "/api/v1/detection_upload"
HEALTH_PATH = "/health"


def ingest_base_url(host: str, port: int, scheme: str = "http") -> str:
    host = host.strip()
    if not host:
        raise ValueError("ingest host must be non-empty")
    return f"{scheme}://{host}:{int(port)}"


def ingest_upload_url(host: str, port: int, scheme: str = "http") -> str:
    return ingest_base_url(host, port, scheme) + UPLOAD_PATH


def ingest_health_url(host: str, port: int, scheme: str = "http") -> str:
    return ingest_base_url(host, port, scheme) + HEALTH_PATH


def ingest_pose_upload_url(host: str, port: int, scheme: str = "http") -> str:
    return ingest_base_url(host, port, scheme) + POSE_UPLOAD_PATH


def ingest_detection_upload_url(host: str, port: int, scheme: str = "http") -> str:
    return ingest_base_url(host, port, scheme) + DETECTION_UPLOAD_PATH
