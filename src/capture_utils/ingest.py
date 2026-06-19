"""Build central ingest URLs from host and port."""

UPLOAD_PATH = "/api/v1/upload"
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
