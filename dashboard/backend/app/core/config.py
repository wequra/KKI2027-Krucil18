# app/core/config.py
from pydantic_settings import BaseSettings, SettingsConfigDict

class Settings(BaseSettings):
    DATABASE_URL: str
    JETSON_API_KEY: str

    # InfluxDB
    INFLUXDB_URL: str = "http://localhost:8086"
    INFLUXDB_TOKEN: str = ""
    INFLUXDB_ORG: str = "dagozilla-itb"
    INFLUXDB_BUCKET: str = "asv_telemetry"

    # MAVLink Config
    USE_SIMULATOR: bool = False
    MAVLINK_CONNECTION_STRING: str = "udpin:0.0.0.0:14550"
    MAVLINK_BAUDRATE: int = 115200

    model_config = SettingsConfigDict(env_file=".env", env_file_encoding="utf-8")

settings = Settings()
