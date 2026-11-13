import json
from pathlib import Path
from typing import Any, Dict
import jsonschema
from fastapi import HTTPException, status

CONTRACT_DIR = Path(__file__).resolve().parents[2] / "docs" / "contract"

def _load_schema(name: str) -> Dict[str, Any]:
    path = CONTRACT_DIR / f"{name}.schema.json"
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)

SCHEMAS = {
    "telemetry": _load_schema("telemetry"),
    "event": _load_schema("event"),
    "image-meta": _load_schema("image-meta"),
}

def validate_contract(schema_name: str, payload: Dict[str, Any]) -> Dict[str, Any]:
    schema = SCHEMAS[schema_name]
    try:
        jsonschema.validate(instance=payload, schema=schema)
    except jsonschema.ValidationError as exc:
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail=f"Contract validation failed for '{schema_name}': {exc.message}",
        )
    return payload
