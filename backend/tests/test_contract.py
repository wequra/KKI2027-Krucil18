"""Validates every contract example against its schema. Run: python -m pytest"""
import json
from pathlib import Path
import pytest
from jsonschema import Draft202012Validator

CONTRACT = Path(__file__).resolve().parents[2] / "docs" / "contract"
PAIRS = [("telemetry", "telemetry"), ("event", "event"), ("image-meta", "image-meta")]


@pytest.mark.parametrize("schema_name,example_name", PAIRS)
def test_example_valid(schema_name, example_name):
    schema = json.loads((CONTRACT / f"{schema_name}.schema.json").read_text())
    example = json.loads((CONTRACT / "examples" / f"{example_name}.json").read_text())
    Draft202012Validator.check_schema(schema)
    Draft202012Validator(schema).validate(example)


def test_bad_telemetry_rejected():
    schema = json.loads((CONTRACT / "telemetry.schema.json").read_text())
    bad = json.loads((CONTRACT / "examples" / "telemetry.json").read_text())
    bad["lane"] = "C"
    with pytest.raises(Exception):
        Draft202012Validator(schema).validate(bad)
