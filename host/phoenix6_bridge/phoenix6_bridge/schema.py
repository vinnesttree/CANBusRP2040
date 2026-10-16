# Copyright (c) 2026 CANBusRP2040 contributors
# SPDX-License-Identifier: Apache-2.0
"""
Phoenix 6 API schema shared by the RP2040 firmware and the host bridge.

The schema lists the Talon FX control requests, configuration fields and
status signals of the installed ``phoenix6`` package, together with the enum
types they use. It is produced by introspecting the official package, so the
firmware and the bridge always use CTR Electronics' names, parameter order,
types and defaults.

The firmware gets a C header generated from ``schema.json``; the bridge loads
the same file and checks it against the installed ``phoenix6`` package before
accepting commands. Both sides compare :func:`schema_hash` at connection time.
"""

from __future__ import annotations

import ast
import enum
import importlib.metadata
import inspect
import json
import re
import zlib
from pathlib import Path
from typing import Any

SCHEMA_FORMAT = 1
SCHEMA_PATH = Path(__file__).with_name("schema.json")

# Parameter / field / signal value types on the wire
T_FLOAT = "float"
T_BOOL = "bool"
T_INT = "int"
T_ENUM = "enum"
T_CONTROL = "control"


def _version() -> str:
    return importlib.metadata.version("phoenix6")


def _talonfx_source() -> str:
    from phoenix6.hardware.core import core_talon_fx

    return inspect.getsource(core_talon_fx)


def _talonfx_control_names(src: str) -> set[str]:
    return set(re.findall(r"def set_control\(self, request: (\w+)\)", src))


def _annotation_names(cls: type, method: str = "__init__") -> dict[str, str]:
    """Map parameter names to their annotation as written in the source."""
    tree = ast.parse(inspect.getsource(cls).lstrip())
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == method:
            return {
                a.arg: ast.unparse(a.annotation) if a.annotation is not None else ""
                for a in node.args.args
            }
    return {}


def _field_annotations(cls: type) -> dict[str, str]:
    """Map ``self.<name>: <annotation> = ...`` assignments in __init__."""
    tree = ast.parse(inspect.getsource(cls).lstrip())
    out: dict[str, str] = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "__init__":
            for stmt in node.body:
                if (
                    isinstance(stmt, ast.AnnAssign)
                    and isinstance(stmt.target, ast.Attribute)
                    and isinstance(stmt.target.value, ast.Name)
                    and stmt.target.value.id == "self"
                ):
                    out[stmt.target.attr] = ast.unparse(stmt.annotation)
    return out


def _enum_classes() -> dict[str, type]:
    from phoenix6.signals import spn_enums

    return {
        name: obj
        for name, obj in vars(spn_enums).items()
        if inspect.isclass(obj) and issubclass(obj, enum.Enum) and obj is not enum.Enum
    }


def _classify(annotation: str, enums: dict[str, type]) -> tuple[str, str | None, str | None]:
    """Return (type, enum name, unit) for a source annotation."""
    if annotation == "bool":
        return T_BOOL, None, None
    if annotation == "int":
        return T_INT, None, None
    if annotation in enums:
        return T_ENUM, annotation, None
    if annotation in ("float", ""):
        return T_FLOAT, None, None
    # Unit aliases such as rotation, volt or ampere are floats
    return T_FLOAT, None, annotation


def _default(value: Any) -> Any:
    if value is inspect.Parameter.empty:
        return None
    if isinstance(value, enum.Enum):
        return value.value
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return float(value)
    raise TypeError(f"unsupported default value {value!r}")


def _update_freq(cls: type) -> float | None:
    m = re.search(r"self\.update_freq_hz: hertz = ([0-9.]+)", inspect.getsource(cls))
    return float(m.group(1)) if m else None


def build_schema() -> dict[str, Any]:
    """Introspect the installed phoenix6 package."""
    from phoenix6 import configs, controls
    from phoenix6.controls import compound

    enums = _enum_classes()
    used_enums: set[str] = set()
    talonfx_src = _talonfx_source()
    accepted = _talonfx_control_names(talonfx_src)

    basic: list[dict[str, Any]] = []
    compound_list: list[dict[str, Any]] = []

    for name in sorted(accepted):
        cls = getattr(controls, name, None) or getattr(compound, name, None)
        if cls is None or not inspect.isclass(cls):
            # Protocol types such as SupportsSendRequest
            continue

        sig = inspect.signature(cls.__init__)
        annotations = _annotation_names(cls)
        params = []
        for p in list(sig.parameters.values())[1:]:
            ann = annotations.get(p.name, "")
            if name.startswith("Diff_"):
                params.append({"name": p.name, "type": T_CONTROL, "control": ann})
                continue
            ptype, ename, unit = _classify(ann, enums)
            if ename:
                used_enums.add(ename)
            entry: dict[str, Any] = {"name": p.name, "type": ptype}
            if ename:
                entry["enum"] = ename
            if unit:
                entry["unit"] = unit
            default = _default(p.default)
            if default is not None:
                entry["default"] = default
            params.append(entry)

        # Only facts needed for interoperability are recorded (no CTRE
        # documentation text): the Pro / CANivore requirement is stated on
        # the first line of the docstring.
        first = (inspect.getdoc(cls) or "").strip().split("\n")[0]
        # Torque current (FOC) control is a Phoenix Pro feature; not every
        # such class states it on the first docstring line.
        requires_pro = first.startswith("Requires Phoenix Pro") or "TorqueCurrentFOC" in name
        item = {
            "name": name,
            "update_freq_hz": _update_freq(cls),
            "requires_pro": requires_pro,
            "requires_canivore": first.startswith("Requires Phoenix Pro and CANivore"),
            "params": params,
        }
        (compound_list if name.startswith("Diff_") else basic).append(item)

    # Stable IDs: basic controls first, then compound controls
    all_controls = basic + compound_list
    for i, c in enumerate(all_controls, start=1):
        c["id"] = i

    cfg = configs.TalonFXConfiguration()
    config_fields: list[dict[str, Any]] = []
    for group_name, group in vars(cfg).items():
        if group_name.startswith("_") or isinstance(group, (bool, int, float, str)):
            continue
        anns = _field_annotations(type(group))
        for field, ann in anns.items():
            ftype, ename, unit = _classify(ann, enums)
            if ename:
                used_enums.add(ename)
            value = getattr(group, field)
            entry = {
                "group": group_name,
                "group_class": type(group).__name__,
                "field": field,
                "type": ftype,
                "default": _default(value),
            }
            if ename:
                entry["enum"] = ename
            if unit:
                entry["unit"] = unit
            config_fields.append(entry)
    for i, f in enumerate(config_fields, start=1):
        f["id"] = i

    signals: list[dict[str, Any]] = []
    for m in re.finditer(
        r"def get_(\w+)\(self, refresh: bool = True\) -> StatusSignal\[(\w+)\]:", talonfx_src
    ):
        sname, ann = m.group(1), m.group(2)
        stype, ename, unit = _classify(ann, enums)
        if ename:
            used_enums.add(ename)
        entry = {"name": sname, "type": stype}
        if ename:
            entry["enum"] = ename
        if unit:
            entry["unit"] = unit
        signals.append(entry)
    for i, s in enumerate(signals, start=1):
        s["id"] = i

    enum_out = {
        name: {member.name: member.value for member in enums[name]} for name in sorted(used_enums)
    }

    return {
        "format": SCHEMA_FORMAT,
        "phoenix6_version": _version(),
        "device": "TalonFX",
        "enums": enum_out,
        "controls": all_controls,
        "configs": config_fields,
        "signals": signals,
    }


def wire_view(schema: dict[str, Any]) -> dict[str, Any]:
    """The parts of the schema that define the wire format."""
    return {
        "format": schema["format"],
        "enums": schema["enums"],
        "controls": [
            {
                "id": c["id"],
                "name": c["name"],
                "params": [
                    {k: p[k] for k in ("name", "type", "enum", "control") if k in p}
                    for p in c["params"]
                ],
            }
            for c in schema["controls"]
        ],
        "configs": [
            {k: f[k] for k in ("id", "group", "field", "type", "enum") if k in f}
            for f in schema["configs"]
        ],
        "signals": [
            {k: s[k] for k in ("id", "name", "type", "enum") if k in s} for s in schema["signals"]
        ],
    }


def schema_hash(schema: dict[str, Any]) -> int:
    """CRC-32 (IEEE) of the canonical JSON encoding of the wire view."""
    canonical = json.dumps(wire_view(schema), sort_keys=True, separators=(",", ":"))
    return zlib.crc32(canonical.encode("utf-8")) & 0xFFFFFFFF


def load_schema(path: Path = SCHEMA_PATH) -> dict[str, Any]:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def compare_with_installed(schema: dict[str, Any]) -> list[str]:
    """Return a list of differences between ``schema`` and the installed phoenix6."""
    installed = build_schema()
    a = wire_view(schema)
    b = wire_view(installed)
    problems = []
    for key in ("format", "enums", "controls", "configs", "signals"):
        if a[key] != b[key]:
            problems.append(
                f"{key} differ between schema.json (phoenix6 {schema.get('phoenix6_version')}) "
                f"and the installed phoenix6 {installed['phoenix6_version']}"
            )
    return problems


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Generate schema.json from the installed phoenix6")
    parser.add_argument("-o", "--output", type=Path, default=SCHEMA_PATH)
    args = parser.parse_args()

    schema = build_schema()
    args.output.write_text(json.dumps(schema, indent=1) + "\n", encoding="utf-8")
    print(
        f"wrote {args.output}: phoenix6 {schema['phoenix6_version']}, "
        f"{len(schema['controls'])} controls, {len(schema['configs'])} config fields, "
        f"{len(schema['signals'])} signals, hash 0x{schema_hash(schema):08x}"
    )


if __name__ == "__main__":
    main()
