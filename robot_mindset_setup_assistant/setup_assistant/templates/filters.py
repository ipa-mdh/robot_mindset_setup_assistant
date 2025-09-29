"""Jinja2 filter helpers for template rendering."""

from __future__ import annotations

import re
from typing import Any

from jinja2 import Environment, FileSystemLoader

__all__ = [
    "regex_replace",
    "to_pascal_case",
    "to_snake_case",
    "default_output_placeholder",
    "create_template_environment",
]


def regex_replace(value: str, pattern: str, replacement: str) -> str:
    return re.sub(pattern, replacement, value)


def to_pascal_case(value: str) -> str:
    if not value:
        return ""
    tokens = re.split(r"[^0-9A-Za-z]+", value)
    return "".join(token.capitalize() for token in tokens if token)


def to_snake_case(value: str) -> str:
    if not value:
        return ""
    intermediate = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", value)
    sanitized = re.sub(r"[^0-9A-Za-z]+", "_", intermediate)
    return sanitized.strip("_").lower()


def default_output_placeholder(port_type: str) -> str:
    if not port_type:
        return "{}"
    simple = port_type.replace(" ", "")
    mapping = {
        "double": "0.0",
        "float": "0.0f",
        "int": "0",
        "int32_t": "0",
        "int64_t": "0",
        "std::string": "std::string{}",
        "bool": "false",
    }
    return mapping.get(simple, f"{port_type}{{}}")


def create_template_environment(*, loader: FileSystemLoader | None = None) -> Environment:
    env = Environment(loader=loader)
    env.filters["regex_replace"] = regex_replace
    env.filters["to_pascal_case"] = to_pascal_case
    env.filters["to_snake_case"] = to_snake_case
    env.filters["default_output_placeholder"] = default_output_placeholder
    return env
