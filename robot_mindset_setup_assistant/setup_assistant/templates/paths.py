"""Utilities for locating template directories."""

from __future__ import annotations

from pathlib import Path

from loguru import logger

__all__ = [
    "get_template_folder",
    "get_level1_template_root",
    "get_level1_template_folder",
]


def _to_path_fragment(value: str) -> str:
    if not isinstance(value, str):
        value = str(value)
    return value.replace(".", "/")


def get_template_folder(environment: str, language: str, package_type: str) -> Path:
    """Locate the level 0 template folder for the requested environment, language, and package type."""

    current_file_path = Path(__file__).resolve()
    parent_dir = current_file_path.parent.parent.parent

    environment_fragment = _to_path_fragment(environment)
    language_fragment = _to_path_fragment(language)
    package_type_fragment = _to_path_fragment(package_type)

    base_root = parent_dir / "template" / environment_fragment

    candidate_paths = [
        base_root / language_fragment / "level_0" / package_type_fragment,
        base_root / language_fragment / package_type_fragment,
        base_root / package_type_fragment / language_fragment,
        base_root / package_type_fragment,
    ]

    for candidate in candidate_paths:
        if candidate.exists():
            return candidate

    logger.error(
        "Template directory not found for environment='{environment}', language='{language}', package_type='{package_type}'. Checked: {candidate_paths}",
        environment=environment,
        language=language,
        package_type=package_type,
        candidate_paths=candidate_paths,
    )
    raise ValueError(f"Unknown environment: {environment}/{package_type}/{language}")


def get_level1_template_root(environment: str, language: str, package_type: str | None = None) -> Path:
    current_file_path = Path(__file__).resolve()
    parent_dir = current_file_path.parent.parent.parent

    environment_fragment = _to_path_fragment(environment)
    language_fragment = _to_path_fragment(language)
    package_fragment = _to_path_fragment(package_type) if package_type else None

    base_root = parent_dir / "template" / environment_fragment

    candidate_paths = [
        base_root / language_fragment / "level_1" / package_fragment if package_fragment else None,
        base_root / language_fragment / "level_1",
        base_root / "level_1" / package_fragment if package_fragment else None,
        base_root / "level_1",
    ]

    for candidate in candidate_paths:
        if candidate and candidate.exists():
            return candidate

    return base_root / language_fragment / "level_1"


def get_level1_template_folder(
    environment: str,
    language: str,
    feature: str,
    package_type: str | None = None,
) -> Path:
    template_root = get_level1_template_root(environment, language, package_type) / feature
    if not template_root.exists():
        logger.error(f"Level 1 template directory {template_root} does not exist.")
        raise ValueError(f"Unknown level 1 feature: {feature}")

    return template_root
