"""Shared template utilities for robot mindset setup assistant."""

from .filters import (
    regex_replace,
    to_pascal_case,
    to_snake_case,
    default_output_placeholder,
    create_template_environment,
)
from .custom_sections import (
    extract_custom_section,
    insert_custom_section,
    preserve_user_content,
)
from .rendering import TemplateRenderer
from .paths import (
    get_template_folder,
    get_level1_template_root,
    get_level1_template_folder,
)

__all__ = [
    "TemplateRenderer",
    "regex_replace",
    "to_pascal_case",
    "to_snake_case",
    "default_output_placeholder",
    "create_template_environment",
    "extract_custom_section",
    "insert_custom_section",
    "preserve_user_content",
    "get_template_folder",
    "get_level1_template_root",
    "get_level1_template_folder",
]
