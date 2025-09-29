"""Helpers for preserving user-defined sections in generated files."""

from __future__ import annotations

import copy
import re
from pathlib import Path
from typing import Iterable

from loguru import logger

CUSTOM_SECTION_START_MARKER = "JINJA-BEGIN"
CUSTOM_SECTION_END_MARKER = "JINJA-END"

__all__ = [
    "CUSTOM_SECTION_START_MARKER",
    "CUSTOM_SECTION_END_MARKER",
    "extract_custom_section",
    "insert_custom_section",
    "preserve_user_content",
]


def extract_custom_section(
    file_content: str,
    start_marker: str = CUSTOM_SECTION_START_MARKER,
    end_marker: str = CUSTOM_SECTION_END_MARKER,
) -> list[dict[str, str]]:
    custom_section: list[dict[str, str]] = []
    sections = re.findall(
        rf"{re.escape(start_marker)}(.*?)^[ \t/#]*{re.escape(end_marker)}",
        file_content,
        re.DOTALL | re.MULTILINE,
    )
    for section in sections:
        match = re.match("^:([\\w\\-]+)(.*)", section, re.DOTALL | re.MULTILINE)

        if match:
            tag = match.group(1)
            content = match.group(2)
            logger.info(f"processing section tag: {tag}")
            custom_section.append({"tag": tag, "content": content})
        else:
            logger.error("Jinja marker detected without a tag. Allowed pattern: :[\\w\\-]+")
            logger.info("Example marker with tag: JINJA-BEGIN:my-tag")
            logger.error("Section without tag:")
            logger.info(section)

    return custom_section


def insert_custom_section(
    generated_content: str,
    custom_content: Iterable[dict[str, str]],
    start_marker: str = CUSTOM_SECTION_START_MARKER,
    end_marker: str = CUSTOM_SECTION_END_MARKER,
) -> str:
    content = copy.deepcopy(generated_content)
    for section in custom_content:
        tag = section["tag"]
        new_content = section["content"]
        pattern = re.compile(
            rf"{start_marker}:{tag}.*?(^[ \t/#]*){end_marker}:{tag}",
            re.DOTALL | re.MULTILINE,
        )
        match = pattern.search(content)
        if match:
            indent_end_marker = match.group(1)
            content = pattern.sub(
                f"{start_marker}:{tag}{new_content}{indent_end_marker}{end_marker}:{tag}\n",
                content,
            )
        else:
            logger.error(f"Tag ({tag}) could not be found.")

    return content


def preserve_user_content(
    file_path: Path,
    new_rendered_output: str,
    preserve: bool = True,
    *,
    start_marker: str = CUSTOM_SECTION_START_MARKER,
    end_marker: str = CUSTOM_SECTION_END_MARKER,
) -> str:
    if not preserve:
        return new_rendered_output

    if not file_path.exists():
        return new_rendered_output

    with open(file_path) as handle:
        file_content = handle.read()

    custom_section_content = extract_custom_section(
        file_content,
        start_marker=start_marker,
        end_marker=end_marker,
    )

    if custom_section_content:
        merged_output = insert_custom_section(
            new_rendered_output,
            custom_section_content,
            start_marker=start_marker,
            end_marker=end_marker,
        )
    else:
        merged_output = new_rendered_output

    return merged_output
