"""Rendering support for template folders."""

from __future__ import annotations

import copy
import re
from pathlib import Path
from typing import Any

import yaml
from jinja2 import FileSystemLoader
from loguru import logger

from .custom_sections import preserve_user_content
from .filters import create_template_environment

LOOP_METADATA_FILENAME = "__loop__.yaml"
LOOP_DIR_PATTERN = re.compile(r"\[\[\s*loop\s*:\s*(?P<path>[^:\]]+)\s*:\s*(?P<alias>[^\]]+)\s*\]\]")

__all__ = [
    "TemplateRenderer",
    "LOOP_METADATA_FILENAME",
    "LOOP_DIR_PATTERN",
]


class TemplateRenderer:
    """Delegate responsible for rendering template directories to destinations."""

    def __init__(self) -> None:
        self._string_env = create_template_environment()

    def render_folder(
        self,
        template_root: Path,
        destination_root: Path,
        context: dict[str, Any],
        *,
        preserve_customizations: bool = True,
        **extra_render_kwargs: Any,
    ) -> None:
        destination_root.mkdir(parents=True, exist_ok=True)
        self._render_directory(
            template_root,
            destination_root,
            context,
            preserve_customizations=preserve_customizations,
            extra_render_kwargs=extra_render_kwargs,
        )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _render_directory(
        self,
        src_dir: Path,
        dest_dir: Path,
        ctx: dict[str, Any],
        *,
        preserve_customizations: bool,
        extra_render_kwargs: dict[str, Any],
    ) -> None:
        for entry in sorted(src_dir.iterdir()):
            if entry.name == LOOP_METADATA_FILENAME:
                continue

            loop_match = LOOP_DIR_PATTERN.fullmatch(entry.name)
            if loop_match:
                data_path = loop_match.group("path").strip()
                alias = loop_match.group("alias").strip()
                sequence = self._resolve_context_path(ctx, data_path)
                if not isinstance(sequence, (list, tuple)):
                    raise ValueError(
                        f"Loop directory '{entry}' expects iterable at '{data_path}', got {type(sequence)}"
                    )
                for index, item in enumerate(sequence):
                    loop_ctx = copy.deepcopy(ctx)
                    loop_ctx[alias] = item
                    loop_ctx[f"{alias}_index"] = index
                    self._render_directory(
                        entry,
                        dest_dir,
                        loop_ctx,
                        preserve_customizations=preserve_customizations,
                        extra_render_kwargs=extra_render_kwargs,
                    )
                continue

            if entry.is_dir():
                if self._render_loop_directory_with_metadata(
                    entry,
                    dest_dir,
                    ctx,
                    preserve_customizations=preserve_customizations,
                    extra_render_kwargs=extra_render_kwargs,
                ):
                    continue

            if entry.is_dir():
                rendered_dir_name = self._string_env.from_string(entry.name).render(**self._render_kwargs(ctx))
                next_dest_dir = dest_dir / rendered_dir_name
                next_dest_dir.mkdir(parents=True, exist_ok=True)
                self._render_directory(
                    entry,
                    next_dest_dir,
                    ctx,
                    preserve_customizations=preserve_customizations,
                    extra_render_kwargs=extra_render_kwargs,
                )
                continue

            self._render_file(
                entry,
                dest_dir,
                ctx,
                preserve_customizations=preserve_customizations,
                extra_render_kwargs=extra_render_kwargs,
            )

    def _render_loop_directory_with_metadata(
        self,
        entry: Path,
        dest_dir: Path,
        ctx: dict[str, Any],
        *,
        preserve_customizations: bool,
        extra_render_kwargs: dict[str, Any],
    ) -> bool:
        loop_meta_path = entry / LOOP_METADATA_FILENAME
        if not loop_meta_path.exists():
            return False

        with open(loop_meta_path) as meta_file:
            loop_meta = yaml.safe_load(meta_file) or {}

        data_path = loop_meta.get("path")
        alias = loop_meta.get("alias")
        if not data_path or not alias:
            raise ValueError(f"Loop metadata in '{entry}' must define 'path' and 'alias'")

        sequence = self._resolve_context_path(ctx, data_path)
        if sequence is None:
            return True
        if not isinstance(sequence, (list, tuple)):
            raise ValueError(
                f"Loop directory '{entry}' expects iterable at '{data_path}', got {type(sequence)}"
            )

        for index, item in enumerate(sequence):
            loop_ctx = copy.deepcopy(ctx)
            loop_ctx[alias] = item
            loop_ctx[f"{alias}_index"] = index
            loop_ctx.setdefault("loop", {})[alias] = {"index": index, "value": item}
            self._render_directory(
                entry,
                dest_dir,
                loop_ctx,
                preserve_customizations=preserve_customizations,
                extra_render_kwargs=extra_render_kwargs,
            )
        return True

    def _render_file(
        self,
        entry: Path,
        dest_dir: Path,
        ctx: dict[str, Any],
        *,
        preserve_customizations: bool,
        extra_render_kwargs: dict[str, Any],
    ) -> None:
        rendered_file_name = self._string_env.from_string(entry.name).render(**self._render_kwargs(ctx))
        dest_path = dest_dir / rendered_file_name

        if entry.suffix == ".j2":
            env = create_template_environment(loader=FileSystemLoader(entry.parent))
            template = env.get_template(entry.name)
            render_kwargs = self._render_kwargs(ctx)
            render_kwargs.update(extra_render_kwargs)
            rendered_content = template.render(**render_kwargs)
            actual_dest_path = dest_path.with_suffix("")
            rendered_content = preserve_user_content(
                actual_dest_path,
                rendered_content,
                preserve=preserve_customizations,
            )
            actual_dest_path.parent.mkdir(parents=True, exist_ok=True)
            with open(actual_dest_path, "w") as handle:
                handle.write(rendered_content)
        else:
            logger.debug(f"Copying file: {entry} to {dest_path}")
            dest_path.parent.mkdir(parents=True, exist_ok=True)
            with open(entry) as source_handle:
                content = source_handle.read()
            content = preserve_user_content(dest_path, content, preserve=preserve_customizations)
            with open(dest_path, "w") as dest_handle:
                dest_handle.write(content)

    @staticmethod
    def _render_kwargs(ctx: dict[str, Any]) -> dict[str, Any]:
        kwargs = {**ctx}
        kwargs.setdefault("args", ctx)
        return kwargs

    @staticmethod
    def _resolve_context_path(context: dict[str, Any], dotted_path: str) -> Any:
        value: Any = context
        for part in dotted_path.split("."):
            if isinstance(value, dict):
                value = value.get(part)
            elif isinstance(value, (list, tuple)):
                try:
                    index = int(part)
                except ValueError as exc:
                    raise ValueError(
                        f"Expected integer index to access list element in path '{dotted_path}', got '{part}'"
                    ) from exc
                try:
                    value = value[index]
                except IndexError as exc:
                    raise ValueError(
                        f"Index {index} out of range while resolving '{dotted_path}'"
                    ) from exc
            else:
                raise ValueError(
                    f"Cannot resolve '{part}' on non-container object while resolving '{dotted_path}'"
                )
            if value is None:
                break
        return value
