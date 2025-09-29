"""Component responsible for applying level 0 templates."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from loguru import logger

from setup_assistant.templates import TemplateRenderer, get_template_folder


class Level0TemplateComponent:
    """Delegate that renders the base (level 0) package structure."""

    def __init__(self, renderer: TemplateRenderer | None = None) -> None:
        self._renderer = renderer or TemplateRenderer()

    def apply(
        self,
        *,
        destination: Path,
        context: dict[str, Any],
        preserve_customizations: bool,
    ) -> None:
        package_info = context.get("package", {})
        environment = package_info.get("environment")
        language = package_info.get("language")
        package_type = package_info.get("type", "package")

        if not environment or not language:
            logger.error(
                "Cannot render level 0 templates without environment (%s) and language (%s).",
                environment,
                language,
            )
            raise ValueError("Package configuration must define environment and language for level 0 generation")

        template_root = get_template_folder(environment, language, package_type)
        self._renderer.render_folder(
            template_root,
            destination,
            context,
            preserve_customizations=preserve_customizations,
        )
