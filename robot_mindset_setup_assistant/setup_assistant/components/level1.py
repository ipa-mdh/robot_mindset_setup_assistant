"""Component responsible for applying level 1 overlays."""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

from loguru import logger

from setup_assistant.templates import (
    TemplateRenderer,
    get_level1_template_folder,
    get_level1_template_root,
)


class Level1TemplateComponent:
    """Delegate that renders optional level 1 feature overlays."""

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
        level1_cfg = context.get("level_1", {}) or {}
        if not level1_cfg:
            return

        environment = package_info.get("environment", "ros.humble")
        language = package_info.get("language")
        if not language:
            logger.warning("Skipping level 1 overlays because package language is not specified.")
            return

        package_type = package_info.get("type", "package")
        level1_root = get_level1_template_root(environment, language, package_type)
        if not level1_root.exists():
            logger.info("No level 1 template folder found at %s; skipping overlays.", level1_root)
            return

        for feature_name, feature_cfg in level1_cfg.items():
            try:
                feature_template_root = get_level1_template_folder(
                    environment,
                    language,
                    feature_name,
                    package_type,
                )
            except ValueError as exc:
                logger.warning(
                    "Level 1 feature '%s' ignored because: %s",
                    feature_name,
                    exc,
                )
                continue

            feature_context = copy.deepcopy(context)
            feature_context.setdefault("level_1_selected_feature", {})[feature_name] = copy.deepcopy(feature_cfg)
            self._renderer.render_folder(
                feature_template_root,
                destination,
                feature_context,
                preserve_customizations=preserve_customizations,
                feature=feature_cfg,
            )
