"""Component responsible for applying level 1 overlays."""

from __future__ import annotations

import copy
import shutil
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

        package_name = package_info.get("name")
        if package_name:
            self._migrate_legacy_factory_layout(destination, package_name)

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
            self._render_feature_contents(
                feature_template_root,
                destination,
                feature_context,
                feature_cfg,
                preserve_customizations,
            )

    def _render_feature_contents(
        self,
        feature_template_root: Path,
        destination: Path,
        context: dict[str, Any],
        feature_cfg: dict[str, Any],
        preserve_customizations: bool,
    ) -> None:
        entries = [entry for entry in sorted(feature_template_root.iterdir()) if entry.name != "__loop__.yaml"]
        directories = [entry for entry in entries if entry.is_dir()]
        files = [entry for entry in entries if entry.is_file()]

        if directories:
            for directory in directories:
                self._renderer.render_folder(
                    directory,
                    destination,
                    context,
                    preserve_customizations=preserve_customizations,
                    feature=feature_cfg,
                )

        if files:
            for template_file in files:
                logger.warning(
                    "Unexpected top-level file '%s' inside level 1 template; skipping.",
                    template_file,
                )

        if not directories and not files:
            # Nothing rendered (e.g., empty template root); fall back to default behaviour
            self._renderer.render_folder(
                feature_template_root,
                destination,
                context,
                preserve_customizations=preserve_customizations,
                feature=feature_cfg,
            )

    def _migrate_legacy_factory_layout(self, destination: Path, package_name: str) -> None:
        legacy_root = destination / "factory" / package_name / package_name
        if not legacy_root.exists():
            return

        target_root = destination / package_name
        logger.info(
            "Migrating legacy level 1 factory layout from {legacy_root} to {target_root}",
            legacy_root=legacy_root,
            target_root=target_root,
        )

        for item in legacy_root.iterdir():
            target_path = target_root / item.name
            if item.is_dir():
                shutil.copytree(item, target_path, dirs_exist_ok=True)
            else:
                target_path.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(item, target_path)

        shutil.rmtree(destination / "factory")

