import re
from enum import Enum
from jinja2 import Environment, FileSystemLoader, Template
from pathlib import Path
import shutil
import yaml
from loguru import logger
import copy

from setup_assistant.load_config import get_config
from setup_assistant.load_config import get_lookup_tables
from setup_assistant.package_versioning import GitFlowRepo
from setup_assistant.dev_setup import apply as apply_dev_setup
from setup_assistant.doxygen_awesome import apply as apply_doxygen_awesome

CUSTOM_SECTION_START_MARKER="JINJA-BEGIN"
CUSTOM_SECTION_END_MARKER="JINJA-END"
LOOP_METADATA_FILENAME = "__loop__.yaml"

def regex_replace(value, pattern, replacement):
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


def extract_custom_section(file_content,
                           start_marker=CUSTOM_SECTION_START_MARKER,
                           end_marker=CUSTOM_SECTION_END_MARKER):
    """
    Extracts the section between markers (exclusive),
    and returns it with indentation added for all lines
    to match the indent of the marker line.
    """
    # Match entire block including markers (allow multiline)
    # pattern = re.compile(
    #     rf"(^[ \t]*){re.escape(start_marker)}(.*)\n(.*?)\n^[ \t]*{re.escape(end_marker)}",
    #     re.DOTALL | re.MULTILINE
    # )
    # match = pattern.search(file_content)
    # if match:
    #     indent = match.group(1)
    #     tag = match.group(2).strip()
    #     content = match.group(3)
    #     return {'indent': indent, 'tag': tag, 'content': content}

    custom_section = []
    sections = re.findall(
        rf'{re.escape(start_marker)}(.*?)^[ \t/#]*{re.escape(end_marker)}',
        file_content,
        re.DOTALL | re.MULTILINE,
    )
    for section in sections:
        match = re.match("^:([\w\-]+)(.*)", section, re.DOTALL | re.MULTILINE)

        if match:
            tag = match.group(1)
            content = match.group(2)
            logger.info(f"processing section tag: {tag}")
            # logger.debug(content)
            custom_section.append({'tag': tag, 'content': content})
        else:
            logger.error("Jinja marker detected without a tag. Allowed pattern: :[\w\-]+")
            logger.info("Example marker with tag: JINJA-BEGIN:my-tag")
            logger.error("Section without tag:")
            logger.info(section)

    return custom_section

def insert_custom_section(generated_content,
                          custom_content,
                          start_marker=CUSTOM_SECTION_START_MARKER,
                          end_marker=CUSTOM_SECTION_END_MARKER):
    """
    Replace the section between start_marker and end_marker,
    preserving any leading whitespace before the markers.
    """
    content = copy.deepcopy(generated_content)
    for c in custom_content:
        tag = c['tag']
        new_content = c['content']
        pattern = re.compile(
            rf'{start_marker}:{tag}.*?(^[ \t/#]*){end_marker}:{tag}',
            re.DOTALL | re.MULTILINE
        )
        match = pattern.search(content)
        if match:
            indent_end_marker = match.group(1)

            content = pattern.sub(f'{start_marker}:{tag}{new_content}{indent_end_marker}{end_marker}:{tag}\n', content)
        else:
            logger.error(f"Tag ({tag}) could not be found.")

    return content

def preserve_user_content(file_path: Path, new_rendered_output, preserve: bool = True):
    """Preserve user content between custom section markers."""
    if not preserve:
        return new_rendered_output

    if not file_path.exists():
        return new_rendered_output

    with open(file_path) as f:
        file_content = f.read()

    # Extract custom section from previous output
    custom_section_content = extract_custom_section(file_content)

    # if custom_section_content:
    #     logger.debug(f"Custom section content:\n{custom_section_content}")

    # Inject custom section back into the new generated content
    if custom_section_content:
        merged_output = insert_custom_section(new_rendered_output, custom_section_content)
    else:
        merged_output = new_rendered_output  # No manual changes found

    return merged_output

def create_template_environment(*, loader: FileSystemLoader | None = None) -> Environment:
    env = Environment(loader=loader)
    env.filters['regex_replace'] = regex_replace
    env.filters['to_pascal_case'] = to_pascal_case
    env.filters['to_snake_case'] = to_snake_case
    env.filters['default_output_placeholder'] = default_output_placeholder
    return env

def render_path(path: Path, context: dict) -> Path:
    """Render each part of the path as a Jinja2 template."""
    env = create_template_environment()
    parts = []
    render_kwargs = {**context}
    render_kwargs.setdefault("args", context)
    for part in path.parts:
        rendered = env.from_string(part).render(**render_kwargs)
        parts.append(rendered)
        
    logger.debug(f"Rendering path: {path} -> {parts}")
    return Path(*parts)

LOOP_DIR_PATTERN = re.compile(r"\[\[\s*loop\s*:\s*(?P<path>[^:\]]+)\s*:\s*(?P<alias>[^\]]+)\s*\]\]")


def resolve_context_path(context: dict, dotted_path: str):
    value = context
    for part in dotted_path.split('.'):
        if isinstance(value, dict):
            value = value.get(part)
        elif isinstance(value, (list, tuple)):
            try:
                index = int(part)
            except ValueError as exc:
                raise ValueError(f"Expected integer index to access list element in path '{dotted_path}', got '{part}'") from exc
            try:
                value = value[index]
            except IndexError as exc:
                raise ValueError(f"Index {index} out of range while resolving '{dotted_path}'") from exc
        else:
            raise ValueError(f"Cannot resolve '{part}' on non-container object while resolving '{dotted_path}'")
        if value is None:
            break
    return value


def render_template_folder(template_root: Path, destination_root: Path, context: dict, *, preserve_customizations: bool = True, **extra_render_kwargs):
    """Render the template folder and copy files to the destination."""

    string_env = create_template_environment()

    def _render_directory(src_dir: Path, dest_dir: Path, ctx: dict):
        for entry in sorted(src_dir.iterdir()):
            if entry.name == LOOP_METADATA_FILENAME:
                continue

            loop_match = LOOP_DIR_PATTERN.fullmatch(entry.name)
            if loop_match:
                data_path = loop_match.group('path').strip()
                alias = loop_match.group('alias').strip()
                sequence = resolve_context_path(ctx, data_path)
                if not isinstance(sequence, (list, tuple)):
                    raise ValueError(f"Loop directory '{entry}' expects iterable at '{data_path}', got {type(sequence)}")
                for index, item in enumerate(sequence):
                    loop_ctx = copy.deepcopy(ctx)
                    loop_ctx[alias] = item
                    loop_ctx[f"{alias}_index"] = index
                    _render_directory(entry, dest_dir, loop_ctx)
                continue

            if entry.is_dir():
                loop_meta_path = entry / LOOP_METADATA_FILENAME
                if loop_meta_path.exists():
                    with open(loop_meta_path) as meta_file:
                        loop_meta = yaml.safe_load(meta_file) or {}

                    data_path = loop_meta.get("path")
                    alias = loop_meta.get("alias")
                    if not data_path or not alias:
                        raise ValueError(f"Loop metadata in '{entry}' must define 'path' and 'alias'")

                    sequence = resolve_context_path(ctx, data_path)
                    if sequence is None:
                        continue
                    if not isinstance(sequence, (list, tuple)):
                        raise ValueError(
                            f"Loop directory '{entry}' expects iterable at '{data_path}', got {type(sequence)}"
                        )

                    for index, item in enumerate(sequence):
                        loop_ctx = copy.deepcopy(ctx)
                        loop_ctx[alias] = item
                        loop_ctx[f"{alias}_index"] = index
                        loop_ctx.setdefault("loop", {})[alias] = {"index": index, "value": item}
                        _render_directory(entry, dest_dir, loop_ctx)
                    continue

            if entry.is_dir():
                dir_render_kwargs = {**ctx}
                dir_render_kwargs.setdefault("args", ctx)
                rendered_dir_name = string_env.from_string(entry.name).render(**dir_render_kwargs)
                next_dest_dir = dest_dir / rendered_dir_name
                next_dest_dir.mkdir(parents=True, exist_ok=True)
                _render_directory(entry, next_dest_dir, ctx)
                continue

            file_render_kwargs = {**ctx}
            file_render_kwargs.setdefault("args", ctx)
            rendered_file_name = string_env.from_string(entry.name).render(**file_render_kwargs)
            dest_path = dest_dir / rendered_file_name

            if entry.suffix == ".j2":
                env = create_template_environment(loader=FileSystemLoader(entry.parent))

                template = env.get_template(entry.name)
                render_kwargs = {**ctx}
                render_kwargs.setdefault("args", ctx)
                render_kwargs.update(extra_render_kwargs)
                rendered_content = template.render(**render_kwargs)

                actual_dest_path = dest_path.with_suffix("")
                rendered_content = preserve_user_content(actual_dest_path, rendered_content, preserve=preserve_customizations)
                actual_dest_path.parent.mkdir(parents=True, exist_ok=True)
                with open(actual_dest_path, "w") as f:
                    f.write(rendered_content)
            else:
                logger.debug(f"Copying file: {entry} to {dest_path}")
                dest_path.parent.mkdir(parents=True, exist_ok=True)
                with open(entry) as f:
                    content = f.read()
                content = preserve_user_content(dest_path, content, preserve=preserve_customizations)
                with open(dest_path, "w") as f:
                    f.write(content)

    destination_root.mkdir(parents=True, exist_ok=True)
    _render_directory(template_root, destination_root, context)

def get_template_folder(environment: str, language: str, package_type: str) -> Path:
    """Locate the level 0 template folder for the requested environment, language, and package type."""

    current_file_path = Path(__file__).resolve()
    parent_dir = current_file_path.parent.parent

    def to_path_fragment(value: str) -> str:
        if not isinstance(value, str):
            value = str(value)
        return value.replace(".", "/")

    environment_fragment = to_path_fragment(environment)
    language_fragment = to_path_fragment(language)
    package_type_fragment = to_path_fragment(package_type)

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
        "Template directory not found for environment='%s', language='%s', package_type='%s'. Checked: %s",
        environment,
        language,
        package_type,
        candidate_paths,
    )
    raise ValueError(f"Unknown environment: {environment}/{package_type}/{language}")


def get_level1_template_root(environment: str, language: str, package_type: str | None = None) -> Path:
    current_file_path = Path(__file__).resolve()
    parent_dir = current_file_path.parent.parent

    def dot_to_forward_slash(string: str) -> str:
        if not isinstance(string, str):
            string = str(string)
        return string.replace(".", "/")

    environment_fragment = dot_to_forward_slash(environment)
    language_fragment = dot_to_forward_slash(language)
    package_fragment = dot_to_forward_slash(package_type) if package_type else None

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


def get_level1_template_folder(environment: str, language: str, feature: str, package_type: str | None = None) -> Path:
    template_root = get_level1_template_root(environment, language, package_type) / feature
    if not template_root.exists():
        logger.error(f"Level 1 template directory {template_root} does not exist.")
        raise ValueError(f"Unknown level 1 feature: {feature}")

    return template_root


class RosNoeticPackage(GitFlowRepo):
    def __init__(self, destination: Path, config_path: Path, preserve_customizations: bool = True):
        self.config_path = config_path
        self.context = get_config(self.config_path)
        self.preserve_customizations = preserve_customizations

        lookup_tables = get_lookup_tables(self.context["package"]["environment"])
        self.context.update({"lookup": lookup_tables})

        self.package_name = self.context["package"]["name"]
        
        self.working_dir = destination / self.package_name
        self.working_dir.mkdir(parents=True, exist_ok=True)
        
        super().__init__(self.working_dir)
        
        self.init()
        self.add_dev_setup()
        self.add_doxygen_awesome()

    @GitFlowRepo.decorator
    def init(self):
        """
        Initialize the package by creating the necessary directories and files.
        """

        package_type = self.context["package"].get("type", "package")

        # Get the template folder based on the environment
        environment = self.context["package"]["environment"]
        language = self.context["package"].get("language")
        template_root = get_template_folder(environment, language, package_type)
        
        # Render the template folder
        render_template_folder(
            template_root,
            self.working_dir,
            self.context,
            preserve_customizations=self.preserve_customizations,
        )

        self.generate_level1_features()
        
        # Copy config to working dir
        dest = self.working_dir / ".robot_mindeset_setup_assistant.yaml"
        shutil.copy(self.config_path, dest)
    
    @GitFlowRepo.decorator
    def add_dev_setup(self):
        """
        Apply the dev setup to the package.
        """
        apply_dev_setup(self, self.working_dir, environment=self.context["package"]["environment"], package_name=self.package_name)
        
    @GitFlowRepo.decorator
    def add_doxygen_awesome(self):
        """
        Add Doxygen Awesome to the package.
        """
        apply_doxygen_awesome(self)

    def generate_level1_features(self):
        package_info = self.context.get("package", {})
        level1_cfg = self.context.get("level_1", {}) or {}
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
            logger.info(f"No level 1 template folder found at {level1_root}; skipping overlays.")
            return

        for feature_name, feature_cfg in level1_cfg.items():
            feature_template_root = get_level1_template_folder(environment, language, feature_name, package_type)
            if not feature_template_root.exists():
                logger.warning(f"Level 1 feature '{feature_name}' ignored because template folder {feature_template_root} is missing.")
                continue

            try:
                feature_context = copy.deepcopy(self.context)
                feature_context.setdefault("level_1_selected_feature", {})[feature_name] = copy.deepcopy(feature_cfg)
                render_template_folder(
                    feature_template_root,
                    self.working_dir,
                    feature_context,
                    preserve_customizations=self.preserve_customizations,
                    feature=feature_cfg,
                )
            except ValueError as exc:
                logger.error(f"Failed to generate level 1 feature '{feature_name}': {exc}")
                raise
