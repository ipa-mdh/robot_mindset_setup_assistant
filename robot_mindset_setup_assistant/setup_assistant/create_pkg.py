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


def build_bt_action_metadata(action_cfg: dict) -> dict:
    if not isinstance(action_cfg, dict):
        raise ValueError("BehaviourTree action configuration must be a dictionary")

    data = copy.deepcopy(action_cfg)

    identifier_source = data.get("id") or data.get("name") or data.get("class_name")
    if not identifier_source:
        raise ValueError("BehaviourTree action configuration requires an 'id', 'name', or 'class_name'")

    identifier_snake = to_snake_case(identifier_source)
    class_name = data.get("class_name") or to_pascal_case(identifier_source)

    registration_id = data.get("registration_id") or to_pascal_case(identifier_source)
    base_class = data.get("base_class", "SyncActionNode")

    ports_cfg = data.get("ports", {}) or {}
    input_ports = []
    for port in ports_cfg.get("inputs", []) or []:
        if "name" not in port:
            raise ValueError("BehaviourTree input ports require a 'name'")
        port_name = port["name"]
        variable = to_snake_case(port_name) or f"{to_snake_case(class_name)}_{len(input_ports)}"
        normalized = {
            "name": port_name,
            "type": port.get("type", "double"),
            "description": port.get("description", "Input port"),
            "variable": variable,
        }
        if "default" in port:
            normalized["default"] = port["default"]
        input_ports.append(normalized)

    output_ports = []
    for port in ports_cfg.get("outputs", []) or []:
        if "name" not in port:
            raise ValueError("BehaviourTree output ports require a 'name'")
        port_name = port["name"]
        placeholder = port.get("placeholder", default_output_placeholder(port.get("type", "double")))
        output_ports.append({
            "name": port_name,
            "type": port.get("type", "double"),
            "description": port.get("description", "Output port"),
            "placeholder_value": placeholder,
        })

    result = {
        "id": identifier_snake,
        "class_name": class_name,
        "registration_id": registration_id,
        "base_class": base_class,
        "ports": {
            "inputs": input_ports,
            "outputs": output_ports,
        },
        "tick_behavior": data.get("tick_behavior"),
        "file_stem": to_snake_case(data.get("file_name", identifier_source)) or identifier_snake,
    }

    return result

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

def render_path(path: Path, context: dict) -> Path:
    """Render each part of the path as a Jinja2 template."""
    parts = []
    for part in path.parts:
        rendered = Template(part).render(args=context)
        parts.append(rendered)
        
    logger.debug(f"Rendering path: {path} -> {parts}")
    return Path(*parts)

def render_template_folder(template_root: Path, destination_root: Path, context: dict, *, preserve_customizations: bool = True, **extra_render_kwargs):
    """Render the template folder and copy files to the destination."""
    for file in template_root.rglob("*"):
        logger.info(f"Processing file: {file}")
        
        rel_path = file.relative_to(template_root)
        dest_path = destination_root / render_path(rel_path, context)
        logger.debug(f"Destination path: {dest_path}")

        if dest_path.is_dir():
            dest_path.mkdir(parents=True, exist_ok=True)
            
        elif file.suffix == ".j2":
            # with open(file) as f:
            #     template = Template(f.read())
            #     rendered = template.render(args=context)

            env = Environment(loader=FileSystemLoader(file.parent))
            # env = Environment()
            # Register the regex_replace filter
            env.filters['regex_replace'] = regex_replace
            # Load the template by filename (e.g., 'template.jinja2')
            template = env.get_template(file.name)
            # Render the template with your context
            render_kwargs = {"args": context}
            render_kwargs.update(extra_render_kwargs)
            rendered = template.render(**render_kwargs)

            dest_path = dest_path.with_suffix("")  # Remove .j2 extension
            rendered = preserve_user_content(dest_path, rendered, preserve=preserve_customizations)
            # create parent directories if they don't exist
            dest_path.parent.mkdir(parents=True, exist_ok=True)
            with open(dest_path, "w") as f:
                f.write(rendered)
        else:
            if file.is_file():
                logger.debug(f"Copying file: {file} to {dest_path}")
                # create parent directories if they don't exist
                dest_path.parent.mkdir(parents=True, exist_ok=True)
                with open(file) as f:
                    content = f.read()
                content = preserve_user_content(dest_path, content, preserve=preserve_customizations)
                with open(dest_path, "w") as f:
                    f.write(content)
                # shutil.copy(src=file, dst=dest_path)

def get_template_folder(environment: str):
    """
    Get the template folder based on the environment.
    Arguments:
    - environment: The environment for which the template is needed (e.g., "ros.humble", "ros.noetic").
    Returns:
    - template_root: The path to the template folder.
    """
    # get this file location
    current_file_path = Path(__file__).resolve()
    # get the parent directory of this file
    parent_dir = current_file_path.parent.parent

    def dot_to_underscore(string: str):
        """Convert dots to underscores n a string."""
        if not isinstance(string, str):
            string = str(string)
        return string.replace(".", "_")
    
    def dot_to_forward_slash(string: str):
        """Convert dots to forward slashes in a string."""
        if not isinstance(string, str):
            string = str(string)
        return string.replace(".", "/")

    # check if template directory exists
    template_root = parent_dir / Path("template") / dot_to_forward_slash(environment)
    if not template_root.exists():
        logger.error(f"Template directory {template_root} does not exist.")
        raise ValueError(f"Unknown environment: {environment}")
    
    return template_root


def get_level1_template_folder(environment: str, language: str, feature: str) -> Path:
    current_file_path = Path(__file__).resolve()
    parent_dir = current_file_path.parent.parent

    def dot_to_forward_slash(string: str) -> str:
        if not isinstance(string, str):
            string = str(string)
        return string.replace(".", "/")

    template_root = parent_dir / Path("template") / dot_to_forward_slash(environment) / language / "level_1" / feature
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
        env = Path(self.context["package"]["environment"]) / package_type / self.context["package"]["language"]
        template_root = get_template_folder(env)
        
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

    def render_level1_template(self, template_path: Path, destination_path: Path, args_context: dict, **extra):
        env = Environment(loader=FileSystemLoader(template_path.parent))
        env.filters['regex_replace'] = regex_replace
        template = env.get_template(template_path.name)
        render_kwargs = {"args": args_context}
        render_kwargs.update(extra)
        rendered = template.render(**render_kwargs)

        destination_path.parent.mkdir(parents=True, exist_ok=True)
        rendered = preserve_user_content(destination_path, rendered, preserve=self.preserve_customizations)
        with open(destination_path, "w") as f:
            f.write(rendered)

    def generate_level1_features(self):
        try:
            self.generate_behaviour_tree_actions()
        except ValueError as exc:
            logger.error(f"Failed to generate level 1 overlays: {exc}")
            raise

    def generate_behaviour_tree_actions(self):
        package_info = self.context.get("package", {})
        if package_info.get("language") != "cpp":
            return

        level1_cfg = self.context.get("level_1", {}) or {}
        behaviour_tree_cfg = level1_cfg.get("behaviour_tree", {}) or {}
        action_nodes = behaviour_tree_cfg.get("action_nodes", []) or []
        if not action_nodes:
            return

        template_root = get_level1_template_folder(
            package_info.get("environment", "ros.humble"),
            package_info.get("language", "cpp"),
            "behaviour_tree",
        )

        actions_metadata = []
        for idx, action_cfg in enumerate(action_nodes):
            try:
                actions_metadata.append(build_bt_action_metadata(action_cfg))
            except ValueError as exc:
                raise ValueError(f"Invalid behaviour tree action node definition at index {idx}: {exc}") from exc

        args_context = copy.deepcopy(self.context)
        args_context.setdefault("level_1_generated", {}).setdefault("behaviour_tree", {})["actions"] = copy.deepcopy(actions_metadata)

        factory_template_root = template_root / "factory"
        if factory_template_root.exists():
            render_template_folder(
                factory_template_root,
                self.working_dir,
                args_context,
                preserve_customizations=self.preserve_customizations,
                actions=actions_metadata,
            )

        action_template_root = template_root / "action_node"
        for action_metadata in actions_metadata:
            action_args_context = copy.deepcopy(args_context)
            action_args_context.setdefault("level_1_generated", {}).setdefault("behaviour_tree", {})["current_action"] = copy.deepcopy(action_metadata)
            action_args_context["action"] = copy.deepcopy(action_metadata)
            if action_template_root.exists():
                render_template_folder(
                    action_template_root,
                    self.working_dir,
                    action_args_context,
                    preserve_customizations=self.preserve_customizations,
                    action=action_metadata,
                )
