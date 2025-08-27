import re
from enum import Enum
from jinja2 import Template
from pathlib import Path
import shutil
import yaml
from loguru import logger

from setup_assistant.load_config import get_config
from setup_assistant.load_config import get_lookup_tables
from setup_assistant.package_versioning import GitFlowRepo
from setup_assistant.dev_setup import apply as apply_dev_setup
from setup_assistant.doxygen_awesome import apply as apply_doxygen_awesome

CUSTOM_SECTION_START_MARKER="# JINJA-BEGIN:customer-section"
CUSTOM_SECTION_END_MARKER="# JINJA-END:customer-section"

def extract_custom_section(file_content,
                           start_marker="# JINJA-BEGIN:customer-section",
                           end_marker="# JINJA-END:customer-section"):
    """
    Extracts the section between markers (exclusive),
    and returns it with indentation added for all lines
    to match the indent of the marker line.
    """
    # Match entire block including markers (allow multiline)
    pattern = re.compile(
        rf"(^[ \t]*){re.escape(start_marker)}\n(.*?)\n^[ \t]*{re.escape(end_marker)}",
        re.DOTALL | re.MULTILINE
    )
    match = pattern.search(file_content)
    if match:
        indent = match.group(1)
        content = match.group(2)
        return content
    return None

def add_symbol_to_lines(text, symbol="#"):
    """Prepends the given symbol to the beginning of each line in the input text."""
    return '\n'.join(f"{symbol}{line}" for line in text.splitlines())



def insert_custom_section(generated_content,
                          custom_content,
                          start_marker=CUSTOM_SECTION_START_MARKER,
                          end_marker=CUSTOM_SECTION_END_MARKER):
    """
    Replace the section between start_marker and end_marker,
    preserving any leading whitespace before the markers.
    """

    # Regex to capture leading whitespace before each marker and the content in between
    pattern = re.compile(
        rf"(^[ \t]*){re.escape(start_marker)}.*?^[ \t]*{re.escape(end_marker)}",
        re.DOTALL | re.MULTILINE
    )

    def replacer(match):
        indent = match.group(1)  # capture leading whitespace (spaces or tabs)
        # Compose new block with preserved indent before each line of block
        new_block = f"{indent}{start_marker}\n"
        new_block += custom_content + "\n"
        new_block += f"{indent}{end_marker}"
        return new_block

    if pattern.search(generated_content):
        return pattern.sub(replacer, generated_content)
    else:
        # If markers not found, append custom block without indent
        new_block = f"{start_marker}\n{custom_content}\n{end_marker}"
        return f"{generated_content}\n{new_block}"

def preserve_user_content(file_path: Path, new_rendered_output):
    """Preserve user content between custom section markers."""
    if not file_path.exists():
        return new_rendered_output

    with open(file_path) as f:
        file_content = f.read()

    # Extract custom section from previous output
    custom_section_content = extract_custom_section(file_content)

    if custom_section_content:
        logger.debug(f"Custom section content:\n{custom_section_content}")

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

def render_template_folder(template_root: Path, destination_root: Path, context: dict):
    """Render the template folder and copy files to the destination."""
    for file in template_root.rglob("*"):
        logger.info(f"Processing file: {file}")
        
        rel_path = file.relative_to(template_root)
        dest_path = destination_root / render_path(rel_path, context)
        logger.debug(f"Destination path: {dest_path}")

        if dest_path.is_dir():
            dest_path.mkdir(parents=True, exist_ok=True)
            
        elif file.suffix == ".j2":
            with open(file) as f:
                template = Template(f.read())
                rendered = template.render(args=context)
            dest_path = dest_path.with_suffix("")  # Remove .j2 extension
            rendered = preserve_user_content(dest_path, rendered)
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
                content = preserve_user_content(dest_path, content)
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

class RosNoeticPackage(GitFlowRepo):
    def __init__(self, destination: Path, config_path: Path):
        self.config_path = config_path
        self.context = get_config(self.config_path)

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
        # Get the template folder based on the environment
        env = Path(self.context["package"]["environment"]) / "package" / self.context["package"]["language"]
        template_root = get_template_folder(env)
        
        # Render the template folder
        render_template_folder(template_root, self.working_dir, self.context)
        
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

