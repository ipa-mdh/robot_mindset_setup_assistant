from enum import Enum
from jinja2 import Template
from pathlib import Path
import shutil
import yaml
from loguru import logger

from setup_assistant.load_config import get_config
from setup_assistant.package_versioning import GitFlowRepo
from setup_assistant.dev_setup import apply as add_dev_setup

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
            # create parent directories if they don't exist
            dest_path.parent.mkdir(parents=True, exist_ok=True)
            with open(dest_path, "w") as f:
                f.write(rendered)
        else:
            if file.is_file():
                logger.debug(f"Copying file: {file} to {dest_path}")
                # create parent directories if they don't exist
                dest_path.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy(src=file, dst=dest_path)

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
    parent_dir = current_file_path.parent

    def dot_to_underscore(string):
        """Convert dots to underscores in a string."""
        return string.replace(".", "_")

    # check if template directory exists
    template_root = parent_dir / Path("template") / dot_to_underscore(environment)
    if not template_root.exists():
        logger.error(f"Template directory {template_root} does not exist.")
        raise ValueError(f"Unknown environment: {environment}")
    
    return template_root

class RosNoeticPackage(GitFlowRepo):
    def __init__(self, destination: Path, config_path: Path):
        self.config_path = config_path
        
        self.context = get_config(self.config_path)
        self.package_name = self.context["package"]["name"]
        
        self.working_dir = destination / self.package_name
        self.working_dir.mkdir(parents=True, exist_ok=True)
        
        super().__init__(self.working_dir)
        
        self.init()
        self.add_dev_setup()
        
    def git_flow_feature(func):
        """
        Decorator to handle git flow feature branches.
        """
        def wrapper(self, *args, **kwargs):
            feature_name = func.__name__
            self.start_feature(feature_name)
            result = func(self, *args, **kwargs)
            self.add_all_commit(f"Finished {feature_name}")
            self.finish_feature(feature_name)
            return result
        return wrapper

    @git_flow_feature
    def init(self):
        """
        Initialize the package by creating the necessary directories and files.
        """
        
        # Get the template folder based on the environment
        template_root = get_template_folder(self.context["package"]["environment"])
        
        # Render the template folder
        render_template_folder(template_root, self.working_dir, self.context)
        
        # Copy config to working dir
        dest = self.working_dir / ".robot_mindeset_setup_assistant.yaml"
        shutil.copy(self.config_path, dest)
    
    @git_flow_feature
    def add_dev_setup(self):
        """
        Apply the dev setup to the package.
        """
        add_dev_setup(self, self.working_dir)
        
        
                
# def main(config_path: Path, working_dir="/workspace"):
#     """
#     Main function to render the template folder and copy files to the destination.
#     Arguments:
#     - config_path: Path to the configuration file.
#     - working_dir: The working directory where the package will be created.
#     Returns:
#     - destination_root: The path to the generated package.
#     """
    
#     # Load the configuration
#     context = get_config(config_path)
    
#     environment = context["package"]["environment"]  # or "ros_humble"
#     destination_root = Path(working_dir) / context["package"]["name"]
    
    

#     # Create the destination root directory if it doesn't exist
#     destination_root.mkdir(parents=True, exist_ok=True)

#     # Render the template folder
#     render_template_folder(template_root, destination_root, context)
    
#     # copy config to working dir
#     dest = destination_root / ".robot_mindeset_setup_assistant.yaml"
#     shutil.copy(config_path, dest)
    
#     return destination_root

