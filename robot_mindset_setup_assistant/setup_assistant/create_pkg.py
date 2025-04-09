from jinja2 import Template
from pathlib import Path
import shutil
import yaml
from loguru import logger

# template_root = Path("template/ros_noetic")
# destination_root = Path("generated_package")

# context = yaml.safe_load(open("example/test.yaml"))
# package_name = context["package"]["name"]

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
                
def main(working_dir="/workspace", context=None):
    """
    Main function to render the template folder and copy files to the destination.
    Arguments:
    - working_dir: The working directory where the package will be created.
    - context: The context dictionary to render the templates.
    Returns:
    - destination_root: The path to the generated package.
    """
    
    type = "ros_noetic"  # or "ros_humble"
    destination_root = Path(working_dir) / context["package"]["name"]
    
    # get this file location
    current_file_path = Path(__file__).resolve()
    # get the parent directory of this file
    parent_dir = current_file_path.parent

    if type == "ros_humble":
        template_root = parent_dir / Path("template/ros_humble")
    elif type == "ros_noetic":
        template_root = parent_dir / Path("template/ros_noetic")
    else:
        raise ValueError(f"Unknown type: {type}")

    # Create the destination root directory if it doesn't exist
    destination_root.mkdir(parents=True, exist_ok=True)

    # Render the template folder
    render_template_folder(template_root, destination_root, context)
    
    return destination_root

