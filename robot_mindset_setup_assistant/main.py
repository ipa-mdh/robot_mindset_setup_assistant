from pathlib import Path
from loguru import logger
import yaml
import shutil

from setup_assistant.create_pkg_ros_noetic import main as create_pkg_ros_noetic
from setup_assistant.create_pkg import main as create_pkg
# from setup_assistant.create_pkg_ros_humble import main as create_pkg_ros_humble

def load_config(config_path):
    """
    Load the configuration from a YAML file.
    """
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

if __name__ == "__main__":
    working_dir = Path("/workspace/src")
        
    # create working directory
    working_dir.mkdir(parents=True, exist_ok=True)
    
    # get this file location
    current_file_path = Path(__file__).resolve()
    # get the parent directory of this file
    parent_dir = current_file_path.parent
    
    # Load the configuration from the YAML file.
    config_path = parent_dir / Path("example/.robot_mindeset_setup_assistant.ros.noetic.yaml")
    if not config_path.exists():
        logger.error(f"Configuration file {config_path} does not exist.")
        exit(1)
        
    # Load the configuration
    config = load_config(config_path)
    
    # create_pkg_ros_noetic(package_name, working_dir, config, template_folder=template_folder)
    package_path = create_pkg(working_dir, config)
    
    # copy config to working dir
    config_file = package_path / ".robot_mindeset_setup_assistant.yaml"
    shutil.copy(config_path, config_file)