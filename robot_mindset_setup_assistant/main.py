from pathlib import Path
from loguru import logger


from setup_assistant.create_pkg import RosNoeticPackage


if __name__ == "__main__":
    working_dir = Path("/workspace/src")
        
    # create working directory
    working_dir.mkdir(parents=True, exist_ok=True)
    
    # get this file location
    current_file_path = Path(__file__).resolve()
    # get the parent directory of this file
    parent_dir = current_file_path.parent
    
    # Load the configuration from the YAML file.
    config_path = parent_dir / Path("example/.robot_mindeset_setup_assistant.ros.humble.package.cpp.yaml")
    if not config_path.exists():
        logger.error(f"Configuration file {config_path} does not exist.")
        exit(1)

    pkg = RosNoeticPackage(working_dir, config_path)