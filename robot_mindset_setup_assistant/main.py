from pathlib import Path
import argparse
from loguru import logger


from setup_assistant.create_pkg import RosNoeticPackage


if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Robot Mindset Setup Assistant - Generate ROS packages from templates'
    )
    parser.add_argument(
        '--config', '-c',
        type=str,
        help='Path to the configuration YAML file'
    )
    parser.add_argument(
        '--working_dir', '--working-dir', '-w',
        type=str,
        default="/workspace/src",
        help='Working directory where the package will be generated (default: /workspace/src)'
    )
    
    args = parser.parse_args()
    
    # Set working directory from argument
    working_dir = Path(args.working_dir)
        
    # create working directory
    working_dir.mkdir(parents=True, exist_ok=True)
    
    # get this file location
    current_file_path = Path(__file__).resolve()
    # get the parent directory of this file
    parent_dir = current_file_path.parent
    
    # Determine config path
    if args.config:
        config_path = Path(args.config)
        # If relative path, make it relative to current working directory
        if not config_path.is_absolute():
            config_path = parent_dir / config_path
    else:
        # Use default config file
        config_path = parent_dir / Path("example/.robot_mindeset_setup_assistant.ros.humble.package.cpp.yaml")
    
    # Load the configuration from the YAML file.
    if not config_path.exists():
        logger.error(f"Configuration file {config_path} does not exist.")
        exit(1)

    logger.info(f"Using configuration: {config_path}")
    logger.info(f"Working directory: {working_dir}")

    pkg = RosNoeticPackage(working_dir, config_path)