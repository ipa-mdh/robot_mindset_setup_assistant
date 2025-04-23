import subprocess
from pathlib import Path
from loguru import logger

from setup_assistant.package_versioning import GitFlowRepo

# Configuration
submodule_url = "https://github.com/ipa-mdh/dev-setup.git"  # replace with actual URL
submodule_path = "dev-setup"
submodule_branch = "master"

def run(cmd, **kwargs):
    """Run a shell command with subprocess, print it, and raise on error."""
    logger.info(f"> {' '.join(cmd)}")
    subprocess.run(cmd, check=True, **kwargs)

def apply(flow: GitFlowRepo, working_dir: Path, environment: str = "ros.noetic", package_name: str = "new_package"):
    """
    Main function to set up the development environment.
    Arguments:
    - flow: The GitFlowRepo instance to manage the repository.
    - working_dir: The working directory where the setup will be applied.
    """
    
    # Add submodule
    logger.info(f"Adding submodule {submodule_url} at {submodule_path}")
    # flow.start_feature("dev-setup")
    flow.add_submodule("dev-setup", submodule_url, submodule_branch)

    setup_command = ["python3", "dev-setup/setup.py", "--environment", f"{environment}", "--package", f"{package_name}", "--yes"]

    # Run setup.py from submodule
    run(setup_command, cwd=working_dir)
