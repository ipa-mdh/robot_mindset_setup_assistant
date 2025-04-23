from loguru import logger

from setup_assistant.package_versioning import GitFlowRepo

# Configuration
submodule_url = "https://github.com/jothepro/doxygen-awesome-css.git"
submodule_path = "doxygen-awesome-css"
submodule_branch = "tags/v2.3.4" # v2.3.4

def apply(flow: GitFlowRepo):
    """
    Main function to set up the development environment.
    Arguments:
    - flow: The GitFlowRepo instance to manage the repository.
    - working_dir: The working directory where the setup will be applied.
    """
    
    # Add submodule
    logger.info(f"Adding submodule {submodule_url} at {submodule_path}")
    # flow.start_feature("dev-setup")
    flow.add_submodule("doxygen-awesome-css", submodule_url, submodule_branch)
