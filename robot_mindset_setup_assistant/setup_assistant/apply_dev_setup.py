from git import Repo
import subprocess
from pathlib import Path
from loguru import logger

from setup_assistant.package_versioning import GitFlowRepo
# from package_versioning import GitFlowRepo

# Configuration
submodule_url = "https://github.com/ipa-mdh/dev-setup.git"  # replace with actual URL
submodule_path = "dev-setup"
submodule_branch = "feature/debian"
setup_command = ["python3", "dev-setup/setup.py", "--environment", "ros.noetic"]

def run(cmd, **kwargs):
    """Run a shell command with subprocess, print it, and raise on error."""
    logger.info(f"> {' '.join(cmd)}")
    subprocess.run(cmd, check=True, **kwargs)

# def main(working_dir: Path):
#     # cwd = Path.cwd()
#     cwd = working_dir

#     # 1. Initialize git repo
#     logger.info(f"Initializing git repository in {cwd}")
#     if not (cwd / ".git").exists():
#         run(["git", "config", "--global", "init.defaultBranch", "main"], cwd=cwd)
#         run(["git", "init"], cwd=cwd)

#         # 2. Initial commit (after creating a dummy file to commit)
#         logger.info("Creating initial commit")
#         if not (cwd / ".git").exists():
#             raise RuntimeError("Git repo not initialized")

#         # Create a .gitignore or README.md if needed
#         readme = cwd / "README.md"
#         if not readme.exists():
#             readme.write_text("# Initial Project\n")
            
#             run(["git", "add", "."], cwd=cwd)
#             run(["git", "commit", "-m", "Initial commit"], cwd=cwd)

#         # 3. Add submodule
#         logger.info(f"Adding submodule {submodule_url} at {submodule_path}")
#         if not (cwd / submodule_path).exists():
#             run(["git", "submodule", "add", "-b", submodule_branch, submodule_url, submodule_path], cwd=cwd)
#             run(["git", "submodule", "update", "--init", "--recursive"], cwd=cwd)

#     # 4. Run setup.py from the submodule
#     # run(setup_command)

def main(working_dir: Path):
    """
    Main function to set up the development environment.
    Arguments:
    - working_dir: The working directory where the setup will be applied.
    """
    # 1. Initialize the main repo
    flow = GitFlowRepo(working_dir)
    
    # 2. Add submodule
    logger.info(f"Adding submodule {submodule_url} at {submodule_path}")
    flow.start_feature("dev-setup")
    flow.add_submodule("dev-setup", submodule_url, submodule_branch)

    # 4. Run setup.py from submodule
    run(setup_command, cwd=working_dir)
    
    # # 5. commit
    logger.info("Committing changes")
    flow.add_all_commit("Added dev-setup submodule and ran setup.py")
    flow.finish_feature("dev-setup")

if __name__ == "__main__":
    # main(Path("/workspace/src/sim2real_gap"))
    main(Path("/workspace/src/sim2real_gap"))