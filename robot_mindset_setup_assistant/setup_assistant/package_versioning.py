from functools import wraps
from loguru import logger
from git import Repo, GitCommandError
from pathlib import Path
import subprocess

# Config
MAIN_BRANCH = "main"
DEVELOP_BRANCH = "develop"

def run(cmd, **kwargs):
    """Run a shell command with subprocess, print it, and raise on error."""
    logger.info(f"> {' '.join(cmd)}")
    subprocess.run(cmd, check=True, **kwargs)

class GitFlowRepo:
    def __init__(self, repo_path="."):
        if not isinstance(repo_path, Path):
            self.repo_path = Path(repo_path)
        else:
            self.repo_path = repo_path
        
        if (self.repo_path / '.git').exists():
            self.repo = Repo(repo_path)
        else:
            self.repo = self._init_repo(repo_path)

        self._ensure_branches()

    def _init_repo(self, repo_path):
        """Ensure the repository is initialized"""
        repo = Repo.init(repo_path)
        # repo.git.branch(MAIN_BRANCH)
        repo.index.add(['*'])
        repo.index.commit("Initial commit")
        
        return repo
        

    def _ensure_branches(self):
        """Make sure main and develop branches exist"""
        git = self.repo.git
        branches = [head.name for head in self.repo.heads]

        if MAIN_BRANCH not in branches:
            logger.info(f"Creating {MAIN_BRANCH} branch")
            self.repo.git.checkout(b="main")
        else:
            git.checkout(MAIN_BRANCH)

        if DEVELOP_BRANCH not in branches:
            logger.info(f"Creating {DEVELOP_BRANCH} branch from {MAIN_BRANCH}")
            self.repo.git.checkout("-b", DEVELOP_BRANCH)
        else:
            git.checkout(DEVELOP_BRANCH)

    def start_feature(self, name):
        feature_branch = f"feature/{name}"
        logger.info(f"Starting feature: {feature_branch}")
        if feature_branch in self.repo.branches:
            logger.warning("The feature branch already exists. Switch to it.")
            self.repo.git.checkout(feature_branch)
        else:
            self.repo.git.checkout(DEVELOP_BRANCH)
            self.repo.git.checkout("-b", feature_branch)
        return feature_branch

    def finish_feature(self, name):
        feature_branch = f"feature/{name}"
        logger.info(f"Finishing feature: {feature_branch}")
        self.repo.git.checkout(DEVELOP_BRANCH)
        self.repo.git.merge(feature_branch)
        self.repo.git.branch("-d", feature_branch)

    def start_release(self, version):
        release_branch = f"release/{version}"
        logger.info(f"Starting release: {release_branch}")
        if release_branch in self.repo.branches:
            logger.warning("The realease branch already exists. Switch to it.")
            self.repo.git.checkout(release_branch)
        else:
            self.repo.git.checkout(DEVELOP_BRANCH)
            self.repo.git.checkout("-b", release_branch)
        return release_branch

    def finish_release(self, version):
        release_branch = f"release/{version}"
        logger.info(f"Finishing release: {release_branch}")
        self.repo.git.checkout(MAIN_BRANCH)
        self.repo.git.merge(release_branch)
        self.create_tag(version, message=f"Release {version}")
        self.repo.git.checkout(DEVELOP_BRANCH)
        self.repo.git.merge(release_branch)
        self.repo.git.branch("-d", release_branch)

    def start_hotfix(self, name):
        hotfix_branch = f"hotfix/{name}"
        logger.info(f"Starting hotfix: {hotfix_branch}")
        if hotfix_branch in self.repo.branches:
            logger.warning("The hotfix branch already exists. Switch to it.")
            self.repo.git.checkout(hotfix_branch)
        else:
            self.repo.git.checkout(MAIN_BRANCH)
            self.repo.git.checkout("-b", hotfix_branch)
        return hotfix_branch

    def finish_hotfix(self, name):
        hotfix_branch = f"hotfix/{name}"
        logger.info(f"Finishing hotfix: {hotfix_branch}")
        self.repo.git.checkout(MAIN_BRANCH)
        self.repo.git.merge(hotfix_branch)
        self.repo.git.checkout(DEVELOP_BRANCH)
        self.repo.git.merge(hotfix_branch)
        self.repo.git.branch("-d", hotfix_branch)

    def add_submodule(self, path, url, branch="main"):
        if (self.repo_path / path).exists():
            logger.warning(f"Submodule already exists at {path}.")
        else:
            logger.info(f"Adding submodule: {path} at {self.repo_path / path} from {url}")
            # submodule = self.repo.create_submodule(name=name, path=path, url=url)
            # submodule_repo = submodule.module()
            # submodule_repo.git.checkout(branch)
            # submodule_repo.git.submodule("update", "--init", "--recursive")
            if not (self.repo_path / path).exists():
                run(["git", "submodule", "add", url, path], cwd=self.repo_path)
                run(["git", "checkout", branch], cwd=self.repo_path / path)
                run(["git", "submodule", "update", "--init", "--recursive"], cwd=self.repo_path)
            
    def create_tag(self, tag_name, message=""):
        logger.info(f"Creating tag: {tag_name}")
        if tag_name in self.repo.tags:
            logger.warning("The tag already exists.")
        else:
            self.repo.create_tag(tag_name, message=message)

    def add_all_commit(self, message=""):
        logger.info(f"Committing all changes with message: {message}")
        if self.repo.is_dirty(untracked_files=True):
            self.repo.git.add(['*'])
            self.repo.index.commit(message)
        else:
            logger.warning("There is nothing to commit.")
        
    def stash(self):
        logger.info("Stashing changes")
        if self.repo.is_dirty(untracked_files=True):
            run(["git", "stash", "--include-untracked"], cwd=self.repo_path)
        else:
            logger.warning("Repo isn't dirty, so there's nothing to stash.")

    def decorator(func):
        """
        Decorator to handle git flow feature branches.
        """
        def wrapper(self, *args, **kwargs):
            result = None
            feature_name = func.__name__
            self.start_feature(feature_name)
            try:
                result = func(self, *args, **kwargs)
                self.add_all_commit(f"Finished {feature_name}")
            except Exception as e:
                logger.error(f"Error in feature {feature_name}: {e}")
            finally:
                self.finish_feature(feature_name)
            return result
        return wrapper
        
# 🧪 Example Usage
if __name__ == "__main__":
    flow = GitFlowRepo("/workspace/example")

    # Uncomment to test:
    # flow.start_feature("awesome-feature")
    # flow.finish_feature("awesome-feature")

    # flow.start_release("v1.0.1")
    flow.finish_release("v1.0.1")

    # flow.start_hotfix("urgent-bug")
    # flow.finish_hotfix("urgent-bug")
