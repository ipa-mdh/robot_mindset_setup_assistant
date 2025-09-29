from pathlib import Path
import shutil

from setup_assistant.components import Level0TemplateComponent, Level1TemplateComponent
from setup_assistant.load_config import get_config
from setup_assistant.load_config import get_lookup_tables
from setup_assistant.package_versioning import GitFlowRepo
from setup_assistant.dev_setup import apply as apply_dev_setup
from setup_assistant.doxygen_awesome import apply as apply_doxygen_awesome
from setup_assistant.templates import TemplateRenderer


class RosNoeticPackage(GitFlowRepo):
    def __init__(self, destination: Path, config_path: Path, preserve_customizations: bool = True):
        self.config_path = config_path
        self.context = get_config(self.config_path)
        self.preserve_customizations = preserve_customizations

        lookup_tables = get_lookup_tables(self.context["package"]["environment"])
        self.context.update({"lookup": lookup_tables})

        self.package_name = self.context["package"]["name"]

        self.working_dir = destination / self.package_name
        self.working_dir.mkdir(parents=True, exist_ok=True)

        self.template_renderer = TemplateRenderer()
        self.level0_component = Level0TemplateComponent(self.template_renderer)
        self.level1_component = Level1TemplateComponent(self.template_renderer)

        super().__init__(self.working_dir)

        self.init()
        self.add_dev_setup()
        self.add_doxygen_awesome()

    @GitFlowRepo.decorator
    def init(self):
        """
        Initialize the package by creating the necessary directories and files.
        """

        self.level0_component.apply(
            destination=self.working_dir,
            context=self.context,
            preserve_customizations=self.preserve_customizations,
        )

        self.level1_component.apply(
            destination=self.working_dir,
            context=self.context,
            preserve_customizations=self.preserve_customizations,
        )

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
