from pathlib import Path
import yaml
from jinja2 import Template
from loguru import logger

CONFIG_FOLDER = Path(__file__).parent.parent / "config"
LOOKUP_TABLES_FOLDER_NAME = "lookup_tables"

def dot_to_forward_slash(string: str) -> str:
    """Convert dots to forward slashes in a string."""
    if not isinstance(string, str):
        string = str(string)
    return string.replace(".", "/")

def load_yaml(path: Path) -> dict:
    if path.exists():
        with open(path, "r") as f:
            return yaml.safe_load(f)
    else:
        logger.warning(f"YAML file not found: {path}")
    return {}

def get_config(path:Path) -> dict:
    """ Load and process a YAML configuration file with Jinja2 templating. """
    processed = {}
    if path.exists():
        with open(path, "r") as f:
            raw = f.read()
            data_org = yaml.safe_load(raw)
            template = Template(raw)
            rendered = template.render(args=data_org)
            logger.debug("Rendered template:")
            logger.debug(rendered)
            
            processed = yaml.safe_load(rendered)

    return processed

def get_lookup_tables(environment: str) -> dict:
    """ load lookup tables """
    data = {}

    path = CONFIG_FOLDER
    path /= dot_to_forward_slash(environment)
    path /= LOOKUP_TABLES_FOLDER_NAME
    # get files in path
    files = path.glob("*.yaml")
    for file in files:
        logger.debug(f"Loading lookup table: {file}")
        lu = load_yaml(file)
        data.update({file.stem: lu})
    
    return data

if __name__ == "__main__":
    # for testing
    env = "ros.humble"
    lookup = get_lookup_tables(env)
    print(lookup)