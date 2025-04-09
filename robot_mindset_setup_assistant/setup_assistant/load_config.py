from pathlib import Path
import yaml
from jinja2 import Template
from loguru import logger

def get_config(path:Path):
    processed = {}
    if path.exists():
        with open(path, "r") as f:
            raw = f.read()
            data_org = yaml.safe_load(raw)
            template = Template(raw)
            rendered = template.render(**data_org)
            logger.info("Rendered template:")
            logger.info(rendered)
            
            processed = yaml.safe_load(rendered)

    return processed