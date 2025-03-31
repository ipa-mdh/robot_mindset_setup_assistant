
from subprocess import run, CalledProcessError
from loguru import logger
from pathlib import Path
import jinja2
import yaml

# Define the Jinja2 template for a ROS 1 launch file with <arg> tags.
# Example data structure for the launch file template:
# data = { "description": None,
#          "default_arguments": None,
#          "arguments": None }
LAUNCH_TEMPLATE = """
<!-- This file is auto-generated. -->
<launch>
{% if args.description  %}<!-- {{ args.description }} -->{% endif %}

<!-- Default arguments -->
{% if args.default_arguments %}{% for arg in args.default_arguments %}
  <arg name="{{ arg.name }}"
    default="{{ arg.default }}"
        doc="{{ arg.description }}" />
{% endfor %}{% endif %}

<!-- Specific arguments -->
{% if args.arguments  %}{% for arg in args.arguments %}
  <arg name="{{ arg.name }}"
    default="{{ arg.default }}"
        doc="{{ arg.description }}" />
{% endfor %}{% endif %}
<!--  Place the main part of the launch file here. -->



</launch>
"""

DESCRIPTION_SINGLE_SERVE_LAUNCH_FILE = """Single_serve starts the nodes that may be required by several nodes, 
but may only run once. """

DESCRIPTION_LINK_SERVE_LAUNCH_FILE = """Link_serve starts the nodes that should only run once, e.g. to publish
the joint_states of a local node in a namespace collected in a global topic."""

DESCRIPTION_MULTI_SERVE_LAUNCH_FILE = """Multi_serve starts the nodes that are to be started for each instance
with the corresponding instance configurations."""

PATH_MULTI_SERVE_LAUNCH_FILE = Path("launch/multi_serve.launch")
PATH_SINGLE_SERVE_LAUNCH_FILE = Path("example/launch/single_serve.launch")
PATH_LINK_SERVE_LAUNCH_FILE = Path("example/launch/link_serve.launch")
PATH_MAIN_LAUNCH_FILE = Path("example/launch/main.launch")



def load_config(config_path):
    """
    Load the configuration from a YAML file.
    """
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

def create_catkin_package(package_name = "beginner_tutorials",
                          dependencies = ["std_msgs", "rospy", "roscpp"],
                          distro = "noetic",
                          working_dir = ""):
    
    command = ["catkin_create_pkg", package_name] + dependencies + ["--rosdistro", distro]

    logger.info(f"Running command: {' '.join(command)}")

    try:
        run(command, cwd=working_dir, check=True)
        logger.success(f"Package '{package_name}' created successfully.")
    except CalledProcessError as e:
        logger.error(f"Error creating package: {e}")

def create_launch_file_from_template(data, path):
    template = jinja2.Template(LAUNCH_TEMPLATE)
        
    # logger.debug(f"Creating launch file with data: {data}")
    launch_content = template.render(args=data)
    # logger.debug(f"Generated launch file content:\n{launch_content}")
    
    # Create the directory if it doesn't exist
    launch_dir = Path(path).parent
    launch_dir.mkdir(parents=True, exist_ok=True)
    
    # Write the launch file
    with open(path, "w") as f:
        f.write(launch_content)

def create_launch_files(config, working_dir):
    logger.info("Creating launch files...")
    
    data = {}
    data["arguments"] = config["launch"]["default_for_all"]["arguments"]
    data["default_arguments"] = config["launch"]["multi_serve"]["arguments"]
    data["description"] = DESCRIPTION_MULTI_SERVE_LAUNCH_FILE
    path = working_dir / PATH_MULTI_SERVE_LAUNCH_FILE
    create_launch_file_from_template(data, path)
    
    data["default_arguments"] = config["launch"]["single_serve"]["arguments"]
    data["description"] = DESCRIPTION_SINGLE_SERVE_LAUNCH_FILE
    path = working_dir / PATH_SINGLE_SERVE_LAUNCH_FILE
    create_launch_file_from_template(data, path)
    
    create_launch_file_from_template(
        config["launch"]["default_for_all"]["arguments"],
        config["launch"]["single_serve"]["arguments"],
        DESCRIPTION_SINGLE_SERVE_LAUNCH_FILE,
        working_dir / PATH_SINGLE_SERVE_LAUNCH_FILE
    )
    create_launch_file_from_template(
        config["launch"]["default_for_all"]["arguments"],
        None,
        "Main launch file",
        working_dir / PATH_MAIN_LAUNCH_FILE
    )
    create_launch_file_from_template(
        config["launch"]["default_for_all"]["arguments"],
        config["launch"]["link_serve"]["arguments"],
        DESCRIPTION_LINK_SERVE_LAUNCH_FILE,
        working_dir / PATH_LINK_SERVE_LAUNCH_FILE
    )

if __name__ == "__main__":
    working_dir = Path("/workspace/example_auto")
    package_name = "robot_ur_spark"
    
    # create working directory
    working_dir.mkdir(parents=True, exist_ok=True)
    
    # Load the configuration from the YAML file.
    config_path = Path(".robot_mindeset_setup_assistant.yaml")
    config = load_config(config_path)
    
    # Create the package
    create_catkin_package(package_name=package_name, working_dir=working_dir)
    
    # Create launch directory
    launch_dir = Path(working_dir) / package_name / "launch"
    launch_dir.mkdir(parents=True, exist_ok=True)
    # Create launch file
    create_launch_files(config, working_dir=working_dir / package_name)
