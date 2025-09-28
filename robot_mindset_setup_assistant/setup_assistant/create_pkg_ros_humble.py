from subprocess import run, CalledProcessError
from loguru import logger

def create_catkin_package(package_name = "beginner_tutorials",
                          dependencies = ["std_msgs", "rospy", "roscpp"],
                          distro = "noetic"):
    
    command = ["catkin_create_pkg", package_name] + dependencies + ["--rosdistro", distro]

    logger.info(f"Running command: {' '.join(command)}")

    try:
        run(command, check=True)
        logger.success(f"Package '{package_name}' created successfully.")
    except CalledProcessError as e:
        logger.error(f"Error creating package: {e}")

if __name__ == "__main__":
    create_catkin_package()
