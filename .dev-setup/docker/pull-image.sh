#!/bin/bash

docker login container-registry.gitlab.cc-asp.fraunhofer.de

# --- Pull and tag images ---
# robot_mindset_setup_assistant:base-1.0
docker pull container-registry.gitlab.cc-asp.fraunhofer.de/multirobot/robot_mindset_setup_assistant:base-1.0
docker tag container-registry.gitlab.cc-asp.fraunhofer.de/multirobot/robot_mindset_setup_assistant:base-1.0 robot_mindset_setup_assistant:base-1.0
# robot_mindset_setup_assistant:ci-1.0
docker pull container-registry.gitlab.cc-asp.fraunhofer.de/multirobot/robot_mindset_setup_assistant:ci-1.0
docker tag container-registry.gitlab.cc-asp.fraunhofer.de/multirobot/robot_mindset_setup_assistant:ci-1.0 robot_mindset_setup_assistant:ci-1.0