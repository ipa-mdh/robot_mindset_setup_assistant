#!/bin/bash

docker login container-registry.gitlab.cc-asp.fraunhofer.de

# --- Push images ---
# robot_mindset_setup_assistant:base-1.0
docker push container-registry.gitlab.cc-asp.fraunhofer.de/multirobot/robot_mindset_setup_assistant:base-1.0
# robot_mindset_setup_assistant:ci-1.0
docker push container-registry.gitlab.cc-asp.fraunhofer.de/multirobot/robot_mindset_setup_assistant:ci-1.0