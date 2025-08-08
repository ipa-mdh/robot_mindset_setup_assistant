#!/bin/bash


# Key Features
# - Attempts pull before prompting for login.
# - Prompts to log in only if the pull fails and you're not logged in.
# - Tries pulling again after login.
# - Prompts to build if pull fails post-login.
function try_pull_image() {
    LOCAL_IMAGE=$1
    REMOTE_IMAGE=$2
    REGISTRY_URL=$(echo "$REMOTE_IMAGE" | cut -d'/' -f1)

    # Try to pull the remote image first
    if docker pull ${REMOTE_IMAGE}; then
        docker tag ${REMOTE_IMAGE} ${LOCAL_IMAGE}
        echo "Successfully pulled remote image ${REMOTE_IMAGE}."
        exit 0
    else
        echo "Could not pull remote image ${REMOTE_IMAGE}."
    fi

    # Check if logged in: scan config.json for an entry for your registry
    function is_logged_in() {
        grep "\"$REGISTRY_URL\"" ~/.docker/config.json &>/dev/null
    }

    # Prompt for login if not logged in
    if ! is_logged_in; then
        read -p "You are not logged in to $REGISTRY_URL. Log in now? (y/n): " answer
        if [[ "$answer" =~ ^[Yy]$ ]]; then
            docker login $REGISTRY_URL || { echo "Docker login failed"; exit 1; }
        else
            echo "Cannot continue without login. Exiting."
            exit 1
        fi
    fi

    # Try pulling again after login
    if docker pull ${REMOTE_IMAGE}; then
        echo "Successfully pulled remote image ${REMOTE_IMAGE} after login."
        exit 0
    else
        echo "Remote image ${REMOTE_IMAGE} still not available after login."
        read -p "Do you want to build it locally? (y/n): " build_answer
        if [[ "$build_answer" =~ ^[Yy]$ ]]; then
            docker build -t ${LOCAL_IMAGE} . || { echo "Docker build failed"; exit 1; }
            docker tag ${LOCAL_IMAGE} ${REMOTE_IMAGE}
        else
            echo "No action taken."
            exit 0
        fi
    fi
}

# --- Pull / Build and tag images ---

# robot_mindset_setup_assistant:base-1.0
try_pull_image robot_mindset_setup_assistant:base-1.0 container-registry.gitlab.cc-asp.fraunhofer.de/multirobot/robot_mindset_setup_assistant:base-1.0

# robot_mindset_setup_assistant:ci-1.0
try_pull_image robot_mindset_setup_assistant:ci-1.0 container-registry.gitlab.cc-asp.fraunhofer.de/multirobot/robot_mindset_setup_assistant:ci-1.0

# robot_mindset_setup_assistant:run-1.0
docker build --file .dev-setup/docker/Dockerfile.run \
    --secret id=gitcreds,src=$HOME/.git-credentials \
    --tag robot_mindset_setup_assistant:run-1.0 \
    .