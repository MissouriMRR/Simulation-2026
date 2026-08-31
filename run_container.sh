#!/usr/bin/bash

compose=podman-compose
oci_cmd=podman

# Prebuilt images published to GitHub Container Registry.
env_image=ghcr.io/missourimrr/multirotor-env:latest
sim_image=ghcr.io/missourimrr/multirotor-sim:latest

function find_container_name() {
    # find container name that matches passed argument
    test=$(eval "$oci_cmd ps --format '{{.Names}}' | grep $1")
    echo $test  # return matched container name
}

# Pull the image if it the local env is missing it
function ensure_image() {
    if $oci_cmd image exists "$1"; then
        return 0
    fi
    echo "No local copy of $1 -- pulling..."
    if ! $oci_cmd pull "$1"; then
        echo "Pull failed. Compose will build from source instead (this takes a while)."
    fi
}

function attach_to() {
    local container
    container=$(find_container_name "$1")
    if [ -z "$container" ]; then
        echo "No running container matching '$1'. Start it first."
        return 1
    fi
    $oci_cmd attach "$container"
}

function pull_images() {
    local rc=0
    $oci_cmd pull "$env_image" || rc=1
    $oci_cmd pull "$sim_image" || rc=1
    if [ "$rc" -ne 0 ]; then
        echo "One or more pulls failed. Are the packages public, or are you logged in?"
    fi
    return $rc
}

function usage() {
    echo This script is intended as a simple wrapper for running the env and sim podman containers.
    echo 
    echo Usage:
    echo "  cmd [command]"
    echo
    echo Available Commands:
    echo "      [all]                 start both the sim and env containers (detached)"
    echo "      <sim|env>             start the specified container only (attached)"
    echo "      pull                  re-download the prebuilt images from ghcr.io"
    echo "      build                 rebuild both images locally from the Containerfiles"
    echo "      shutdown              shutdown both containers if any are running"
    echo "  h,  help                  print this"
    echo "      restart <all|sim|env> restart the specified container(s)"
    echo "      attach <sim|env>      attach the specified container (it must be running)"
    echo
}

# very simple command implementation
case $1 in
    ""|all)
        eval "$compose up -d"
    ;;
    pull)
        pull_images
    ;;
    build)
        $compose build
    ;;
    shutdown)
        eval "$compose down"
    ;;
    sim)
        ensure_image "$sim_image"
        $compose run --rm sim
    ;;
    env)
        ensure_image "$sim_image"
        $compose run --rm env
    ;;
    h|help)
        usage
    ;;
    restart)
        case $2 in
            ""|all)
                eval "$compose restart"
            ;;
            sim)
                eval "$compose restart sim"
            ;;
            env)
                eval "$compose restart env"
            ;;
            *)
                echo "Could not restart '$2'"
                usage
            ;;
        esac
    ;;
    attach)
        case ${2:-} in
            ""|env)
                attach_to env
            ;;
            sim)
                attach_to sim
            ;;
            *)
                echo "Could not attach to '$2'"
                usage
            ;;
        esac
    ;;
    *)
        echo "Unrecognized command '$1'"
        usage
    ;;
esac
