#!/bin/bash

# Function to modify the XML structure in memory
modify_config() {
    local interface_name="$1"
    local peer_address="$2"

    # Check if the XML file exists
    if [ ! -f "$CYCLONEDDS_URI" ]; then
        echo "[ERROR] Configuration file $CYCLONEDDS_URI does not exist."
        exit 1
    fi

    # Read the XML file into a variable
    local xml_content
    xml_content=$(cat $CYCLONEDDS_URI)

    # Append new network interface with correct indentation
    xml_content=$(echo "$xml_content" | sed "/<Interfaces>/a\                <NetworkInterface name=\"$interface_name\" priority=\"1\"/>")

    # Append new peer with correct indentation
    xml_content=$(echo "$xml_content" | sed "/<Peers>/a\                <Peer Address=\"$peer_address\"/>")

    # Export modified XML content as an environment variable
    export CYCLONEDDS_URI="$xml_content"
    export PAL_ROBOT_CONNECTED=1
    ros2 daemon stop > /dev/null 2>&1
}

# Check if enough arguments are provided
if [[ $# -ne 2 ]]; then
    echo "Usage: $0 <interface_name> <peer_address/hostname>"
    exit 1
fi

# Call the function with the provided arguments
if [[ $PAL_ROBOT_CONNECTED -eq 1 ]]; then
    echo "A connection is already stablished, please disconnect before connecting to another robot"
    exit 1
fi

# Modify the configuration as needed
modify_config "$1" "$2"
if [ $? -ne 0 ]; then
  echo "Failed to modify configuration"
  exit 1
fi

# Start a new shell with a modified prompt
exec bash --rcfile <(echo -e "source ~/.bashrc\nPS1=\"\$PS1\[\e[33m\]($2)\[\e[0m\] \"")
