#!/bin/bash

# bender_arm_planning package installer
# run: $ bash bender_arm_planning/install/install.sh

#  - - - - - - - - - Setup - - - - - - - - - - - 
# Color
red=$(tput setaf 1)
green=$(tput setaf 2)
yellow=$(tput setaf 3)
reset=$(tput sgr0)
bold=$(tput bold)

# Line tittle
installer="${bold}[bender_arm_planning]:${reset}"

# The directory of the script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# the temp directory used, within $DIR
WORK_DIR="$(mktemp -d -p "$DIR")"
# deletes the temp directory
function cleanup {
  rm -rf "$WORK_DIR"
  echo "$installer Deleted temp working directory $WORK_DIR"
}
# Register the cleanup function to be called on the EXIT signal
trap cleanup EXIT
cd "$WORK_DIR"

#  - - - - - - - - - Download files - - - - - - - - - - - 
# Download bender_arm_planning capability map
echo "$installer Downloading capability map file"
"$BENDER_SYSTEM"/bash/megadown/megadown 'https://mega.nz/#!e9NmnD6D!kXfuEhqlOtme87Cb1ZyiIDLI7XBzRdBWikLq0Pbu4KA'
if [ $? -ne 0 ]; then
    echo "$installer ${red}Error downloading capability map file.${reset}"
    exit 1 # Terminate and indicate error
else
    echo "$installer Capability map file downloaded successfully."
fi

#  - - - - - - - - - Extract files - - - - - - - - - - -
# Extract files
echo "$installer Extracting Capability map files"
unzip -q -o bender_arm_planning.zip -d "$BENDER_WS"/soft_ws/src/beder_manipulation/
if [ $? -ne 0 ]; then
    echo "$installer ${red}Error extracting Capability map file.${reset}"
    exit 1 # Terminate and indicate error
else
    echo "$installer Capability map files extracted successfully."
fi

