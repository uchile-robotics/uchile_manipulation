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
BACKUP_FOLDER="$UCHILE_WS"/deps/bender/soft/manipulation/arm_planning
BACKUP_FILE=bender_arm_planning.zip
BACKUP_FILE_FULL="$BACKUP_FOLDER/$BACKUP_FILE"
if [ ! -r "$BACKUP_FILE_FULL" ]; then
	
	# Download bender_arm_planning capability map
	echo "$installer Backup file not found ($BACKUP_FILE_FULL), downloading capability map file ..."
	"$UCHILE_WS"/system/shell/megadown/megadown 'https://mega.nz/#!e9NmnD6D!kXfuEhqlOtme87Cb1ZyiIDLI7XBzRdBWikLq0Pbu4KA'
	if [ $? -ne 0 ]; then
	    echo "$installer ${red}Error downloading capability map file.${reset}"
	    exit 1 # Terminate and indicate error
	fi
	echo "$installer Capability map file downloaded successfully."

	# create backup
	echo "$installer Creating backup on $BACKUP_FILE_FULL"
	mkdir -p "$BACKUP_FOLDER"
	cp "$BACKUP_FILE" "$BACKUP_FILE_FULL"
	
else
	echo "$installer A backup file for $BACKUP_FILE already exists on $BACKUP_FOLDER"
	cp "$BACKUP_FILE_FULL" "$WORK_DIR"
fi


#  - - - - - - - - - Extract files - - - - - - - - - - -

# Extract files
echo "$installer Extracting Capability map files"
unzip -q -o "$BACKUP_FILE" -d "$UCHILE_WS"/pkgs/soft_ws/uchile_manipulation/
OUT=$?
if [ $OUT -ne 0 ]; then
    echo "$installer ${red}Error extracting Capability map file.${reset}"

    # remove backup in error
    echo "removing backup ... "
    rm -f "$BACKUP_FILE_FULL"

    exit 1 # Terminate and indicate error
else
    echo "$installer Capability map files extracted successfully."
fi
