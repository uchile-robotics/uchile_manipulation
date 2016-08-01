#!/bin/bash

# run: $ ./scripts/install.sh

# The directory of the script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# the temp directory used, within $DIR
WORK_DIR="$(mktemp -d -p "$DIR")"
# deletes the temp directory
function cleanup {
  rm -rf "$WORK_DIR"
  echo "Deleted temp working directory $WORK_DIR"
}
# Register the cleanup function to be called on the EXIT signal
trap cleanup EXIT
cd "$WORK_DIR"


# Download bender_arm_planning capability map
"$BENDER_SYSTEM"/bash/megadown/megadown 'https://mega.nz/#!e9NmnD6D!kXfuEhqlOtme87Cb1ZyiIDLI7XBzRdBWikLq0Pbu4KA'

# Extract files
unzip bender_arm_planning.zip -d "$BENDER_WS"/soft_ws/src/bender_manipulation/


