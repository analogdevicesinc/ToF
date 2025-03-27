#!/bin/bash

BASE_DST_PREFIX="../adcam-installer/resources"

#########################
### Clean staging folder
#########################
if [ ! -d "$BASE_DST_PREFIX" ]; then
    mkdir "$BASE_DST_PREFIX"
fi
if [ -d "$BASE_DST_PREFIX" ]; then
    echo "Cleaning all contents inside: $BASE_DST_PREFIX"

    # Deletes everything inside the folder — but not the folder itself
    find "$BASE_DST_PREFIX" -mindepth 1 -delete

    echo "🧼 Folder cleaned!"
else
    echo "❌ Directory does not exist: $BASE_DST_PREFIX"
fi