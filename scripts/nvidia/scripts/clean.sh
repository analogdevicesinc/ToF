#!/bin/bash

COPY_LOG="copied_files.txt"

if [ ! -f "$COPY_LOG" ]; then
    echo "❌ Log file not found: $COPY_LOG"
    exit 1
fi

echo "🧹 Starting cleanup from: $COPY_LOG"

while IFS= read -r file; do
    if [ -e "$file" ]; then
        echo "  🗑️ Removing: $file"
        rm -f "$file"
    else
        echo "  ⚠️ File not found (skipping): $file"
    fi
done < "$COPY_LOG"

rm "$COPY_LOG"

echo "✅ Cleanup complete!"
