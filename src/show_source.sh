#!/usr/bin/env bash

# Recursively print the contents of text-like source/project files.
#
# Usage:
#   ./show_source.sh            # auto-detect project root if run from src/
#   ./show_source.sh .          # scan current directory
#   ./show_source.sh ..         # scan parent directory
#   ./show_source.sh /path/to/project
#
# The script skips common build/cache/VCS directories and only cats files that
# appear to be text, so binary files such as .obj files are ignored safely.

set -u

# If no directory is supplied, make a sensible guess:
# - if run from src/ and ../CMakeLists.txt exists, scan the project root
# - otherwise scan the current directory
if [[ $# -gt 0 ]]; then
    DIR="$1"
elif [[ "$(basename "$PWD")" == "src" && -f "../CMakeLists.txt" ]]; then
    DIR=".."
else
    DIR="."
fi

if [[ ! -d "$DIR" ]]; then
    echo "Error: '$DIR' is not a directory." >&2
    exit 1
fi

# Directories to skip while recursing.
PRUNE_DIRS=(
    ".git"
    "build"
    "cmake-build-debug"
    "cmake-build-release"
    ".cache"
    ".vscode"
    ".idea"
    "__pycache__"
)

# Build the prune expression for find.
PRUNE_EXPR=()
for d in "${PRUNE_DIRS[@]}"; do
    PRUNE_EXPR+=( -name "$d" -o )
done
unset 'PRUNE_EXPR[${#PRUNE_EXPR[@]}-1]'

# Return success if a file looks safe to cat.
can_cat() {
    local file="$1"

    # Empty files are safe to cat.
    [[ ! -s "$file" ]] && return 0

    # grep -Iq returns success for text files and failure for binary files.
    grep -Iq . "$file"
}

# Find files recursively, skip unwanted directories, sort the output, and cat
# only files that appear to be text.
find "$DIR" \
    \( -type d \( "${PRUNE_EXPR[@]}" \) -prune \) -o \
    -type f -print |
sort |
while IFS= read -r FILE; do
    if can_cat "$FILE"; then
        echo "===== $FILE ====="
        cat "$FILE"
        echo
    else
        echo "===== $FILE ====="
        echo "[skipped: binary or non-text file]"
        echo
    fi
done
