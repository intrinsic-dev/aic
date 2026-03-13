#!/usr/bin/env bash
# Run from the repository root. Updates image paths in docs/sphinx/source/*.md
# from ../../media/ to /./source/_static/assets/ so Sphinx resolves them
# inside the docs tree (works in Docker and on host).
set -e

REPO_ROOT="$(git rev-parse --show-toplevel)"
SOURCE_DIR="${REPO_ROOT}/docs"

cd "${SOURCE_DIR}"
find . -name "*.md" -exec sed -i 's|../../media/|./source/_static/assets/|g' {} +

echo "Done. Updated image paths in ${SOURCE_DIR}"