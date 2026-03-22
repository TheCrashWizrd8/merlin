#!/bin/bash
# Zips the dataset for upload to Google Colab.
# Run from the project root:  bash scripts/zip_dataset.sh
set -e
cd "$(dirname "$0")/.."
echo "Zipping dataset..."
zip -r dataset.zip data/images
echo ""
echo "Done!  File: $(pwd)/dataset.zip  ($(du -sh dataset.zip | cut -f1))"
echo ""
echo "Next: copy dataset.zip to your PC, then upload it in the Colab notebook."
