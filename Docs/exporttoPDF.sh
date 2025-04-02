#!/bin/bash
# convert_svgs.sh
#
# Usage: ./convert_svgs.sh <input_folder> <output_folder>
#
# This script converts all SVG files in the input folder to PDFs (with LaTeX text support)
# in the output folder. Each PDF will have the same base name as its corresponding SVG.
# Existing files are overwritten.
#
# Note: This uses Inkscape’s --export-latex option so that any LaTeX-formatted text
#       in your SVG (e.g., from draw.io) is exported as LaTeX overlay code in a companion
#       .pdf_tex file.

# Check for correct number of arguments
if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <input_folder> <output_folder>"
  exit 1
fi

input_folder="$1"
output_folder="$2"

# Create the output directory if it doesn't exist
mkdir -p "$output_folder"

# Loop over all SVG files in the input folder
shopt -s nullglob
for svg in "$input_folder"/*.svg; do
  # Get the base filename (without extension)
    base=$(basename "$svg" .svg)
    pdf_file="$output_folder/$base.pdf"
    echo "Converting $svg to $pdf_file with LaTeX text export..."
    # Convert the SVG to PDF with LaTeX overlay export
    inkscape -D --export-filename="$pdf_file" "$svg"  
  if [ $? -ne 0 ]; then
    echo "Conversion failed for $svg"
  else
    echo "Successfully converted $svg"
  fi
done

echo "All conversions complete."
