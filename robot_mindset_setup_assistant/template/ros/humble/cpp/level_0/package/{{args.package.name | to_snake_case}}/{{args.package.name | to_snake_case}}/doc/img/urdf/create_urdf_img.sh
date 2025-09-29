#!/bin/bash

# This script generates the URDF documentation for the FANUC robot.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Set your main xacro/URDF filenames here
XACRO_INPUT="$SCRIPT_DIR/../../../example/urdf/example.xacro"
OUTPUT_DIR="$SCRIPT_DIR"
URDF_OUTPUT="$OUTPUT_DIR/robot.urdf"
PDF_OUTPUT="$OUTPUT_DIR/robot"
PNG_OUTPUT="$OUTPUT_DIR/robot.png"

if [ ! -f "$XACRO_INPUT" ]; then
  echo "Input Xacro file does not exist: $XACRO_INPUT"
  exit 1
fi

# Create output directory if it does not exist
mkdir -p "$OUTPUT_DIR"

# Step 1: Generate URDF from Xacro
echo "Converting Xacro to URDF..."
xacro "$XACRO_INPUT" > "$URDF_OUTPUT"
if [ $? -ne 0 ]; then
  echo "Failed to generate URDF from Xacro."
  exit 2
fi
echo "URDF generated: $URDF_OUTPUT"

# Step 2: Generate PDF using urdf_to_graphviz
echo "Generating PDF from URDF..."
urdf_to_graphviz "$URDF_OUTPUT" "$PDF_OUTPUT"
if [ $? -ne 0 ]; then
  echo "Failed to convert URDF to PDF."
  exit 3
fi

# Step 3: Convert dot to PNG
dot -Tpng "$PDF_OUTPUT.gv" -o "$PNG_OUTPUT"
if [ $? -ne 0 ]; then
  echo "Failed to convert dot to PNG."
  exit 4
fi
echo "PNG generated: $PNG_OUTPUT"