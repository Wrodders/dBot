#!/bin/bash
set -e

# 1. Perform motion analysis with vidstabdetect.
#    This generates the transforms file needed by vidstabtransform.
ffmpeg -i output.mp4 -vf "histeq,vidstabdetect=shakiness=2:accuracy=15" -f null - \
  || { echo "Motion analysis failed"; exit 1; }

# 2. Run stabilization commands in parallel using GNU Parallel.
#    We add 'crop=0' to vidstabtransform to avoid cropping (adjust if black borders appear).
parallel --halt soon,fail=1 ::: \
  "ffmpeg -i output.mp4 -vf \"vidstabtransform=zoom=5:input=transforms.trf:crop=0\" -c:v libx264 -crf 18 stabilized_output.mp4" \
  "ffmpeg -i output.mp4 -vf deshake -c:v libx264 -crf 18 reshaped_output.mp4"

# 3. Compare the three videos (Original, vidstab, deshake) side-by-side.
#    Each input is scaled to a height of 360 pixels (width is computed to preserve aspect ratio).
ffmpeg -i output.mp4 -i stabilized_output.mp4 -i reshaped_output.mp4 \
  -filter_complex "\
    [0:v]scale=-2:360[orig]; \
    [2:v]scale=-2:360[reshape]; \
    [1:v]scale=-2:360[stab]; \
    [orig][stab][reshape]hstack=inputs=3" \
  comparison_scaled.mp4 -y

# 4. Play the side-by-side comparison video.
ffplay comparison_scaled.mp4
