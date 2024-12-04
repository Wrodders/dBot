#!/bin/bash

# ----------------------------------------------------------------------------
# Script Configuration: Set the parameters for signal generation
# ----------------------------------------------------------------------------

# Duration and Sample Period for signals (in seconds)
DURATION=5.0        # s
SAMPLE_PERIOD=0.01  # s
AMPLITUDE=1.0       # V
FREQUENCY_SINE=2    # Hz
SLOPE_RAMP=1        # mx slope
F0_CHIRP=1          # Start frequency (Hz)
F1_CHIRP=5         # End frequency (Hz)
# Output folder to store the generated CSV files
OUTPUT_DIR="./generated_signals"
mkdir -p "$OUTPUT_DIR"  # Create the directory if it doesn't exist

# ----------------------------------------------------------------------------
# Generate Signals and View them
# ----------------------------------------------------------------------------

# Generate Sine Signal
SINE_OUTPUT="$OUTPUT_DIR/sine_wave.csv"
echo "Generating Sine Wave with frequency $FREQUENCY_SINE Hz, saving to $SINE_OUTPUT"
python3 sigGen.py sine --duration $DURATION --sample_period $SAMPLE_PERIOD --amplitude $AMPLITUDE --frequency $FREQUENCY_SINE --output $SINE_OUTPUT

# Generate Ramp Signal
RAMP_OUTPUT="$OUTPUT_DIR/ramp_wave.csv"
echo "Generating Ramp Wave with slope $SLOPE_RAMP, saving to $RAMP_OUTPUT"
python3 sigGen.py ramp --duration $DURATION --sample_period $SAMPLE_PERIOD --amplitude $AMPLITUDE --slope $SLOPE_RAMP --output $RAMP_OUTPUT

# Generate Chirp Signal
CHIRP_OUTPUT="$OUTPUT_DIR/chirp_wave.csv"
echo "Generating Chirp Wave from $F0_CHIRP Hz to $F1_CHIRP Hz, saving to $CHIRP_OUTPUT"
python3 sigGen.py chirp --duration $DURATION --sample_period $SAMPLE_PERIOD --amplitude $AMPLITUDE --f0 $F0_CHIRP --f1 $F1_CHIRP --output $CHIRP_OUTPUT

# ----------------------------------------------------------------------------
# View Generated Signals
# ----------------------------------------------------------------------------

echo "Now visualizing the generated signals..."

# View Sine Wave
echo "Displaying Sine Wave from $SINE_OUTPUT"
python3 sigView.py "$SINE_OUTPUT"

# View Ramp Wave
echo "Displaying Ramp Wave from $RAMP_OUTPUT"
python3 sigView.py "$RAMP_OUTPUT"

# View Chirp Wave
echo "Displaying Chirp Wave from $CHIRP_OUTPUT"
python3 sigView.py "$CHIRP_OUTPUT"

echo "All signals generated and visualized successfully!"
