import numpy as np
import argparse
import os
import sys
import logging
import csv

# Set up logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger()

def generate_signal(signal_type, duration, sample_period, amplitude, **params):
    # Generate time array
    t = np.arange(0, duration, sample_period)
    
    if signal_type == 'sine':
        frequency = params.get('frequency', 1)
        logger.info(f"Generating Sine Wave with frequency: {frequency} Hz")
        return amplitude*np.sin(2 * np.pi * frequency * t), t
    
    elif signal_type == 'ramp':
        slope = params.get('slope', 1)
        logger.info(f"Generating Ramp Wave with slope: {slope}")
        return slope * t, t
    
    elif signal_type == 'chirp':
        f0 = params.get('f0', 1)  # start frequency
        f1 = params.get('f1', 10) # end frequency
        logger.info(f"Generating Chirp Wave from {f0} Hz to {f1} Hz")
        return amplitude*np.sin(2 * np.pi * (f0 + (f1 - f0) * t / duration) * t), t

    else:
        raise ValueError(f"Unsupported signal type: {signal_type}")

def save_signal_to_csv(signal, time, filename):
    logger.info(f"Saving signal to CSV file: {filename}")
    try:
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time', 'Signal'])  # CSV header
            for t, s in zip(time, signal):
                writer.writerow([round(t, 4), round(s,3)])
        logger.info(f"Signal saved successfully to {filename}")
    except Exception as e:
        logger.error(f"Error saving CSV file: {e}")
        sys.exit(1)

def main():
    parser = argparse.ArgumentParser(description="Generate and save various signals: Sine, Ramp, Chirp.")
    parser.add_argument('signal_type', choices=['sine', 'ramp', 'chirp'], help="Type of signal to generate")
    parser.add_argument('--duration', type=float, default=1.0, help="Duration of the signal in seconds")
    parser.add_argument('--sample_period', type=float, default=0.001, help="Sample period (time step) in seconds")
    parser.add_argument('--output', type=str, required=True, help="Output CSV file to save the signal data")
    parser.add_argument('--amplitude', type=float, required=True, help="Signal Amplitude")
    # Parameters for each signal type
    parser.add_argument('--frequency', type=float, default=1, help="Frequency for sine wave (Hz)")
    parser.add_argument('--slope', type=float, default=1, help="Slope for ramp wave")
    parser.add_argument('--f0', type=float, default=1, help="Start frequency for chirp (Hz)")
    parser.add_argument('--f1', type=float, default=10, help="End frequency for chirp (Hz)")

    args = parser.parse_args()

    # Generate the signal
    try:
        signal, time = generate_signal(args.signal_type, args.duration, args.sample_period, args.amplitude,
                                       frequency=args.frequency, slope=args.slope, f0=args.f0, f1=args.f1)
        save_signal_to_csv(signal, time, args.output)
    except Exception as e:
        logger.error(f"Error occurred: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
