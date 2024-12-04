import numpy as np
import argparse
import logging
import matplotlib.pyplot as plt
import csv
import sys

# Set up logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger()

def load_signal_from_csv(filename):
    logger.info(f"Loading signal from CSV file: {filename}")
    try:
        time = []
        signal = []
        with open(filename, mode='r') as file:
            reader = csv.reader(file)
            next(reader)  # Skip the header row
            for row in reader:
                time.append(float(row[0]))
                signal.append(float(row[1]))
        return np.array(signal), np.array(time)
    except Exception as e:
        logger.error(f"Failed to load file {filename}: {e}")
        sys.exit(1)

def plot_signal(signal, time, signal_type):
    plt.figure(figsize=(10, 6))
    plt.plot(time, signal, label=f'{signal_type.capitalize()} Signal')
    plt.title(f'{signal_type.capitalize()} Signal')
    plt.xlabel('Time [s]')
    plt.ylabel('Amplitude')
    plt.grid(True)
    plt.legend()
    plt.show()

def main():
    parser = argparse.ArgumentParser(description="Load and view signals (Sine, Ramp, Chirp) from CSV file.")
    parser.add_argument('input', type=str, help="Input CSV file containing the signal data")
    parser.add_argument('--signal_type', choices=['sine', 'ramp', 'chirp'], help="Type of signal for plotting (optional)")
    args = parser.parse_args()

    # Load signal
    signal, time = load_signal_from_csv(args.input)

    # If signal_type is not specified, try to infer it from the filename or user input
    if not args.signal_type:
        filename = args.input.lower()
        if 'sine' in filename:
            signal_type = 'sine'
        elif 'ramp' in filename:
            signal_type = 'ramp'
        elif 'chirp' in filename:
            signal_type = 'chirp'
        else:
            signal_type = 'unknown'
        logger.info(f"Inferred signal type: {signal_type}")
    else:
        signal_type = args.signal_type
    
    # Plot the signal
    plot_signal(signal, time, signal_type)

if __name__ == '__main__':
    main()
