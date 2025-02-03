import numpy as np
import argparse
import os
import sys
import logging
import csv

from dataclasses import dataclass

@dataclass
class BaseSigParameters:
    duration: float = 1.0
    sample_period: float = 0.001
    amplitude: float = 1.0

@dataclass
class PulseSigParameters(BaseSigParameters):
    on_time: float = 0.0
    on_value: float = 1.0
    off_time: float = 0.0
    off_value: float = 0.0

@dataclass
class SineSigParameters(BaseSigParameters):
    frequency: float = 1.0

@dataclass
class RampSigParameters(BaseSigParameters):
    slope: float = 1.0

@dataclass
class ChirpSigParameters(BaseSigParameters):
    f0: float = 1.0
    f1: float = 10.0

# Set up logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger()

def generate_signal(params: BaseSigParameters):
    t = np.arange(0, params.duration, params.sample_period)
    
    if isinstance(params, SineSigParameters):
        logger.info(f"Generating Sine Wave with frequency: {params.frequency} Hz")
        return params.amplitude * np.sin(2 * np.pi * params.frequency * t), t
    
    elif isinstance(params, RampSigParameters):
        logger.info(f"Generating Ramp Wave with slope: {params.slope}")
        return params.slope * t, t
    
    elif isinstance(params, ChirpSigParameters):
        logger.info(f"Generating Chirp Wave from {params.f0} Hz to {params.f1} Hz")
        return params.amplitude * np.sin(2 * np.pi * (params.f0 + (params.f1 - params.f0) * t / params.duration) * t), t
    
    else:
        raise ValueError(f"Unsupported signal parameters: {type(params)}")

def save_signal_to_csv(signal, time, filename):
    logger.info(f"Saving signal to CSV file: {filename}")
    try:
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time', 'Signal'])
            for t, s in zip(time, signal):
                writer.writerow([round(t, 4), round(s, 3)])
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
    parser.add_argument('--amplitude', type=float, default=1.0, help="Signal Amplitude")
    parser.add_argument('--frequency', type=float, default=1, help="Frequency for sine wave (Hz)")
    parser.add_argument('--slope', type=float, default=1, help="Slope for ramp wave")
    parser.add_argument('--f0', type=float, default=1, help="Start frequency for chirp (Hz)")
    parser.add_argument('--f1', type=float, default=10, help="End frequency for chirp (Hz)")
    
    args = parser.parse_args()
    
    if args.signal_type == 'sine':
        params = SineSigParameters(duration=args.duration, sample_period=args.sample_period, amplitude=args.amplitude, frequency=args.frequency)
    elif args.signal_type == 'ramp':
        params = RampSigParameters(duration=args.duration, sample_period=args.sample_period, amplitude=args.amplitude, slope=args.slope)
    elif args.signal_type == 'chirp':
        params = ChirpSigParameters(duration=args.duration, sample_period=args.sample_period, amplitude=args.amplitude, f0=args.f0, f1=args.f1)
    else:
        logger.error(f"Unsupported signal type: {args.signal_type}")
        sys.exit(1)
    
    try:
        signal, time = generate_signal(params)
        save_signal_to_csv(signal, time, args.output)
    except Exception as e:
        logger.error(f"Error occurred: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
