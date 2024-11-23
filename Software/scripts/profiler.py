import time
import psutil
import os
import argparse
import subprocess

def monitor_resources(pid, interval=0.1):
    """
    Monitors CPU and memory usage of a process.
    :param pid: Process ID to monitor.
    :param interval: Time interval (in seconds) between samples.
    :return: Dictionary with average CPU and memory usage.
    """
    process = psutil.Process(pid)
    cpu_usage = []
    memory_usage = []

    try:
        while process.is_running():
            cpu = process.cpu_percent(interval=interval)  # CPU percent
            memory = process.memory_info().rss / (1024 ** 2)  # Memory in MB
            cpu_usage.append(cpu)
            memory_usage.append(memory)
    except psutil.NoSuchProcess:
        pass  # Process has terminated

    return {
        "avg_cpu": sum(cpu_usage) / len(cpu_usage) if cpu_usage else 0,
        "max_cpu": max(cpu_usage) if cpu_usage else 0,
        "avg_memory": sum(memory_usage) / len(memory_usage) if memory_usage else 0,
        "max_memory": max(memory_usage) if memory_usage else 0,
    }

def run_and_profile(command, interval=0.1):
    """
    Runs a command and profiles its resource usage.
    :param command: Command to execute as a list.
    :param interval: Time interval (in seconds) between resource usage samples.
    :return: Execution statistics.
    """
    start_time = time.time()
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    # Monitor the process
    resources = monitor_resources(process.pid, interval)
    
    stdout, stderr = process.communicate()  # Wait for process to finish
    end_time = time.time()
    elapsed_time = end_time - start_time

    # Check return code for errors
    if process.returncode != 0:
        print(f"Error: Process exited with code {process.returncode}. Stderr:\n{stderr.decode()}")

    resources["cycle_time"] = elapsed_time
    return resources

def main():
    parser = argparse.ArgumentParser(description="Performance analysis for program efficiency.")
    parser.add_argument(
        "program",
        type=str,
        help="Path to the program or script to profile."
    )
    parser.add_argument(
        "args",
        nargs=argparse.REMAINDER,
        help="Arguments to pass to the program."
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.1,
        help="Sampling interval in seconds (default: 0.1s)."
    )
    args = parser.parse_args()

    # Build the command
    command = [args.program] + args.args

    print(f"Running command: {' '.join(command)}")
    stats = run_and_profile(command, interval=args.interval)

    # Display performance results
    print("\nPerformance Analysis:")
    print(f"  Average CPU Usage: {stats['avg_cpu']:.2f}%")
    print(f"  Max CPU Usage: {stats['max_cpu']:.2f}%")
    print(f"  Average Memory Usage: {stats['avg_memory']:.2f} MB")
    print(f"  Max Memory Usage: {stats['max_memory']:.2f} MB")
    print(f"  Cycle Time: {stats['cycle_time']:.2f} seconds")

if __name__ == "__main__":
    main()
