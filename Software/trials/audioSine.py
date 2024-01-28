import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pydub import AudioSegment

def load_audio(file_path):
    try:
        audio = AudioSegment.from_mp3(file_path)
        data = np.array(audio.get_array_of_samples())
        normalized_data = data / np.max(np.abs(data))
        return normalized_data
    except Exception as e:
        print(f"Error loading audio: {e}")
        sys.exit(1)

# Check if the file path is provided as a command-line argument
if len(sys.argv) != 2:
    print("Usage: python script.py <mp3_file_path>")
    sys.exit(1)

file_path = sys.argv[1]
normalized_data = load_audio(file_path)

# Initialize matplotlib plot
fig, ax = plt.subplots()
x = np.fft.fftfreq(len(normalized_data), d=1.0/44100)  # Frequencies
line, = ax.plot(x, np.abs(np.fft.fft(normalized_data)))

# Function to update the plot
def update_plot(frame):
    line.set_ydata(frame)
    return line,

# Function to continuously update the plot with the audio data
def audio_stream(i):
    spectrum = np.abs(np.fft.fft(normalized_data))
    
    # Display only positive frequencies
    positive_frequencies = x[x > 0]
    magnitude = spectrum[:len(spectrum)//2]

    # Update the plot with the magnitude spectrum
    line.set_ydata(magnitude)
    ax.set_xlim(0, max(positive_frequencies))
    return line,

# Use FuncAnimation to animate the plot
ani = FuncAnimation(fig, audio_stream, blit=True)

# Keep the plot open until the user closes it
plt.show()
