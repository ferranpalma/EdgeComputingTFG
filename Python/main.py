# Imports
import serial
import struct
import matplotlib.pyplot as plt
import numpy as np
import scipy.fft as libfft
from scipy.signal.windows import blackman
from statistics import mean

# Define constants for the program
BUFFER_SIZE = 1024
N = BUFFER_SIZE*2
FREQUENCY = 40000
SAMPLE_FREQUENCY = 1/FREQUENCY

# Function to open the serial port to comunicate via UART. Always open
# Returns the instance of the serial port
def start_port():
    port = serial.Serial(
        port="/dev/tty.usbmodem11303",
        baudrate=115200,
        bytesize=8,
        parity="N",
        timeout=None,
    )

    port.reset_input_buffer()

    return port

#Program logic
def main():
    #Start datagram to sync program and ucontroller
    start_datagram = []
    #Buffer to store ucontroller data (8 bit integer)
    buffer = []
    #Buffer where reassembled data is stored
    mic_data = []
    #Variable to control how many data do we read
    index = 0

    port = start_port()

    print("Preparing to recieve the sync datagram!")

    #Read until start datagram arrives
    while 1:
        try:    
            data = port.read().decode('utf8', errors='strict')
            start_datagram.append(data)
        except UnicodeDecodeError:
            if len(start_datagram) == 0 :
                port.reset_input_buffer()
            continue
            
        if "".join(start_datagram[:-1]) == "Start TX":
            break
    
    
    print("Sync done, grabbing values...")

    # Gather the data from the ucontroller
    while index < BUFFER_SIZE*4:
        unpacked_data = struct.unpack('>B', port.read(size=1))[0]
        buffer.append(unpacked_data)
        index += 1

    index = 0

    print("Reassembling data...")

    # Reassemble the data (bit operations)
    while index < BUFFER_SIZE*4:
        reassembled_data = ((buffer[index] << 8) | buffer[index+1])
        mic_data.append(reassembled_data)
        index += 2

    print("Converting digital to analog data")

    # Convert digital value to the analog one
    analog_signal = list(map(lambda x : (float(x)*3.3)/float(4095), mic_data))


    # Center the signal by subtracting the mean to all the values
    centered_analog_signal = list(map(lambda x : (x - mean(analog_signal)), analog_signal))


    print("Data reassembled. Plotting...")

    fig1, (ax1, ax2) = plt.subplots(ncols=2, nrows=1)
    fig2, (ax3, ax4) = plt.subplots(ncols=2, nrows=1)
    
    # Plot the data

    ax1.plot(np.arange(start=0, stop=SAMPLE_FREQUENCY*N, step=SAMPLE_FREQUENCY), analog_signal)
    ax1.grid()
    ax1.set_xlabel("Time (us)")
    ax1.set_ylabel("Amplitude (V)")
    ax1.set_title("Sound wave in the temporal domain")

    ax3.plot(np.arange(start=0, stop=SAMPLE_FREQUENCY*N, step=SAMPLE_FREQUENCY), centered_analog_signal)
    ax3.grid()
    ax3.set_xlabel("Time (us)")
    ax3.set_ylabel("Amplitude (V)")
    ax3.set_title("Sound wave in the temporal domain (DC offset compensated)")


    # Transform to the frequency domain and plot
    yf = libfft.fft(analog_signal)
    yf_centered = libfft.fft(centered_analog_signal)

    # DFT sample frequencies array
    xf = libfft.fftfreq(N,SAMPLE_FREQUENCY)[:N//2]

    # Plot the normalized (2/n * abs(y)) FFT
    ax2.plot(xf, 2.0/N * np.abs(yf[:N//2]))
    ax2.grid()
    ax2.set_xlabel("Frequency (Hz)")
    ax2.set_ylabel("Amplitude (V)")
    ax2.set_title("FFT of the sound wave")

    ax4.plot(xf, 2.0/N * np.abs(yf_centered[:N//2]))
    ax4.grid()
    ax4.set_xlabel("Frequency (Hz)")
    ax4.set_ylabel("Amplitude")
    ax4.set_title("FFT of the compensated sound wave")

    plt.show()

    print("End!")


if __name__ == "__main__":
    main()
