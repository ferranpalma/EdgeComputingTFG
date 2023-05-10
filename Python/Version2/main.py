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
N = BUFFER_SIZE
FREQUENCY = 40000
SAMPLE_FREQUENCY = 1/FREQUENCY

# Function to open the serial port to comunicate via UART. Always open
# Returns the instance of the serial port
def start_port():
    port = serial.Serial(
        port="/dev/tty.usbmodem1303",
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
    while index < BUFFER_SIZE:
        unpacked_data = struct.unpack('>B', port.read(size=1))[0]
        buffer.append(unpacked_data)
        index += 1

    index = 0

    print("Reassembling data...")

    # Reassemble the data (bit operations)
    while index < BUFFER_SIZE:
        reassembled_data = ((buffer[index] << 8) | buffer[index+1])
        mic_data.append(reassembled_data)
        index += 2

    # DFT sample frequencies array
    xf = libfft.fftfreq(N,SAMPLE_FREQUENCY)[:N//2]

    # Plot the data

    print("Data reassembled. Plotting...")

    fig, (ax1) = plt.subplots(ncols=1, nrows=1)    

    ax1.plot(xf, mic_data)
    ax1.grid()
    ax1.set_xlabel("Frequency (Hz)")
    ax1.set_ylabel("Amplitude")
    ax1.set_title("FFT of the sound wave")

    plt.show()

    print("End!")


if __name__ == "__main__":
    main()
