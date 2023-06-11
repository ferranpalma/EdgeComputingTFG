# Imports
import serial

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
    #Buffer to store alert message
    buffer = []

    #Variable to control the prints
    state = 0

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
    
    
    print("Sync done, waiting for alarm message")
    
    while 1:
        try:    
            data = port.read().decode('utf8', errors='strict')
            if data != '\x00' and state == 0:
                state = 1
            if state == 1:
                buffer.append(data)
            if data == '\x00' and state == 1:
                state = 2
        except UnicodeDecodeError:
            if len(buffer) == 0 :
                port.reset_input_buffer()
            continue
            
        if state == 2:
            message = "".join(buffer[:-1])
            print(message)
            buffer.clear()
            state = 0

# es moore
            


if __name__ == "__main__":
    main()
