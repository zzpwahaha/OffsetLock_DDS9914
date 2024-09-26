import serial
import time

# Replace 'COM3' with your COM port (e.g., '/dev/ttyUSB0' on Linux)
com_port = 'COM11'
baud_rate = 115200  # Set to your device's baud rate

try:
    # Initialize serial connection
    ser = serial.Serial(com_port, baud_rate, timeout=1)
    print(f'Connected to {com_port}')

    # Optional: Give some time for the device to initialize
    time.sleep(2)

    # Example: Send data to the device
    message = 'Hello, Device!'
    ser.write(message.encode('utf-8'))
    print(f'Sent: {message}')
    time.sleep(10)

    # Optional: Read response from the device
    response = ser.readline().decode('utf-8').strip()
    print(f'Received: {response}')

except serial.SerialException as e:
    print(f'Error: {e}')
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print('Serial port closed')
