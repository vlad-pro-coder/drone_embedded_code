import serial

ser = serial.Serial("/dev/serial0", baudrate=9600, timeout=1)

while True:
    line = ser.readline().decode("ascii", errors="replace").strip()
    print(line)
