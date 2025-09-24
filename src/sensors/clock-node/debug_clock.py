import serial
import time

port = "/dev/ttyACM2"
baud = 115200

with serial.Serial(port, baudrate=baud, timeout=2) as ser: 
	print("waiting for clock to wake up...")
	time.sleep(5)
	
	cmd = "rst 300000 060000\n"
	print(f"sending: {cmd.strip()}")
	ser.write(cmd.encode("utf-8"))
	time.sleep(1.5)
	
	response = ser.readline().decode().strip()
	print(f"clock responded: {response}")
	
	

