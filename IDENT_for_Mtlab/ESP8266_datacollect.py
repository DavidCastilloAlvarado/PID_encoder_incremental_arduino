import serial
import time
import json
import csv
import sys

ser = serial.Serial('COM8', 115200, timeout=0)
with open('DC_motor.csv', 'a', newline='') as csvfile: 
	regwriter = csv.writer(csvfile, delimiter=',', quoting=csv.QUOTE_MINIMAL)
	while True:
		try:
			data1 = ser.readline()
			data = data1.decode('utf8').replace("'",'"')
			#print(data)
			HUD_data = json.loads(data)
			# Obteniendo la lectura 
			input = HUD_data['input']
			output = HUD_data['output']
			print (" input ={} output={} ".format(input,output))
			if input == 0:
				regwriter.writerow([str(input), str(0)])
			else:
				regwriter.writerow([str(input), str(output)])
			#print(data1)
			#time.sleep(0.01)
		except:
			#print("no reads")
			time.sleep(0.1)
			pass
