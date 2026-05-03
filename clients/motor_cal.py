import sys
import motor_speed_client as msc
import time
import numpy as np

app = msc.QApplication(sys.argv)
client = msc.MotorSpeedClient()

start_reg = 0x2300
end_reg = 0
num_points = 250
sample_count = 5
pause_s = 0.2

def write_speed(x):
	pwm_value = int(x)
	low = pwm_value & 0xFF
	high = (pwm_value >> 8) & 0xFF
	try:
	# Remote firmware expects single-byte register + single-byte value.
		client.bus.write_byte_data(msc.I2C_ADDR, msc.REG_PWM_LOW, low)
		client.bus.write_byte_data(msc.I2C_ADDR, msc.REG_PWM_HIGH, high)
	except OSError as e:
		print("Error: {0}".format(e))

tries = 3

calfile = "motorcal.txt"

with open(calfile,'w') as f:
	f.write('#Rotation(Hz)\tRegister\n')

for x in np.linspace(start_reg,end_reg,num_points):
	pwm_value=int(x)
	write_speed(pwm_value)
	for _ in range(tries):
		time.sleep(2) # wait for motor to settle
		deltas = []
		for _ in range(sample_count):
			deltas.append(client.read_edge_time_us())
			time.sleep(pause_s)
		delta = np.median(deltas)
		if delta != 0:
			cv = np.std(deltas)/delta
			if (cv < 0.025):
				hz = 1000000./delta 
				print("{0:.2f} {1} {2}".format(hz,pwm_value,cv))
				with open(calfile,'a') as f:
					f.write("{0:.2f}\t{1}\n".format(hz,pwm_value))
				break
		else:
			break

# turn off motor
write_speed(0x2300)

client.bus.close()
