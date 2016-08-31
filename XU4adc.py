import time

def getXU4battery():
	f = open("/sys/devices/12d10000.adc/iio:device0/in_voltage3_raw",'r')
	raw = f.read()
	raw = raw.rstrip()
	batt = 8.2278*(float(raw)/4096)
	f.close()
	return batt


def main():
	while True:
		batt = getXU4battery()
		print batt
		time.sleep(1)

if __name__ == '__main__':
	main()
