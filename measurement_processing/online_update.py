import numpy as np
import matplotlib.pyplot as plt
from matrix_lite import sensors
import time
from numpy import savetxt

def ReadAndAdd(original, filtered, cutoff, sample_time):
	alpha = (cutoff*sample_time)/(1 + cutoff*sample_time)
	reading = sensors.imu.read()

	print("Original:")
	print(original)
	print("Filtered:")
	print(filtered)

	np.insert(arr=original, obj=len(original),values=reading.roll, axis=0)
	np.insert(arr=filtered, obj=len(filtered),values=reading.roll*alpha + filtered[filtered.shape[1] - 2]*(1 - alpha), axis=0)

	return original, filtered

def main():

	############################################################################################################
	#### Initializing variable
	fig, (ax1,ax2) =  plt.subplots(2, 1, figsize=(9, 4.5), sharey=True, sharex=True)
	ax1.set_title('Read signal')
	ax2.set_title('Filtered signal')
	ax1.set_xlabel('Step')
	ax2.set_xlabel('Step')
	ax1.set_ylabel('Value')
	ax2.set_ylabel('Value')
	original = np.zeros((1,1))
	filtered = np.zeros((1,1))
	_time = np.zeros((1,1))

	n_values = 1000
	cutoff = 15
	sample_time = 0.01
	
	start_initial = time.monotonic()

	############################################################################################################
	#### Execution of the measurement and processing

	for i in range(0,n_values):
		start = time.monotonic()
		#print(np.append(_time, start - start_initial))
		_time = np.append(_time, start - start_initial)
		original, filtered = ReadAndAdd(original, filtered, cutoff=cutoff, sample_time=sample_time)
		left = sample_time - (time.monotonic() - start)
		if (left > 0):
			time.sleep(left)
		
	############################################################################################################
	#### Store the data

	print(original.shape[0])
	print(original.shape[1])
	print(filtered.shape[0])
	print(filtered.shape[1])
	print(_time.shape[0])
	print(_time.shape[1])

	axis_0 = np.concatenate((original,filtered,_time),axis=0)
	axis_1 = np.concatenate((original,filtered,_time),axis=1)
	print(_time)

	savetxt('axis_0.csv', axis_0, delimiter=',')
	savetxt('axis_1.csv', axis_1, delimiter=',')

	############################################################################################################
	#### Plot the data

	ax1.plot(_time, original)
	ax2.plot(_time, filtered)

	fig.show()


if __name__ == '__main__':
	main()