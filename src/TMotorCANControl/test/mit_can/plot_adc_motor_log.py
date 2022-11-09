from cProfile import label
from matplotlib import pyplot as plt
import csv
import numpy as np
from scipy.signal import butter, lfilter, freqz

def butter_lowpass(cutoff, fs, order=5):
    return butter(order, cutoff, fs=fs, btype='low', analog=False)

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

time = []
torque_command = []
torque_adc = []
torque_motor = []
current_motor = []
speed_motor = []
curr_lim = []

test_dir= "saved_logs/"
log_dir="sys_ID_final/"
name="trial9-compensation"

with open(test_dir + log_dir + name + ".csv",'r') as fd:
    reader = csv.reader(fd)
    i = 0
    for row in reader:
        if i > 1:
            time.append(float(row[0]))
            torque_command.append(float(row[1]))
            torque_adc.append(float(row[2]))
            torque_motor.append(float(row[3]))
            current_motor.append(float(row[4]))
            speed_motor.append(float(row[5]))
            curr_lim.append(float(row[6]))
        i += 1


torque_motor = np.array(torque_motor)
current_motor = np.array(current_motor)
torque_adc_adjusted = -np.array(torque_adc)

order = 6
fs = 1/0.01       # sample rate, Hz
cutoff = 10.0  # desired cutoff frequency of the filter, Hz
torque_adc_filtered = butter_lowpass_filter(torque_adc_adjusted, cutoff, fs, order).reshape(-1,)

print("Torque adjustment factor: " + str(np.median(torque_adc_filtered/torque_motor)))

plt.plot(np.array(time),torque_adc_adjusted,label="τ_adc (max: " + str(round(torque_adc_adjusted.max(),2)) + "Nm)" + " (min: " + str(round(torque_adc_adjusted.min(),2)) + "Nm)")
plt.plot(np.array(time),torque_motor,label="τ_motor (max: " + str(round(torque_motor.max(),2)) + "Nm)" + " (min: " + str(round(torque_motor.min(),2)) + "Nm)")
# plt.plot(np.array(time),curr_lim,label="lim")
# plt.plot(np.array(time),current_motor,label="τ_motor (max: " + str(round(current_motor.max(),2)) + "Nm)" + " (min: " + str(round(current_motor.min(),2)) + "Nm)")

plt.title('Torque vs Time')
plt.ylabel('Torque [Nm]')
plt.xlabel('Time [s]')
plt.grid(True)
plt.legend()

plt.show()

plt.savefig(test_dir + log_dir + name + ".png")
# plt.clf()



print("Average τ_adc: " + str(np.average(torque_adc_filtered)))
print("Std Dev τ_adc: " + str(np.std(torque_adc_filtered)))
print("Max τ_adc: " + str(torque_adc_filtered.max()))

print("Average τ_motor: " + str(np.average(torque_motor)))
print("Std Dev τ_motor: " + str(np.std(torque_motor)))
print("Max τ_motor: " + str(torque_motor.max()))

print("Average i_motor: " + str(np.average(current_motor)))
print("Std Dev i_motor: " + str(np.std(current_motor)))
print("Max i_motor: " + str(current_motor.max()))

