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

test_dir= "test/saved_logs/"
# log_dir="system_id_test_opposing_motor_2/"
log_dir="system_id_torque_compensation_worked/"

with open(test_dir + log_dir + "log_adc_and_motor.csv",'r') as fd:
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
        i += 1

torque_adc_adjusted = [-1*τ for τ in torque_adc]
og_torque = [0.146*9*i for i in current_motor]

torque_adc_adjusted = np.array([torque_adc_adjusted])
order = 6
fs = 1/0.01       # sample rate, Hz
cutoff = 10.0  # desired cutoff frequency of the filter, Hz

torque_adc_filtered = butter_lowpass_filter(torque_adc_adjusted, cutoff, fs, order).reshape(-1,)

# plt.subplot(2, 1, 1)
plt.plot(np.array(time),torque_adc_filtered,label="τ_adc")
plt.plot(time,torque_motor,label="τ_motor")
plt.plot(time,og_torque,label="τ_unadjusted")
plt.plot(np.array(time),speed_motor,label="v")
plt.plot(np.array(time),current_motor,label="i_q")
plt.title('Torque vs Time')
plt.ylabel('Torque [Nm]')
plt.xlabel('Time [s]')
plt.legend()

# plt.subplot(2, 1, 2)
# plt.plot(np.array(time),speed_motor,label="v")
# plt.title('Velocity vs Time')
# plt.ylabel('Velocity  [rad/s]')
# plt.xlabel('Time [s]')
# plt.legend()


# plt.subplot(2, 1, 2)
# plt.plot(np.array(time),current_motor,label="i_q")
# plt.title('Current vs Time')
# plt.ylabel('Current  [A]')
# plt.xlabel('Time [s]')
# plt.legend()

plt.show()
# plt.savefig(test_dir + log_dir + "torque_vs_time.png")
plt.clf()

print("Average τ_adc: " + str(np.average(torque_adc)))
print("Std Dev τ_adc: " + str(np.std(torque_adc)))

print("Average τ_motor: " + str(np.average(torque_motor)))
print("Std Dev τ_motor: " + str(np.std(torque_motor)))

