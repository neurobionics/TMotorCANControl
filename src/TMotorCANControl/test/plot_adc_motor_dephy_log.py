from cProfile import label
from matplotlib import pyplot as plt
import csv
import numpy as np
from scipy.signal import butter, lfilter, freqz
from scipy import interpolate

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

test_dir= "saved_logs/"
log_dir="torque_vary_with_angle/synchronized_trial_1/"
name="test-opposing-motor-2"


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
        i += 1

time_dephy = []
angle_dephy = []
dephy_name = "log_mitry_dephy"
with open(test_dir + log_dir + dephy_name + ".csv",'r') as fd:
    reader = csv.reader(fd)
    i = 0
    for row in reader:
        if i > 1:
            time_dephy.append(float(row[0]))
            angle_dephy.append(float(row[1]))
        i += 1

torque_motor = np.array(torque_motor)
current_motor = np.array(current_motor)
time = np.array(time)

time_dephy = np.array(time_dephy) + 2.6
angle_dephy = np.array(angle_dephy)*9/50
angle_dephy_func = interpolate.interp1d(time_dephy, angle_dephy, kind='linear',fill_value="extrapolate")
angle_dephy_adjusted = angle_dephy_func(time[time>2.6])

torque_adc_adjusted = -np.array(torque_adc)
# og_torque = [0.091*9*i for i in current_motor]

order = 6
fs = 1/0.01       # sample rate, Hz
cutoff = 10.0  # desired cutoff frequency of the filter, Hz

torque_adc_filtered = butter_lowpass_filter(torque_adc_adjusted, cutoff, fs, order).reshape(-1,)

# torque_adc_filtered = np.array(torque_adc_filtered)


# og_torque = np.array(og_torque)

print("Average τ_adc: " + str(np.average(torque_adc_filtered)))
print("Std Dev τ_adc: " + str(np.std(torque_adc_filtered)))
print("Max τ_adc: " + str(torque_adc_filtered.max()))

print("Average τ_motor: " + str(np.average(torque_motor)))
print("Std Dev τ_motor: " + str(np.std(torque_motor)))
print("Max τ_motor: " + str(torque_motor.max()))

# current_motor = current_motor/1.56

print("Average i_motor: " + str(np.average(current_motor)))
print("Std Dev i_motor: " + str(np.std(current_motor)))
print("Max i_motor: " + str(current_motor.max()))



# plt.subplot(2, 1, 1)
# plt.plot(np.array(time),torque_adc_adjusted.flatten(),label="τ_adc_raw (max: " + str(round(torque_adc_adjusted.max(),2)) + "Nm)")
# plt.plot(time,og_torque,label="τ_motor (max: "+ str(round(og_torque.max(),2)) + "Nm)")
# plt.plot(time,og_torque,label="τ_unadjusted (max: "+ str(round(og_torque.max(),2)) + "Nm)" )
# plt.plot(np.array(time),speed_motor,label="v")
# plt.plot(np.array(time),current_qaxis,label="i_q (max: " + str(round(current_qaxis.max(),2)) + "A)")
# plt.plot(np.array(time),torque_adc_filtered,label="τ_adc (max: " + str(round(torque_adc_filtered.max(),2)) + "Nm)")
# plt.plot(np.array(time),current_adc,label="i_q (max: " + str(round(current_adc.max(),2)) + "A)")

# plt.plot(np.array(time),current_motor,label="i_motor (max: " + str(round(current_motor.max(),2)) + "A)")
# plt.plot(np.array(time),torque_motor,label="τ_q (max: " + str(round(torque_motor.max(),2)) + "Nm)")

plt.plot(np.array(time),torque_adc_adjusted,label="τ_adc (max: " + str(round(torque_adc_adjusted.max(),2)) + "Nm)")

plt.title('Torque vs Time')
plt.ylabel('Torque [Nm]')
plt.xlabel('Time [s]')
plt.grid(True)
plt.legend()

plt.show()

# plt.savefig(test_dir + log_dir + name + "_combined.png")
# plt.clf()

plt.plot(angle_dephy_adjusted,torque_adc_adjusted[time > 2.6],label="τ_adc (max: " + str(round(torque_adc_adjusted.max(),2)) + "Nm)")

plt.title('Torque vs Angle')
plt.ylabel('Torque [Nm]')
plt.xlabel('Motor-side Angle [rad]')
plt.grid(True)
plt.legend()

plt.show()

# plt.savefig(test_dir + log_dir + name + "_combined.png")
# plt.clf()


