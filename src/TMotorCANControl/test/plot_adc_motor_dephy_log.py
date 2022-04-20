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
log_dir="torque_vary_with_angle/synchronized_trial_3/"
name="test-opposing-motor"


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
torque_adc_adjusted = -np.array(torque_adc)
torque_motor = np.array(torque_motor)
current_motor = np.array(current_motor)
time = np.array(time)
threshold1 = 3
threshold2 = 50

torque_adc_adjusted = torque_adc_adjusted[time>threshold1]
# torque_adc_adjusted = torque_adc_adjusted[time<threshold2]
time = time[time>threshold1]
# time = time[time<threshold2]

time_dephy = np.array(time_dephy) + threshold1

num_pole_pairs = 21
tmotor_gear_ratio = 9
gearbox_gear_ratio = 50
angle_dephy = ((np.array(angle_dephy)/gearbox_gear_ratio)*tmotor_gear_ratio*num_pole_pairs)

angle_dephy_func = interpolate.interp1d(time_dephy,angle_dephy, kind='linear',fill_value="extrapolate")
angle_dephy_adjusted = angle_dephy_func(time)


order = 6
fs = 1/0.01       # sample rate, Hz
cutoff = 10.0  # desired cutoff frequency of the filter, Hz

torque_adc_filtered = butter_lowpass_filter(torque_adc_adjusted, cutoff, fs, order).reshape(-1,)

torque_flucuation_magnitude = (torque_adc_adjusted[time < 30].max() - torque_adc_adjusted[time < 30].min())/np.average(torque_adc_filtered)
print("Torque fluctuation: +-" + str(torque_flucuation_magnitude) + "Nm")

print("Average τ_adc: " + str(np.average(torque_adc_filtered)))
print("Std Dev τ_adc: " + str(np.std(torque_adc_filtered)))
print("Max τ_adc: " + str(torque_adc_filtered.max()))

print("Average τ_motor: " + str(np.average(torque_motor)))
print("Std Dev τ_motor: " + str(np.std(torque_motor)))
print("Max τ_motor: " + str(torque_motor.max()))

plt.plot(time,torque_adc_adjusted,label="τ_adc (max: " + str(round(torque_adc_adjusted.max(),2)) + "Nm)" + " (min: " + str(round(torque_adc_adjusted.min(),2)) + "Nm)")

plt.title('Torque vs Time')
plt.ylabel('Torque [Nm]')
plt.xlabel('Time [s]')
plt.grid(True)
plt.legend()
plt.show()

# plt.savefig(test_dir + log_dir + name + ".png")
plt.clf()
num_cycles= 5
plt.plot(angle_dephy_adjusted[angle_dephy_adjusted < 3.14*2*num_cycles],torque_adc_adjusted[angle_dephy_adjusted < 3.14*2*num_cycles],label="(worst torque fluctuation: " + str(round(torque_flucuation_magnitude,2)) + "%)")

plt.title('Torque vs Angle')
plt.ylabel('Torque [Nm]')
plt.xlabel('Phase Angle (5 cycles) [rad]')
plt.grid(True)
plt.legend()
plt.show()

# plt.savefig(test_dir + log_dir + dephy_name + "_combined.png")
# plt.clf()




