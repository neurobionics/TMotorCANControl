from matplotlib import pyplot as plt
import csv
import numpy as np
import pandas as pd


time_array = []
voltage_array = []
torque_array = []
current_array = []
velocity_array = []
temperature_array = []

iq_des = 30
dt = 0.005

name = "torque_test_script_{}_A".format(iq_des)
with open(name + ".csv") as fd:
    reader = csv.reader(fd)
    i = 0
    T=  0
    for row in reader:
        if i >=0:
            time_array.append(T)
            voltage_array.append(float(row[0]))
            torque_array.append(float(row[1]))
            current_array.append(float(row[2]))
            velocity_array.append(float(row[3]))
            temperature_array.append(float(row[4]))
        i += 1
        T += dt
time = np.array(time_array)
torque_adc = np.array(torque_array)*-1
current = np.array(current_array)

print("Average Torque: {}Nm".format(torque_adc.mean()))

# plt.plot(time_array, torque_array)
# plt.savefig("torque_plot_{}_A.png".format(iq_des))

plt.plot(time,torque_adc,label="τ_adc (max: " + str(round(torque_adc.max(),2)) + "Nm)" + " (min: " + str(round(torque_adc.min(),2)) + "Nm)")
plt.plot(time,current,label="iq_motor (max: " + str(round(current.max(),2)) + "A)" + " (min: " + str(round(current.min(),2)) + "A)")
# plt.plot(np.array(time),curr_lim,label="lim")
# plt.plot(np.array(time),current_motor,label="τ_motor (max: " + str(round(current_motor.max(),2)) + "Nm)" + " (min: " + str(round(current_motor.min(),2)) + "Nm)")

plt.title('Torque and Current vs Time')
plt.ylabel('Torque [Nm] or Current [A]')
plt.xlabel('Time [s]')
plt.grid(True)
plt.legend()

plt.show()
plt.savefig("torque_plot_{}_A.png".format(iq_des))
# plt.clf()