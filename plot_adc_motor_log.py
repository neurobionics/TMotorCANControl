from matplotlib import pyplot as plt
import csv
import numpy as np

time = []
torque_command = []
torque_adc = []
torque_motor = []
current_motor = []
speed_motor = []

with open("log_adc_and_motor.csv",'r') as fd:
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

plt.plot(time,torque_adc,label="τ_adc")
plt.plot(time,torque_motor,label="τ_motor")
# plt.plot(time,torque_adc_adjusted,label="τ_adc_adjusted")
plt.title('Torque vs Time')
plt.ylabel('Torque [Nm]')
plt.xlabel('Time [s]')
plt.legend()
plt.show()
plt.savefig('plots/ADC_Motor_Torque.png')
plt.clf()

print("Average τ_adc: " + str(np.average(torque_adc)))
print("Std Dev τ_adc: " + str(np.std(torque_adc)))

print("Average τ_motor: " + str(np.average(torque_motor)))
print("Std Dev τ_motor: " + str(np.std(torque_motor)))
        