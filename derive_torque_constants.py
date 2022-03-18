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

t = np.array(time)
τ_des =np.array(torque_command)
τ_adc = np.array(torque_adc)
τ_motor = np.array(torque_motor)
i = np.array(current_motor)
v = np.array(speed_motor)

# 0.146 Nm per amp motor-side, with gear ratio of 1:9
kt = 0.146*9.0

A = np.vstack((np.ones_like(t),kt*i,-i*i,-np.sign(v),-np.abs(i)*np.sign(v))).T
print(A.shape)
invAtA = np.linalg.inv(A.T@A)
a_hat = invAtA@A.T@τ_adc
print(a_hat)
τ_approx = a_hat[0] + a_hat[1]*kt*i - a_hat[2]*i*i - a_hat[3]*np.sign(v) - a_hat[4]*np.abs(i)*np.sign(v)

nonlinear_torque_const = ((kt-a_hat[2]*np.abs(i)/a_hat[1]))
print(np.max(nonlinear_torque_const))
print(np.min(nonlinear_torque_const))

plt.plot(time,τ_approx,label="τ_approx")
plt.plot(time,torque_motor,label="τ_motor")
plt.plot(time,torque_adc,label="τ_adc")
plt.title('Torque vs Time')
plt.ylabel('Torque [Nm]')
plt.xlabel('Time [s]')
plt.legend()
plt.show()
plt.savefig('plots/ADC_Motor_Torque.png')
plt.clf()

# v = 0.0
# i = 0.0
# torque = -1.0
# ides = (torque - a_hat[0] + (a_hat[3] + a_hat[4]*np.abs(i))*np.sign(v) )/(a_hat[1]*(kt-a_hat[2]*np.abs(i)/a_hat[1]))
# print(ides)
