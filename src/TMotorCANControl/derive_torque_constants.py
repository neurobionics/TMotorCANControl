from matplotlib import pyplot as plt
import csv
import numpy as np

time = []
torque_command = []
torque_adc = []
torque_motor = []
current_motor = []
speed_motor = []

with open("test/saved_logs/system_id_test_1/log_adc_and_motor.csv",'r') as fd:
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

torque_motor = [-1*τ for τ in torque_motor]

t = np.array(time)
t = t.reshape((t.shape[0],1))
τ_adc = np.array(torque_adc).reshape((t.shape[0],1))
τ_motor = np.array(torque_motor).reshape((t.shape[0],1))
τ_des =np.array(torque_command).reshape((t.shape[0],1))
i = np.array(current_motor).reshape((t.shape[0],1))
v = np.array(speed_motor).reshape((t.shape[0],1))

# 0.146 Nm per amp motor-side, with gear ratio of 1:9
kt = 0.146
gr = 9.0

A = np.hstack((np.ones_like(t), gr*kt*i, -gr*np.abs(i)*i, -np.sign(v), -np.abs(i)*np.sign(v)))
print(A.shape)
invAtA = np.linalg.inv(A.T@A)
a_hat = invAtA@A.T@τ_adc
print(a_hat)

τ_approx = a_hat[0] + a_hat[1]*gr*kt*i - a_hat[2]*gr*np.abs(i)*i - a_hat[3]*np.sign(v) - a_hat[4]*np.abs(i)*np.sign(v)

nonlinear_torque_const = ((kt-a_hat[2]*np.abs(i)/a_hat[1]))
print(np.max(nonlinear_torque_const))
print(np.min(nonlinear_torque_const))


plt.plot(time,torque_motor,label="τ_motor")
plt.plot(time,torque_adc,label="τ_adc")
plt.plot(time,τ_approx,label="τ_approx")
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
