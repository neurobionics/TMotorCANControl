from matplotlib import pyplot as plt
import csv
import numpy as np
from scipy.signal import butter, filtfilt, lfilter
from scipy.fft import fft,fftfreq


def butter_lowpass_filter(data, cutoff, fs, order):
    nyq=0.5*fs
    normal_cutoff = cutoff / nyq
    # Get the filter coefficients
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = lfilter(b, a, data)
    return y

time = []
torque_command = []
torque_adc = []
torque_motor = []
current_motor = []
speed_motor = []

with open("src/TMotorCANControl/test/saved_logs/system_id_test_motor_drive_2/log_adc_and_motor_3V.csv",'r') as fd:
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

torque_adc = [-1*τ for τ in torque_adc]

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
ϵ = 0.1
A = np.hstack((np.ones_like(t), gr*kt*i, -gr*np.abs(i)*i, -np.sign(v)*(np.abs(v)/(ϵ + np.abs(v)) ), -np.abs(i)*np.sign(v)*(np.abs(v)/(ϵ + np.abs(v)) )))

print(A.shape)

a_hat = np.linalg.inv(A.T@A)@A.T@τ_adc
print(a_hat)

τ_approx = a_hat[0] + a_hat[1]*gr*kt*i - a_hat[2]*gr*np.abs(i)*i - a_hat[3]*np.sign(v)*(np.abs(v)/(ϵ + np.abs(v)) ) - a_hat[4]*np.abs(i)*np.sign(v)*(np.abs(v)/(ϵ + np.abs(v)) )

error = np.abs((τ_approx - τ_adc)/τ_adc)
rmse = np.sqrt(np.mean(τ_approx - τ_adc)**2)
nonlinear_torque_const = ((kt-a_hat[2]*np.abs(i)/a_hat[1]))
print("nonlinear torque constant range: ")
print(np.max(nonlinear_torque_const))
print(np.min(nonlinear_torque_const))
print("Coulomb friction: ")
print(a_hat[3])
print("Gear Friction")
print(np.max(a_hat[4]*np.abs(i)/(gr*kt*i)))
print("average error")
print(np.mean(error))
print("error less than 0.39")
print(error[error<0.39].shape[0]/error.shape[0])
print("RMSE: ")
print(rmse)

plt.plot(time,torque_motor,label="τ_motor")
plt.plot(time,torque_adc,label="τ_adc")
plt.plot(time,τ_approx,label="τ_approx")
plt.title('Torque vs Time')
plt.ylabel('Torque [Nm]')
plt.xlabel('Time [s]')
plt.legend()
plt.show()
plt.savefig('src/TMotorCANControl/plots/ADC_Motor_Torque.png')
plt.clf()

# computing dynamics?
# I = 9.0 # kg/m^2

# t = 0.0
# Δt = 0.01
# i =1.0
# torques = []
# accels = []
# speeds = []
# times = []
# ω_next = 0.0
# α_old = 0.0
# while t < 1.0:
#     τ_next = a_hat[0] + a_hat[1]*gr*kt*i - a_hat[2]*gr*np.abs(i)*i - a_hat[3]*np.sign(ω_next)*(np.abs(ω_next)/(ϵ + np.abs(ω_next)) ) - a_hat[4]*np.abs(i)*np.sign(ω_next)*(np.abs(ω_next)/(ϵ + np.abs(ω_next)) )
#     α_next = τ_next/I
#     ω_next = ω_next + (α_next-α_old)*Δt
#     α_old = α_next
#     if t < 0.01:
#         print(α_next)
#         print(τ_next)
#         print(ω_next)
#     torques.append(τ_next)
#     accels.append(α_next)
#     speeds.append(ω_next)
#     times.append(t)
#     t += Δt

# plt.plot(times,torques,label="τ")
# # plt.plot(time,torque_adc,label="τ_adc")
# # plt.plot(time,τ_approx,label="τ_approx")
# plt.title('Torque vs Time')
# plt.ylabel('Torque [Nm]')
# plt.xlabel('Time [s]')
# plt.legend()
# plt.show()
# plt.savefig('src/TMotorCANControl/plots/ADC_Motor_Torque.png')
# plt.clf()

