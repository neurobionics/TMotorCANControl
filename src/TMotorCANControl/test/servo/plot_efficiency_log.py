from matplotlib import pyplot as plt
import csv
import numpy as np
import pandas as pd

name = "Measuring_efficiency_0_A_antagonist1667907781.924308.csv"
data = pd.read_csv(name)
print([k for k in data])

torque_adc = -1*np.array(data['Futek Torque (Nm)'])
time = np.array(data['loop time (s)'])
current = np.array(data['i_q'])
v_bus = np.array(data['v_bus'])
i_bus = np.array(data['i_bus'])
i_q = np.array(data['i_q'])
v_q = np.array(data['v_q'])
speed = np.array(data['velocity (Rad/S)'])
power_out = torque_adc*speed
power_in = v_bus*i_bus
eff = np.zeros_like(power_out)
for i in range(power_out.shape[0]):
    tmp = 100*power_out[i]/power_in[i]
    eff[i] = tmp if (not np.isnan(tmp) and not np.isinf(tmp)) else -100

print(np.mean(eff))
print(np.max(eff))

# print(data)
print("Average Torque: {}Nm".format(torque_adc.mean()))

# plt.plot(time_array, torque_array)
# plt.savefig("torque_plot_{}_A.png".format(iq_des))

# plt.plot(time,torque_adc,label="τ_adc (max: " + str(round(torque_adc.max(),2)) + "Nm)" + " (min: " + str(round(torque_adc.min(),2)) + "Nm)")
# plt.plot(time,current,label="iq_motor (max: " + str(round(current.max(),2)) + "A)" + " (min: " + str(round(current.min(),2)) + "A)")
# # plt.plot(np.array(time),curr_lim,label="lim")
# # plt.plot(np.array(time),current_motor,label="τ_motor (max: " + str(round(current_motor.max(),2)) + "Nm)" + " (min: " + str(round(current_motor.min(),2)) + "Nm)")

# plt.title('Torque and Current vs Time')
# plt.ylabel('Torque [Nm] or Current [A]')
# plt.xlabel('Time [s]')
# plt.grid(True)
# plt.legend()

# plt.show()
# plt.savefig("{}_torque_plot.png".format(name))
# plt.clf()



plt.rcParams.update({'font.size': 14})
fig, axs = plt.subplots(1,2)
# I vs V
plt1 = axs[0].scatter(v_q, i_q, s = 200, c=eff, vmin = -100, vmax = 100, cmap = 'viridis')
# axs[0].scatter(vol_l_mean, i_l_mean, s = 200, c=eff_r, vmin = -100, vmax = 100, cmap = 'viridis')
# axs[0].set_xlim(left = -10)
# axs[0].set_ylim(bottom = 0)
axs[0].set_ylabel('$Current$ $(A)$')
axs[0].set_xlabel('$Voltage$ $(V)$')
# T vs Vel
axs[1].scatter(speed, torque_adc, s = 200, c=eff, vmin = -100, vmax = 100, cmap = 'viridis')
# axs[1].scatter(vel_r_mean, torque_mean, s = 200, c=eff_r, vmin = -100, vmax = 100, cmap = 'viridis')
axs[1].set_ylim(bottom = 0)
axs[1].set_ylabel('$Torque$ $(N-m)$')
axs[1].set_xlabel('$Velocity$ $(rad/s)$')

fig.colorbar(plt1, ax = axs, orientation = 'horizontal')
fig.suptitle('Efficiency')

plt.savefig("{}_efficiency_plot.png".format(name), dpi = 600)
plt.show()


