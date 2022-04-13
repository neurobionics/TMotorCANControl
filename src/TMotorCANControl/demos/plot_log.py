from matplotlib import pyplot as plt
import csv
import numpy as np

# pre-declare lists for the data
time = []
position = []
velocity = []
acceleration = []
current = []
torque = []

Mposition = []
Mvelocity = []
Macceleration = []
Mtorque = []

motorside = False
save_figures = False

test_dir= "test/"

# use the csv module to parse the file
with open(test_dir + "log.csv",'r') as fd:
    reader = csv.reader(fd)
    i = 0
    for row in reader:
        if i > 1:
            time.append(float(row[0]))
            position.append(float(row[1]))
            velocity.append(float(row[2]))
            acceleration.append(float(row[3]))
            current.append(float(row[4]))
            torque.append(float(row[5]))
            if motorside:
                Mposition.append(float(row[6]))
                Mvelocity.append(float(row[7]))
                Macceleration.append(float(row[8]))
                Mtorque.append(float(row[9]))
        i += 1

# plot the data, and label the figures
plt.plot(time,position,label="θ")
if motorside:
    plt.plot(time,Mposition,label="ϕ")
plt.title('Position vs Time')
plt.ylabel('Position [rad]')
plt.xlabel('Time [s]')
plt.legend()
plt.show()
if save_figures:
    plt.savefig('plots/position.png')
plt.clf()

plt.plot(time,velocity,label="θd")
if motorside:
    plt.plot(time,Mvelocity,label="ϕd")
plt.title('Velocity vs Time')
plt.ylabel('Velocity [rad/s]')
plt.xlabel('Time [s]')
plt.legend()
plt.show()
if save_figures:
    plt.savefig('plots/velocity.png')
plt.clf()

plt.plot(time,acceleration,label="θdd")
if motorside:
    plt.plot(time,Macceleration,label="ϕdd")
plt.title('Acceleration vs Time')
plt.ylabel('Acceleration [rad/s/s]')
plt.xlabel('Time [s]')
plt.legend()
plt.show()
if save_figures:
    plt.savefig('plots/acceleration.png')
plt.clf()

plt.plot(time,current,label="i")
plt.title('Current vs Time')
plt.ylabel('Current [A]')
plt.xlabel('Time [s]')
plt.legend()
plt.show()
if save_figures:
    plt.savefig('plots/current.png')
plt.clf()

plt.plot(time,torque,label="τ")
if motorside:
    plt.plot(time,Mtorque,label="τm")
plt.title('Torque vs Time')
plt.ylabel('Torque [Nm]')
plt.xlabel('Time [s]')
plt.legend()
plt.show()
if save_figures:
    plt.savefig('plots/torque.png')
plt.clf()

# You could also print out some sample statistics
print("Average current: " + str(np.average(current)))
print("Std Dev current: " + str(np.std(current)))
        