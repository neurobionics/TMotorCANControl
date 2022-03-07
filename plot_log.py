from matplotlib import pyplot as plt
import csv
import numpy as np

position = []
velocity = []
acceleration = []
current = []
time = []
with open("test/log.csv",'r') as fd:
    reader = csv.reader(fd)
    i = 0
    for row in reader:
        if i > 1:
            time.append(float(row[0]))
            position.append(float(row[1]))
            velocity.append(float(row[2]))
            acceleration.append(float(row[3]))
            current.append(float(row[4]))
        i += 1

plt.plot(time,position)
plt.title('Position vs Time')
plt.ylabel('Position [rad]')
plt.xlabel('Time [s]')
plt.show()
plt.savefig('plots/position.png')
plt.clf()

plt.plot(time,velocity)
plt.title('Velocity vs Time')
plt.ylabel('Velocity [rad/s]')
plt.xlabel('Time [s]')
plt.show()
plt.savefig('plots/velocity.png')
plt.clf()

plt.plot(time,acceleration)
plt.title('Acceleration vs Time')
plt.ylabel('Acceleration [rad/s/s]')
plt.xlabel('Time [s]')
plt.show()
plt.savefig('plots/acceleration.png')
plt.clf()

plt.plot(time,current)
plt.title('Current vs Time')
plt.ylabel('Current [A]')
plt.xlabel('Time [s]')
plt.show()
plt.savefig('plots/current.png')
plt.clf()


print("Average current: " + str(np.average(current)))
print("Std Dev current: " + str(np.std(current)))

print("Average position: " + str(np.average(position)))
print("Std Dev position: " + str(np.std(position)))

print("Average velocity: " + str(np.average(velocity)))
print("Std Dev velocity: " + str(np.std(velocity)))

print("Average acceleration: " + str(np.average(acceleration)))
print("Std Dev acceleration: " + str(np.std(acceleration)))


        