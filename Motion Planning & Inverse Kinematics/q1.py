import numpy as np
import matplotlib.pyplot as plt
import math

T3 = 7.5
T5 = 9.375

theta_s = np.array([math.pi, 0])
theta_e = np.array([2*math.pi, (5*math.pi)/4])

traj = []
velo = []
acc = []
xvalues = []

for i in range(1000):
    theta = theta_e[1]-theta_s[1]
    j = i * (T3 / 1000)
    t = theta_s[1] + (((3*j**2)/(T3**2)) - ((2*j**3)/(T3**3)))*(theta)
    v = ((((6*j)/T3**2)) - ((6*(j**2))/(T3**3)))*(theta)
    a = ((6/(T3**2))) - ((12*j)/(T3**3)) * (theta)

    xvalues += [j]
    traj += [t]
    velo += [v]
    acc += [a]

plt.plot(xvalues, traj, color='r')
plt.plot(xvalues, velo, color='g')
plt.plot(xvalues, acc, color='b')
plt.title("Question 1.2(a) Third-Order Polynomial Scaling")
plt.ylabel('s')
plt.xlabel('t')
plt.show()

traj2 = []
velo2 = []
acc2 = []
xvalues2 = []

for i in range(1000):
    theta = theta_e[1]-theta_s[1]
    j = i * (T5 / 1000)
    t = ((10*(j**3))/(T5**3)) - ((15*(j**4))/(T5**4)) + ((6*(j**5))/(T5**5))
    v = ((30*(j**2))/(T5**3)) - ((60*(j**3))/(T5**4)) + ((30*(j**4))/(T5**5))
    a = ((60*(j))/(T5**3)) - ((180*(j**2))/(T5**4)) + ((120*(j**3))/(T5**5))

    xvalues2 += [j]
    traj2 += [t]
    velo2 += [v]
    acc2 += [a]

plt.figure()
plt.plot(xvalues2, traj2, color='r')
plt.plot(xvalues2, velo2, color='g')
plt.plot(xvalues2, acc2, color='b')
plt.title("Question 1.2(b) Fifth-Order Polynomial Scaling")
plt.ylabel('s')
plt.xlabel('t')
plt.show()
