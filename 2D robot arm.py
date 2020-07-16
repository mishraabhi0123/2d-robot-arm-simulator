import numpy as np 
import matplotlib.pyplot as plt 
import time ,sys

plt.ion()
kp = 25
dt = 0.01
l1 = l2 = 1
xi = 1.0
yi = 1.0
flag = 1
theta1, theta2 = np.pi/2,np.pi
history = [[],[]]

def onclick(event):
    global xi, yi
    xi,yi = event.xdata, event.ydata
    run(xi,yi)

def onpress(event):
    global history, flag
    if event.key == 'e' or event.key == 'E':
        flag = 0
        plt.close()
        exit(0)
    elif event.key == 'c' or event.key == 'C':
        history = [[],[]]

def inverse_kinematics(x, y):
    try :
        e = 0.00001
        theta2 = np.arccos((x*x + y*y - l1*l1 - l2*l2)/(2*l1*l2))
        theta1 = np.arctan(y/abs(x+e)) + np.arcsin(l2*np.sin(np.pi - theta2)/np.sqrt(x*x + y*y + e))

        if x < 0:
            theta1 = np.pi - (np.arctan(y/abs(x+e)) - np.arcsin(l2*np.sin(np.pi - theta2)/np.sqrt(x*x + y*y + e)))
        
        return 1, theta1, theta2

    except :
        print('Goal Unreachable!')
        return 0, None, None

def run(xi,yi):
    global kp, theta1,theta2, flag
    check, goal_theta1, goal_theta2 = inverse_kinematics(xi,yi)
    if check:
        while flag:
            # print(theta1, theta2)
            theta1 += kp * (goal_theta1 - theta1) * dt
            theta2 += kp * (goal_theta2 - theta2) * dt
            plot(theta1,theta2,xi,yi)
        

def plot(theta1,theta2,xi,yi):
    global l1, l2, dt, history

    shoulder = np.array([0,0])
    elbow = shoulder + l1 * ([np.cos(theta1), np.sin(theta1)])
    wrist = elbow + l2 *([np.sin(np.pi/2 + theta2 - theta1),np.cos(np.pi/2 + theta2 - theta1)])

    history[0].append(wrist[0])
    history[1].append(wrist[1])

    domain_x = np.linspace(-l1-l2, l1+l2, 100)
    domain_y = np.sqrt((l1+l2)**2 - domain_x**2)

    plt.cla()
    plt.plot( domain_x, domain_y, 'r--' )

    # plt.plot( [xi],[yi], 'gx')
    plt.plot( [shoulder[0], elbow[0], wrist[0]], [shoulder[1], elbow[1], wrist[1]], 'k-' )
    plt.plot( [elbow[0], wrist[0]], [elbow[1], wrist[1]], 'ro' )
    plt.plot( [shoulder[0]], [shoulder[1]],'bo' )
    # plt.plot( [wrist[0],xi], [wrist[1],yi], 'g--' )

    plt.plot( history[0], history[1], 'b--' )

    plt.xlabel('2D arm simulator')
    plt.title('left click within red semicircle to change goal location \nPress "c" to clear blue path\nPress "e" to exit.', size = 10)
    plt.xlim( -l1-l2, l1+l2 )
    plt.ylim( (-l1-l2)/4, l1+l2 ) 

    plt.pause(dt)



def main():
    global xi,yi
    fig = plt.figure()
    fig.canvas.mpl_connect('button_press_event',onclick)
    fig.canvas.mpl_connect('key_press_event', onpress)
    run(xi,yi)

if __name__ == '__main__':
    main()