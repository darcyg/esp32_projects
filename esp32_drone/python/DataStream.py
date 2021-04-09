import matplotlib.pyplot as plt
import numpy as np
from threading import Thread
import socket
import struct
import time

# use ggplot style for more sophisticated visuals
plt.style.use('ggplot')


def live_plotter(y_vec, figure):
    if figure is None:
        # this is the call to matplotlib that allows dynamic plotting
        plt.ion()
        figure = plt.figure(figsize=(13, 6))
        ax_acc = figure.add_subplot(211)
        ax_gyr = figure.add_subplot(212)
        # create a variable for the line so we can later update it
        graph_acc = ax_acc.plot(y_vec[:, :3], alpha=0.8)
        graph_gyr = ax_gyr.plot(y_vec[:, 3:], alpha=0.8)
        cmap = ['r', 'g', 'b']
        for axis in range(3):
            graph_acc[axis].set_c(cmap[axis])
            graph_gyr[axis].set_c(cmap[axis])

        ax_acc.set_title(f'Accelerometer')
        ax_acc.set_ylabel('G')
        ax_acc.set_ylim([-5, 5])
        ax_acc.set_xlim([0, 1000])

        ax_gyr.set_title(f'Gyroscope')
        ax_gyr.set_ylabel('DPS')
        ax_gyr.set_ylim([-250, 250])
        ax_gyr.set_xlim([0, 1000])
        plt.show()

    else:
        ax_acc = figure.axes[0].lines
        ax_gyr = figure.axes[1].lines

        for frame in range(y_vec.shape[0]):
            for axis in range(int(y_vec.shape[1]/2)):
                vec_acc = ax_acc[axis].get_data()
                vec_gyr = ax_gyr[axis].get_data()

                new_x = np.hstack((vec_acc[0], np.array(vec_acc[0][-1]+1)))
                new_acc_y = np.hstack((vec_acc[1], np.array(y_vec[frame, axis])))
                new_gyr_y = np.hstack((vec_gyr[1], np.array(y_vec[frame, axis+3])))

                ax_acc[axis].set_data(np.array([new_x, new_acc_y]))
                ax_gyr[axis].set_data(np.array([new_x, new_gyr_y]))


        dummy = 1

        # adjust limits if new data goes beyond bounds
        x_min, x_max =figure.axes[0].get_xlim()
        if x_max <= new_x[-1]:
            x_min_new = x_min + (new_x[-1] - x_max)
            x_max_new = new_x[-1]
            figure.axes[0].set_xlim([x_min_new, x_max_new])
            figure.axes[1].set_xlim([x_min_new, x_max_new])
    # this pauses the data so the figure/axis can catch up - the amount of pause can be altered above
    plt.pause(0.001)
    # return figure so we can update it again in the next iteration
    return figure


server_address = '192.168.178.68'
port = 3333
print("creating socket")
# Create a UDP socket
socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Bind the socket to the port
print(f'starting up on {server_address} port {port}')
socket.bind((server_address, port))

figure = None

while True:
    data, address = socket.recvfrom(240)
    data_float = struct.unpack('<60f', data)
    y_vec = np.array(data_float).reshape((10, 6))
    figure = live_plotter(y_vec, figure)


