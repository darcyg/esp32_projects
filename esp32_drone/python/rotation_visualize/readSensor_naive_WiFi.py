#!/usr/bin/env python

from threading import Thread
import socket
import time
import struct
import numpy as np
import pandas as pd


class WifiRead:
    def __init__(self, server_address='192.168.178.68', port=3333, dataNumBytes=2, numParams=1):
        self.server_address = server_address
        self.port = port
        self.dataNumBytes = dataNumBytes
        self.numParams = numParams
        self.rawData = bytearray(numParams * dataNumBytes)
        self.dataType = None
        if dataNumBytes == 2:
            self.dataType = 'h'     # 2 byte integer
        elif dataNumBytes == 4:
            self.dataType = 'f'     # 4 byte float
        self.data = np.zeros(numParams)
        self.isRun = True
        self.isReceiving = False
        self.thread = None
        # self.csvData = []

        print("creating socket")
        # Create a UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Bind the socket to the port
        print(f'starting up on {self.server_address} port {self.port}')
        self.socket.bind((self.server_address, self.port))

    def readSerialStart(self):
        if self.thread == None:
            self.thread = Thread(target=self.backgroundThread)
            self.thread.start()
            # Block till we start receiving values
            while self.isReceiving != True:
                time.sleep(0.1)

    def getData(self):
        return np.array((self.rawData[0], self.rawData[1], self.rawData[2], self.rawData[3]))

    def backgroundThread(self):    # retrieve data
        time.sleep(1.0)  # give some buffer time for retrieving data
        while (self.isRun):
            data, address = self.socket.recvfrom(160)
            data_float = struct.unpack('<40f', data)
            self.rawData = data_float[:4]
            self.isReceiving = True
            #print(self.rawData)

    def close(self):
        self.isRun = False
        self.thread.join()
        self.socket.close()
        print('Disconnected...')
        # df = pd.DataFrame(self.csvData)
        # df.to_csv('/home/rikisenia/Desktop/data.csv')