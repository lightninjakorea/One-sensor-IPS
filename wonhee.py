import numpy as np
import os
import PIL.Image as pilimg
import numpy as np
from afasf import pred
import math
import matplotlib.pyplot as plt
from pf import *
import serial
from multiprocessing import Queue
import threading
import collections
from scipy.optimize import curve_fit


class MovingAverageFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.data = []

    def filter(self, new_value):
        # 새 데이터를 리스트에 추가
        self.data.append(new_value)
        
        # 윈도우 사이즈보다 데이터가 많으면 가장 오래된 데이터 제거
        if len(self.data) > self.window_size:
            self.data.pop(0)
        
        # 데이터의 평균 계산
        average = sum(self.data) / len(self.data)
        return average

ser = serial.Serial('COM10', 115200, timeout = 3)
ser2 = serial.Serial('COM19', 115200, timeout = 3)

queue = Queue(1000)

def serial_read(s):
    while 1:
        line = s.readline()
        queue.put(line)
        
with open("experiment.txt", "a") as file:
    file.truncate(0)
with open("theoretical.txt", "a") as file:
    file.truncate(0)
with open("test.txt", "a") as file:
    file.truncate(0)

def find_a(x_list, y_list):
    n = len(x_list)
    sx, sx2, sy, sxy = sum(x_list), 0, sum(y_list), 0
    for i in range(n):
        sx2 += x_list[i] ** 2
        sxy += x_list[i] * y_list[i]
    if (n * sxy > sx * sy) ^ (n * sx2 > sx):
        return True  # True 일 때 증가하는 것임
    else:
        return False

def read_data(file_path):
    time = []
    x = []
    y = []
    with open(file_path, 'r') as file:
        lines = file.readlines()
        for line in lines:
            t, x_val, y_val = map(float, line.split())
            time.append(t)
            x.append(x_val)
            y.append(y_val)
    return time, x, y

def update_plot(a):
    x_data1.append(a[0])
    y_data1.append(a[1])
    x_data1.append(x_data1[-1] + 1)
    y_data1.append(a[1])
    with open("experiment.txt", "a") as file:
        file.write(f"{time} {x_data1[-1]} {y_data1[-1]}\n")
        x_data1.append(a[0])
    


packet = bytearray()
packet.append(0x6D)
packet.append(0x65)
packet.append(0x6E)
packet.append(0x75)
packet.append(0x0D)
packet.append(0x0A)
ser.write(packet)
ser.readlines()
ser.write(b'8')
ser.readlines()
ser.write(b'10')
ser.readlines()
ser.write(b'1')
ser.readline()
ser.readline()
ser.readline()
ser.readline()
ser.readline()
ser.readline()

threadA = threading.Thread(target=serial_read, args=(ser,),).start()
threadT = threading.Thread(target=serial_read, args=(ser2,),).start()

N = 500
i = 0
time = 0
dt = 0.001
simend = 10000

x, y = [], []

x_data1 = []
y_data1 = []
x_data2 = []
y_data2 = []
x_datamid = []
y_datamid = []
fig, ax = plt.subplots()
shift_check = 0


distanceData = [0]
distanceData2 = [0]
positionData = [0]
enable = True
tmp_check = 0
tmp_distance = 0
tmp_postion = 0

num_particles = 10000
x_range = [0, 30]
y_range = [0, 30]
process_variance = 5.0
measurement_variance = 5.0
tmp_a=0, 0
a=0, 0
pf = ParticleFilter(num_particles, x_range, y_range, process_variance, measurement_variance)
filter = MovingAverageFilter(window_size=20)
enable_first = 1

def custom_function(x, a, b, c):
    return np.sqrt(a + b * np.sin(x + c))

while True:
    data = queue.get(True, 1)
    string_data = data.decode()
    if string_data[0] == "D":
        for i in range(len(string_data)):
            if string_data[i] == "T":
                start = i + 4
            if string_data[i] == "C":
                end = i
        if int(string_data[start:end]) != -10000:
            if  0 <=int(string_data[start:end])/100 <= 20 and distanceData2[-1]!=int(string_data[start:end])/100:
                filtered_data=filter.filter(int(string_data[start:end])/100)
                #print(filtered_data)
                distanceData2.append(filtered_data)
                with open("test.txt", "a") as file:
                    file.write(f"{time} {distanceData2[-1]} {positionData[-1]}\n")
    elif string_data[-1:-5] != '\r\n':
        positionData.append(((int(string_data) - 2024)/4096) * 2 * np.pi)
        distanceData.append(distanceData2[-1])
        #print(positionData[-1], distanceData[-1])
    time += dt
    restart = 5
    i = 0
    enable = True
    if len(distanceData2) > 100:
        if 0 < positionData[-1] % (2 * np.pi) < 0.1 and enable == True:
            enable = False
            time_exp, dis_exp, po_exp = read_data('test.txt')
            x_data = np.array(po_exp[-100:])
            y_data = np.array(dis_exp[-100:])
            initial_guess = [0.1, 0.1, 0.1]
            popt, pcov = curve_fit(custom_function, x_data, y_data, p0=initial_guess)
            angle = ((-1 * popt[2]) + np.pi/2)
            distance = (math.sqrt(popt[0] + popt[1]) + math.sqrt(popt[0] - popt[1]))/2
            print("a")
            print(distance, angle)
            x = distance * math.sin(angle)
            y = distance * math.cos(angle)
            with open("experiment.txt", "a") as file:
                file.write(f"{time} {x} {y}\n")
            i += 1
            print(x, y)
    if positionData[-1] >= np.pi:
            enable = True