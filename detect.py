# -*- coding: utf-8 -*-

import platform
import numpy as np
import argparse
import cv2
import serial
import time
import sys
from threading import Thread

serial_use = 1

serial_port = None
Read_RX = 0
receiving_exit = 1
threading_Time = 0.01

RX = []


# -----------------------------------------------

# -----------------------------------------------
def TX_data_py2(ser, one_byte):  # one_byte= 0~255

    # ser.write(chr(int(one_byte)))          #python2.7
    ser.write(serial.to_bytes([one_byte]))  # python3


# -----------------------------------------------
def RX_data(ser):
    if ser.inWaiting() > 0:
        result = ser.read(1)
        RX = ord(result)
        return RX
    else:
        return 0


# -----------------------------------------------

# *************************
def Receiving(ser):
    global receiving_exit

    global X_255_point
    global Y_255_point
    global X_Size
    global Y_Size
    global Area, Angle
    global RX
    receiving_exit = 1
    while True:
        if receiving_exit == 0:
            break
        time.sleep(threading_Time)
        while ser.inWaiting() > 0:
            result = ser.read(1)
            if not ord(result) in RX:
                RX.append(ord(result))
            # print("THREAD RX=" + str(RX))

            # -----  remocon 16 Code  Exit ------
            if RX == 16:
                receiving_exit = 0
                break


# **************************************************
# percent 설정
def percent(x):
    global perc
    over = int(x.size * perc / 100)
    return over


# **************************************************
def object():
    global RX
    #flag = 0
    flag = 0
    check = 0
    find = 0
    head = 0
    # 머리 숙이기
    '''TX_data_py2(serial_port, 29)
    while True:
        print("11")
        if 100 in RX:
            RX.remove(100)
            print("ok")
            break'''

    time.sleep(1)

    print("head")
    min_area = 50
    count = 0

    # hsv 필터(블루)
    lower = np.array([90, 50, 20])
    upper = np.array([150, 255, 255])
    width = [180, 480, 640]
    #lower = np.array([30, 50, 20])
    #upper = np.array([90, 255, 255])
    '''TX_data_py2(serial_port, 4)
    while True:
        if 102 in RX:
            RX.remove(102)
            print("ok")
            break'''

    while True:
        ret, image = cap.read()
        if count < 100:
            count += 1
            continue
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        x4, y4 = 0, 0
        mask = cv2.inRange(hsv, lower, upper)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            # epsilon = 0.005 * cv2.arcLength(c, True)
            # approx =cv2.approxPolyDP(c,epsilon,True)
            # size = len(approx)
            # print(size)
            # print(len(c))
            ((X, Y), radius) = cv2.minEnclosingCircle(c)

            Area = cv2.contourArea(c) / min_area
            if Area > 255:
                Area = 255

            if Area > min_area:
                x4, y4, w4, h4 = cv2.boundingRect(c)
                cv2.rectangle(image, (x4, y4), (x4 + w4, y4 + h4), (0, 255, 0), 2)
                # ----------------------------------------

                # ----------------------------------------

                X_Size = int((255.0 / W_View_size) * w4)
                Y_Size = int((255.0 / H_View_size) * h4)
                X_255_point = int((255.0 / W_View_size) * X)
                Y_255_point = int((255.0 / H_View_size) * Y)
            else:
                x4 = 0
                y4 = 0
        else:
            x = 0
            y = 0
            X_255_point = 0
            Y_255_point = 0
            X_Size = 0
            Y_Size = 0
            Area = 0
            Angle = 0

        # record
        # out.write(mask)

        # 255 to 1
        mask = mask / 255

        # cutting
        left_cut = mask[:, 0:160]
        center_cut = mask[:, 160:480]
        right_cut = mask[:, 480:640]
        masksize = 640*360

        left = left_cut.sum()
        center = center_cut.sum()
        right = right_cut.sum()

        left_pos = percent(left_cut)
        center_pos = percent(center_cut)
        right_pos = percent(right_cut)
        mask_pos = percent(mask)

        pos_list = [left, center, right]
        pos_list.sort()
        # pos_list.sort(reverse=True)

        print(x4, y4)
        # TX_data_py2(serial_port, 1)
        # print(cnts)
        # if mask.sum() > mask_pos:
        #    print("over!")

        '''if flag == 0 :
            if mask.sum() > mask_pos:
                print("find!")
                flag = 1
            else:
                TX_data_py2(serial_port, 1)'''

        if flag == 0:
            if x4 > 0:
                print("find!")
                flag = 1
            else:
                TX_data_py2(serial_port, 1)
        if flag == 1:
            if check < 2:
                '''if left > left_pos or center > center_pos or right > right_pos:
                    if pos_list[0] == left:
                        TX_data_py2(serial_port, 1)
                        print("LEFT!!!")
                    elif pos_list[0] == center:
                        TX_data_py2(serial_port, 2)
                        print("CENTER!!!")
                    elif pos_list[0] == right:
                        TX_data_py2(serial_port, 3)
                        print("RIGHT!!!")'''
                if x4+y4 > 0:
                    if x4 < width[0]:
                        TX_data_py2(serial_port, 1)
                        print("LEFT!!!")
                    elif width[0] <= x4 <= width[1] and y4 < 300:
                        TX_data_py2(serial_port, 2)
                        print("CENTER!!!")
                    elif x4 > width[1]:
                        TX_data_py2(serial_port, 3)
                        print("RIGHT!!!")

                else:
                    check += 1
                    width = [220, 420]
                    if check == 1:
                        count = 0
                        while True:
                            # print("trashead")
                            min_area = 10
                            TX_data_py2(serial_port, 31)
                            # print("RX = ", RX)
                            if 101 in RX:
                                RX.remove(101)
                                print("ok")
                                break
                    print("NO!!!")
            else:
                flag = 2
                time.sleep(1)
                TX_data_py2(serial_port, 4)
                while True:
                    if 102 in RX:
                        RX.remove(102)
                        print("ok")
                        break
                # break
        if flag == 2:

            lower = np.array([30, 50, 20])
            upper = np.array([90, 255, 255])
            #####

            # 안전구역찾기
            if find == 0:
                if mask.sum() > mask_pos:
                    print("find!")
                    find = 1
                else:
                    TX_data_py2(serial_port, 8)

            # 범위 확인
            if find == 1:
                print(mask.sum()/masksize)
                # 입력퍼센트 넘어야함
                if left > left_pos or center > center_pos or right > right_pos:
                    # 80퍼 넘어가면
                    if mask.sum()/masksize > 0.95:
                        while True:
                            TX_data_py2(serial_port, 9)
                            if 102 in RX:
                                RX.remove(102)
                                serial_port.flush()
                                print("ok")
                                break
                        break
                    elif mask.sum()/masksize > 0.6:
                        if head == 0:
                            while True:
                                TX_data_py2(serial_port, 31)
                                if 101 in RX:
                                    RX.remove(101)
                                    print("ok")
                                    break
                                head += 1
                                count = 0
                        else:
                            TX_data_py2(serial_port, 10)
                            print("CENTER!!!")

                    elif mask.sum() < mask_pos:
                        TX_data_py2(serial_port, 10)
                        print("CENTER!!!")

                    else:
                        if pos_list[0] == left:
                            TX_data_py2(serial_port, 8)
                            print("LEFT!!!")
                        elif pos_list[0] == center:
                            TX_data_py2(serial_port, 10)
                            print("CENTER!!!")
                        elif pos_list[0] == right:
                            TX_data_py2(serial_port, 7)
                            print("RIGHT!!!")


        cv2.imshow('cut', left_cut)
        cv2.imshow('cut1', center_cut)
        cv2.imshow('cut2', right_cut)
        cv2.imshow('image', image)
        cv2.moveWindow('cut', 50, 0)
        cv2.moveWindow('cut1', 210, 0)
        cv2.moveWindow('cut2', 530, 0)
        cv2.moveWindow('image', 50, 360)
        key = 0xFF & cv2.waitKey(1)


# **************************************************
if __name__ == '__main__':
    global perc
    perc = int(input('퍼센트 입력: '))
    #perc=5
    # -------------------------------------
    print("-------------------------------------")
    print("---- (2020-1-20)  MINIROBOT Corp. ---")
    print("-------------------------------------")

    os_version = platform.platform()
    print(" ---> OS " + os_version)
    python_version = ".".join(map(str, sys.version_info[:3]))
    print(" ---> Python " + python_version)
    opencv_version = cv2.__version__
    print(" ---> OpenCV  " + opencv_version)
    print("-------------------------------------")

    W_View_size = 640  # 320  #640
    H_View_size = int(W_View_size / 1.777)


    fourcc = cv2.VideoWriter_fourcc('X', 'V', 'I', 'D')
    out = cv2.VideoWriter('output.avi', fourcc, 30.0, (640, 360))

    BPS = 4800  # 4800,9600,14400, 19200,28800, 57600, 115200

    # ---------local Serial Port : ttyS0 --------
    # ---------USB Serial Port : ttyAMA0 --------

    serial_port = serial.Serial('/dev/ttyS0', BPS, timeout=0.01)
    serial_port.flush()  # serial cls

    # ---------------------------

    serial_t = Thread(target=Receiving, args=(serial_port,))
    serial_t.daemon = True
    serial_t.start()
    time.sleep(0.1)
    # ---------------------------

    # First -> Start Code Send
    TX_data_py2(serial_port, 250)

    time.sleep(1)
    TX_data_py2(serial_port, 29)

    time.sleep(1)

    cap = cv2.VideoCapture(0)
    cap.set(3, W_View_size)
    cap.set(4, H_View_size)
    ''''# -----  remocon 16 Code  Exit ------
    while receiving_exit == 1:
        time.sleep(0.01)'''

    print("111")
    # while True:
    object()
    print('end')
    # ---------------------------
    # time.sleep(1)
    # print(serial_port.readline())
    # print("Return DATA: "+ str(RX_data(serial_port)))
    # print ("-------------------------------------")

    exit(1)
