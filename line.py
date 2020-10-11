# -*-coding:utf-8-*-
import cv2
import platform
import numpy as np
import math_point
import traceback
import operator
import sys
import serial
import time
from threading import Thread

# 현재 수행해야 하는 연산
task_step = 1
# 왼쪽방향 오른쪽방향 (True: 왼쪽, False: 오른쪽)
L_R_flag = False
# 다음에는 나간다는 플래그
exit_flag = False

serial_use = 1

serial_port = None
Read_RX = 0
receiving_exit = 1
threading_Time = 0.01

RX = []
TX = []
TX_old = None
delay_num = 0

font = cv2.FONT_HERSHEY_PLAIN
fontScale = 2
# -----------------------------------------------

# -----------------------------------------------
def TX_data_py2(ser, one_byte):  # one_byte= 0~255
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

    receiving_exit = 1
    while True:
        if receiving_exit == 0:
            break
        time.sleep(threading_Time)
        # print("RX = ", RX)
        # 로봇 움직일 때 이거 키면 동작 수행함.
        TX_pop()
        while ser.inWaiting() > 0:
            result = ser.read(1)
            RX_append(ord(result))

            # 제어보드쪽에서 신호 종류에 따라 분기하는 알고리즘을 힘들더라도 만들어야 할듯..
            # --> 제어보드에서 이쪽으로 동작 완료 신호를 보내고 딜레이 좀 줘야 할듯.
            # 무조건 최근 신호 보내게
            # TX_pop()
            # 제어보드에서 라즈베리v4로 특정 신호들은 구분 필요할듯... (동작 종료 신호만으로는 머리 숙이는거 구분 힘듬)
            # print("RX=" + str(RX))

            # -----  remocon 16 Code  Exit ------
            if RX == 16:
                receiving_exit = 0
                break

# 동작 멈춤 : 38
# 앞으로 이동 : 40
# 오른쪽 대각선 신호 : 41
# 왼쪽 대각선 신호 : 42
# --> 회전은 그냥 다 모다서 하나로 ㄱ : 43
# 머리 다 숙이기 완료 신호 : 44
# 머리 초기 상태 : 45
# 정지 신호 : 46
# 위의 신호들 중 확인 필요한 신호는 현재로써는 회전, 머리 관련, 멈춤 신호가 확인 필요

def re_hangul_text(cur_TX):
    if cur_TX == 33:
        print("Receive Data : 앞으로 이동 시작")
    elif cur_TX == 34:
        print("Receive Data : 오른쪽 대각선 이동 시작")
    elif cur_TX == 35:
        print("Receive Data : 왼쪽 대각선 이동 시작")
    elif cur_TX == 36:
        print("Receive Data : 오른쪽 회전 시작")
    elif cur_TX == 37:
        print("Receive Data : 왼쪽 회전 시작")
    elif cur_TX == 38:
        print("Receive Data : 오른쪽 90도 회전 시작")
    elif cur_TX == 39:
        print("Receive Data : 왼쪽 90도 회전 시작")
    elif cur_TX == 40:
        print("Receive Data : 멈춤 시작")
    elif cur_TX == 41:
        print("Receive Data : 머리 숙이기 시작")
    elif cur_TX == 42:
        print("Receive Data : 머리 초기화(올리기) 시작")
    elif cur_TX == 44:
        print("Receive Data : 머리 다 숙임")
    elif cur_TX == 111:
        print("Receive Data : Main")
    elif cur_TX == 222:
        print("Receive Data : Main_2")
    elif cur_TX == 333:
        print("Receive Data : RX_EXIT")
    elif cur_TX == 1:
        print("Receive Data : 머리 1단계")
    elif cur_TX == 2:
        print("Receive Data : 머리 2단계")
    elif cur_TX == 3:
        print("Receive Data : 머리 3단계")
    elif cur_TX == 4:
        print("Receive Data : 머리 4단계")

# 가장 최근 신호를 끝에 놓는 함수 --> 한 신호에 대해 무조건 하나만 존재
def RX_append(data):
    global RX
    re_hangul_text(data)
    # print("Receive Data:", data)
    if not data in RX:
        RX.append(data)
    else:
        RX.remove(data)
        RX.append(data)

# 동작 완료 신호
def get_complete_RX():
    global RX
    if 38 in RX:
        RX.remove(38)
        return True
    return False

# 회전 종료 신호 왔는지
def get_RX_turn():
    global RX
    if 43 in RX:
        RX.remove(43)
        return True
    return False


# 머리 끝까지 숙였다는 신호 왔는지 체크 함수
def get_RX_down_head():
    global RX
    if 44 in RX:
        RX.remove(44)
        return True
    return False


# 머리 다시 전방 앞으로 보는 함수
def get_RX_up_head():
    global RX
    if 45 in RX:
        RX.remove(45)
        return True
    return False


# 멈췄다는 신호 왔는지 체크 함수
def get_RX_stop():
    global RX
    if 46 in RX:
        RX.remove(46)
        return True
    return False


# 가장 최근 신호를 유지하기 위한 함수
def TX_append(data):
    global TX
    if not data in TX:
        TX.append(data)
    else:
        TX.remove(data)
        TX.append(data)

def hangul_text(cur_TX):
    if cur_TX == 33:
        print("Send Data : 앞으로 이동")
    elif cur_TX == 34:
        print("Send Data : 오른쪽 대각선 이동")
    elif cur_TX == 35:
        print("Send Data : 왼쪽 대각선 이동")
    elif cur_TX == 36:
        print("Send Data : 오른쪽 회전")
    elif cur_TX == 37:
        print("Send Data : 왼쪽 회전")
    elif cur_TX == 38:
        print("Send Data : 오른쪽 90도 회전")
    elif cur_TX == 39:
        print("Send Data : 왼쪽 90도 이동")
    elif cur_TX == 40:
        print("Send Data : 정지 신호")
    elif cur_TX == 41:
        print("Send Data : 머리 숙이기")
    elif cur_TX == 42:
        print("Send Data : 머리 초기 상태")

# 제어보드에서 신호가 오면 가장 최근 데이터 보내고 전부 pop --> 데이터 보내는 부분
def TX_pop():
    global TX, TX_old, delay_num
    if len(TX) != 0:
        cur_TX = TX[(len(TX) - 1)]
        # 원래 코드
        if TX_old != cur_TX or (TX_old == cur_TX and delay_num > 15):
            delay_num = 0
            hangul_text(cur_TX)
            # print("Send Data : ", cur_TX)
            TX_data_py2(serial_port, cur_TX)
            TX_old = cur_TX

        delay_num += 1
        
        # 버전 2 --> 문제 많음
        # if TX_old is None:
        #     TX_old = cur_TX
        # 
        # head_flag = False
        # if cur_TX == 41 or cur_TX == 42:
        #     head_flag = True
        # 
        # # 머리만 딜레이
        # if head_flag:
        #     if TX_old == cur_TX and delay_num > 20:
        #         delay_num = 0
        #         hangul_text(cur_TX)
        #         TX_data_py2(serial_port, cur_TX)
        #         TX_old = cur_TX
        # 
        #     delay_num += 1
        # # 몸통 신호는 바로바로
        # else:
        #     hangul_text(cur_TX)
        #     # print("Send Data : ", cur_TX)
        #     TX_data_py2(serial_port, cur_TX)
        #     TX_old = cur_TX

    # 초기화
    TX = []


# 비디오 로드 함수 --> 실제로 할 때는 필요없!
def video_load():
    global cap
    cap = cv2.VideoCapture("video\\output_3.avi")


# ----------------------------------------------------------------------------------------------- 밑에만 확인하면 됨


class line_precondition:
    def __init__(self):
        # 화면 해상도
        self.size_y = 300
        self.size_x = int(self.size_y * 1.777)

        # 기본 정의 변수

        # 1. 라인 색
        self.lower_yellow_line = (20, 95, 95)
        self.upper_yellow_line = (30, 180, 180)

        # 2. 머리 다 숙인 플래그 & 위치 허용 범위 정의
        self.head_flag = False
        self.conor_flag = False
        self.degree_range = 25  # 각도 허용 범위
        self.bottom_range = 100  # 선 위치 허용 범위

        # 3. 라인 탐색 --> 상황 단계에 따라서
        self.step = 1
        # self.sub_step = 1

        # 제어 신호들
        self.Forward = 33  # 앞으로 이동
        self.Right_dia = 34  # 오른쪽 대각선 신호
        self.Left_dia = 35  # 왼쪽 대각선 신호
        self.Right_turn = 36  # 오른쪽 회전
        self.Left_turn = 37  # 왼쪽 회전
        self.Right_90_turn = 38  # 오른쪽 90도 회전
        self.Left_90_turn = 39  # 왼쪽 90도 회전
        self.down_head = 41  # 머리 숙이기
        self.up_head = 42  # 머리 초기 상태
        self.stop = 40  # 정지 신호

        # 같은 직선 판별할 때 재귀함수로 인덱스값 찾는데 걍 귀찮아서 클래스변수로 선언
        self.detection_index = []

        # 행동 하기 전 영상 전처리하여 라인 특정 짓는 함수 --> 행동 명령 함수까지 이어짐
        self.line_pre_cond()

    """ 건들거 없는 부분 시작 """

    # 같은 직선이라 판별된 것끼리 묶어서 반환
    def equal_lines(self, degree_matrix, distance_matrix=None, index=None, flag=True):

        temp_index = np.array(np.where(degree_matrix[index, :] <= 10))
        temp_index = np.squeeze(temp_index, axis=0)

        temp_index_2 = np.array(np.where(distance_matrix[index, temp_index] <= 50))
        temp_index_2 = np.squeeze(temp_index_2, axis=0)

        temp_index = temp_index[temp_index_2]

        if flag and len(temp_index) > 1:
            result_index = temp_index
            for i in temp_index:
                self.detection_index.append(i)
                result_index = np.unique(
                    np.append(result_index, self.equal_lines(degree_matrix, distance_matrix, i, False), axis=0))

            return result_index

        elif not flag:
            # 차집합
            diff_index = list(set(temp_index) - set(self.detection_index))
            if diff_index:
                for i in diff_index:
                    self.detection_index.append(i)
                    temp_index = np.unique(
                        np.append(temp_index, self.equal_lines(degree_matrix, distance_matrix, i, False), axis=0))

            return temp_index

        else:
            return temp_index

    # 같은 시작점과 끝점 끼리 계산하기 위해 시작점 끝점 교환하는 함수
    def line_start_end_pre(self, line_re_arr, temp_index):
        # 선분의 시작점하고 끝점의 위치가 비슷한 것끼리 계산하기 위한 전처리
        point_x, point_y = line_re_arr[temp_index[0]][0], line_re_arr[temp_index[0]][1]
        temp_arr = line_re_arr[temp_index]
        re_temp_arr = temp_arr

        for re_index, re_point in enumerate(temp_arr):
            candidate_1 = abs(re_point[0] - point_x) + abs(re_point[1] - point_y)
            candidate_2 = abs(re_point[2] - point_x) + abs(re_point[3] - point_y)
            if candidate_1 >= candidate_2:
                re_temp_arr[re_index] = re_point[[2, 3, 0, 1]]

        return re_temp_arr

    # 대표선 구하기
    def get_fitline(self, img, f_lines):
        lines = None
        if not len(f_lines) == 1:
            lines = np.squeeze(f_lines)
        else:
            lines = f_lines

        length = len(lines)
        lines = lines.sum(axis=0)
        lines = np.array(lines // length)

        # 수직
        if (lines[2] - lines[0]) == 0:
            result = [lines[0], self.size_x, lines[0], 0]
            m = 10000
            b = 0
        else:
            # 기울기
            m = (lines[3] - lines[1]) / (lines[2] - lines[0])
            b = lines[1] - int(m * lines[0])

            # 기울기가 가로방향이면
            if m < self.size_x / self.size_y:
                result = [0, b, self.size_y - 1, int(m * (self.size_y - 1) + b)]

            # 기울기가 세로방향이면
            else:
                x_end = int((self.size_x - 1 - b) / m)
                result = [x_end, (self.size_x - 1), int(-b / m), 0]

        return result, m, b, lines

    """ 건들거 없는 부분 끝 """

    # 라인 전처리 ( 라인 찾기 main함수부분)
    def line_pre_cond(self):
        # cap: 영상, output, input, 찬호꺼에서 왼쪽 오른쪽 여부, 과제 스텝: ex) 내꺼, 찬호꺼, 형꺼
        global cap, TX, RX, L_R_flag, task_step

        while True:
            ret, frame = cap.read()
            # 이미지 읽기
            if ret:
                frame = cv2.resize(frame, dsize=(self.size_x, self.size_y), interpolation=cv2.INTER_AREA)
                # 노란색 라인 인식
                hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                yellow_mask = cv2.inRange(hsv_frame, self.lower_yellow_line, self.upper_yellow_line)
                yellow_img = cv2.bitwise_and(frame, frame, mask=yellow_mask)

                # 이걸로 한 번에 조지거나, 적응형 쓰레쉬홀드로 조지거나 (속도는 떨어질 수도)
                ret, bit_img = cv2.threshold(yellow_img, 1, 255, cv2.THRESH_BINARY)
                gause_img = cv2.GaussianBlur(bit_img, (5, 5), 0)
                # 전처리 끝이고 median_img로 허프라인 구해서 라인 탐색할 예정
                median_img = cv2.medianBlur(gause_img, 5)
                edge_img = cv2.Canny(median_img, 50, 200, apertureSize=5)

                # 허프변환 1
                #                   검출 이미지,   거리,       각도, 임계값, 최소 선 길이,      최대 선 길이
                lines = cv2.HoughLinesP(edge_img, 2, np.pi / 180, 40, minLineLength=90, maxLineGap=200)
                result_frame = np.copy(frame)

                # flag는 탐색된 라인 존재 여부임 --> 없으면 밑에 계산 필요 X
                flag = True

                # 탐색된 라인 없으면 except 발생 --> 머리 일정 각도 숙이기 실행
                try:
                    for i in lines:
                        break
                except TypeError as e:
                    # print("No Fined")
                    # 머리  숙이기
                    TX_append(self.down_head)
                    flag = False
                    pass

                # 탐색된 라인이 있다면
                if flag:
                    line_arr = np.squeeze(lines)

                    # 허프라인이 하나만 추출 된 경우 차원 추가..
                    if len(line_arr.shape) == 1:
                        line_arr = line_arr[np.newaxis]

                    # 각 선분들의 기울기 리스트 생성 (각도)
                    slope_select = np.array(
                        (np.arctan2(line_arr[:, 1] - line_arr[:, 3], line_arr[:, 0] - line_arr[:, 2]) * 180) / np.pi,
                        dtype=np.int32)

                    # line_re_arr for문 구지 필요한가 점검 필요..
                    line_re_arr = []
                    for index, i in enumerate(line_arr):
                        degree = slope_select[index]

                        # 음수일 경우, 출발점 끝점 치환
                        if degree < 0:
                            temp = i[3]
                            i[3] = i[1]
                            i[1] = temp

                            temp = i[2]
                            i[2] = i[0]
                            i[0] = temp

                        line_re_arr.append(i)

                    line_re_arr = np.array(line_re_arr)

                    slope_degree = np.where(slope_select < 0, slope_select + 180, slope_select)
                    degree_matrix = math_point.degree_diff(slope_degree)
                    distance_matrix, m_result = math_point.line_to_point(line_re_arr)
                    # 같은 라인을 표현한 선을 확인하는 과정에서 이미 탐색된 라인은 패스하기 위한 행렬
                    pass_index = np.array([-1])

                    # 교차점 결정 짓고 라인 특성 추출한 후 행동 결정 함수로 이어지는 부분
                    # 에러 뜨면 추적하려고 try 나중에 완성되고 이 부분에 오류없다면 삭제 필요
                    try:
                        long_line_point = []  # 화면을 꽉 채우는 라인 위에 그려진 긴 선
                        line_point = []  # 실제 계산된 라인위의 선분의 시작과 끝점
                        line_m = []  # 라인의 기울기
                        line_b = []  # 라인의 y절편

                        # 각 허프라인 선들의 갯수만큼 반복문 --> 라인 특정지어서 그리는 부분 --> 확인 완료
                        for i in range(len(degree_matrix)):
                            # 해당 인덱스(선)이 탐색된 적 없다면 해당 선은 다른 라인을 표현하고 있다는 뜻.. --> 탐색 필요
                            if not i in pass_index:
                                # print("Degree type : ", degree_matrix, distance_matrix)
                                # 같은 라인인지 탐색하는 함수
                                temp_index = self.equal_lines(degree_matrix=degree_matrix, distance_matrix=distance_matrix, index=i)

                                # 이미 탐색된 인덱스는 패스하기 위한 리스트 생성
                                pass_index = np.unique(np.append(pass_index, temp_index, axis=0))

                                # 같은 라인이라고 판정된게 한개라면 라인으로 취급 X
                                # --> 그냥 오류일 확률 매우 높음 제대로 된 라인이라면 여러개의 선이 그어짐..
                                if len(temp_index) > 1:
                                    re_temp_arr = self.line_start_end_pre(line_re_arr, temp_index)

                                    # 같은 라인을 표현한 선들을 성분들(시작점과 끝점의 평균)을 이용하여 라인의 특성을 4개로 표현
                                    # 화면 채우는 선, 기울기, y절편, 선분의 시작점과 끝점
                                    fit_line, m, b, line = self.get_fitline(frame, re_temp_arr)

                                    long_line_point.append(fit_line)
                                    line_m.append(m)
                                    line_b.append(b)
                                    line_point.append(line)
                                    # 화면에 그리기 --> 완성되면 지워야 되는 부분
                                    cv2.line(result_frame, (fit_line[0], fit_line[1]), (fit_line[2], fit_line[3]),
                                             (0, 0, 255), 4)
                                    cv2.line(result_frame, (line[0], line[1]), (line[2], line[3]), (0, 0, 255), 2)
                                    # for i in line_re_arr[temp_index]:
                                    #    cv2.line(result_frame, (i[0],i[1]),(i[2],i[3]), color[iter], 1)
                        # -------------------------------------------------------------------------------------------------------------------------------

                        # 교차점 매트릭스
                        cross_matrix = []

                        # 라인들 보고 행동에 영향 끼치는 변수 결정 --> 가로 선 여부, 교차점 매트릭스 여부 및 생성
                        # 교차점 구할려고 하는데 라인이 한개라면 교차점 발생 X --> 탐색 과정 생략
                        if len(long_line_point) > 1:

                            # 가장 수직인 직선 추출
                            max_m = -10
                            max_index = 0

                            for index, i in enumerate(line_m):
                                if abs(i) > max_m:
                                    max_m = abs(i)
                                    max_index = index

                            # 가장 큰 기울기가 최소 0.5 이상인 경우만 교차점 찾음 --> 너무 누워있는거는 잘못 탐색된 경우가 많음..
                            # --> 경우에 따라 생길 수도 있어서 일단 임시임.. --> 교차점때 문제 발생확률 높음 일단 패스
                            if max_m > 0.4:
                                self.vertle_line_flag = False
                                # 가장 기울기 값이 큰 거를 맨 앞으로 이동
                                long_line_point.insert(0, long_line_point.pop(max_index))
                                line_point.insert(0, line_point.pop(max_index))
                                line_m.insert(0, line_m.pop(max_index))
                                line_b.insert(0, line_b.pop(max_index))

                                # 교차점 매트릭스
                                cross_matrix = math_point.cross_point(result_frame, long_line_point, line_m, line_b)

                                # 교차점 확인하기 위해 교차점 그리는 반복문 --> 나중에 삭제 필요
                                for point in cross_matrix:
                                    cv2.circle(result_frame, point, 5, (0, 0, 0), 5)

                                # 마지막에 매개변수로 step값 줘서 미개하게 해야 할 듯
                                # self.line_condition(cross_matrix, long_line_point, line_point, line_m, line_b)

                            # 여러 직선이 나왔는데 가로 직선들로만 구성
                            else:
                                # 복귀 시 --> 근완짱꺼 끝나고 라인 복귀할때... 그때말고는 딱히 있을까......??
                                if self.step == 2:
                                    # 교차점 매트릭스
                                    cross_matrix = math_point.cross_point(result_frame, long_line_point, line_m, line_b)
                                    self.vertle_line_flag = True

                                # elif self.step == 1 or self.step == 3 or self.step == 4:
                                #     # 가로 직선만 보이는데 코너점 나왔었고 동작 종료신호 있으면
                                #     if self.conor_flag and get_RX_head():
                                #         self.corner(line_point, L_R_flag=L_R_flag)
                                # else:
                                #     print("여러 가로 찬호껄로 ㄱㄱ")
                                #     task_step = 0
                                #     break

                        # 그냥 한 직선만 나왔을 때 그냥 직선 갈
                        else:

                            # if len(line_m) == 0:
                            #     continue
                            # 가장 수직인 직선 추출
                            # print("line_m : ", line_m)
                            # print("line_m : ", line_m)
                            max_m = abs(line_m[0])

                            # 가장 큰 기울기가 최소 0.5 이상인 경우만 --> 너무 누워있는거는 잘못 탐색된 경우가 많음..
                            # --> 경우에 따라 생길 수도 있어서 일단 임시임..
                            if max_m > 0.4:
                                self.vertle_line_flag = False
                                cross_matrix = None
                                # 마지막에 매개변수로 step값 줘서 미개하게 해야 할 듯
                                # self.line_condition(cross_matrix, long_line_point, line_point, line_m, line_b)

                            # 가로 직선만 나오면
                            else:
                                self.vertle_line_flag = True
                        
                        # 교차점이 없어서 길이가 0인 경우에도 None으로 대체해서 들어가는걸로
                        if cross_matrix is not None and len(cross_matrix) == 0:
                            cross_matrix = None
                        cv2.circle(result_frame, (int(self.size_x / 2), self.size_y), 5, (255, 255, 255), 5)
                        # 라인 특성 & 스탭에 따라 취해야 하는 행동 결정하는 함수
                        self.line_condition(cross_matrix, long_line_point, line_point, line_m, line_b, result_frame)

                        # task_step(근완땅, 현우땅, 찬호땅)이 1(현우땅)이 아니라면 스땁쁘!
                        if task_step != 1:
                            break

                    except Exception as e:
                        # 에외처리에서 에러 뜬 부분 추척
                        traceback.print_exc()

                # 이미지 읽었다는 if문 안에서 이 밑으로는 완성되면 다 삭제 필요
                # print("아령하세요")
                # cv2.imshow('original_video', frame)
                # v2.moveWindow('original_video', 0, 0)  # Move it
                # cv2.imshow('median_img_video', median_img)
                # cv2.moveWindow('median_img_video', 0, 500)  # Move it
                # cv2.imshow('edge_img_video', edge_img)
                # cv2.moveWindow('edge_img_video', 900, 00)  # Move it
                cv2.imshow('line_result', result_frame)
                cv2.moveWindow('line_result', 0, 0)  # Move it

                k = cv2.waitKey(1)
                # esc 키 종료
                if k == 27:
                    break

            # 읽히는 이미지 없다면 --> 이거는 하드웨어 문제거나 아니면 내가 병신인거지...
            else:
                print("error")
                break

    # 머리 상태 플래그 결정
    def head_flag_dicision(self):
        # True일 때는 업 상태인지만 확인하면 되고
        # False잉 때는 다운 상태인지만 확인하면 됨
        # 그 외는 상태 유지
        if self.head_flag:
            # 머리 초기 상태 완료 신호가 오면
            if get_RX_up_head():
                self.head_flag = False
        else:
            if get_RX_down_head():
                self.head_flag = True
    
    '''일단 돌려보고 만약 턴 할 때의 거리가 좀 부족하면 앞으로 좀 더 가는거를 고정적으로 넣고 턴하는 걸로'''
    # 현재 해야 하는 행동에 따라 함수 호출하는 함수
    # 교차점 매트릭스, 화면 채우는 선 포인트(시작, 끝) 리스트, 원래 선 포인트(시작, 끝) 리스트, 선 기울기 리스트, 선 절편 리스트
    def line_condition(self, cross_matrix, long_line_point, line_point, line_m, line_b, result_frame):
        # 나가기 플래그 & task_step ( 근완땅, 찬호땅, 현우땅) 다 같이 공유
        global exit_flag, task_step, L_R_flag

        # 머리 상태 신호에 따라 결정
        self.head_flag_dicision()

        # 코너점 이동 함수 & 입장시 코너 이동 함수는 코너점이 하나여야함
        # 입장 시 --> 찬호꺼 조지고 난 후라고 생각하고
        # 코너점 돌고나서 세로선 보이면 step 2로 넘어감
        if self.step == 1:

            # 가로선만 보이고 머리 다 숙인게 아니라면
            if not (self.vertle_line_flag and self.head_flag):
                # 코너점 갔다가 다시 세로선 보여서 온거면 다음 스탭으로
                if self.conor_flag:
                    self.conor_flag = False
                    self.step = 2
                    TX_append(self.up_head)
                    print(" 1단계 완료!!!!!")
                    time.sleep(20)
                else:
                    # 가로선만 보이면 --> 머리 숙이기
                    if self.vertle_line_flag:
                        cv2.putText(result_frame, 'Verticle', (70, 70), font, fontScale, (0, 0, 255), 2)
                        TX_append(self.down_head)
                    else:
                        # 기울기 가장 큰 세로선의 성분만 들어감
                        self.walk_line(long_line_point[0], line_point[0], line_m[0], line_b[0], cross_matrix, result_frame)

            # 머리 상태 신호에 따라 결정
            self.head_flag_dicision()
            
            # 이거를 무조건 한번에 턴한다는 마음가짐으로
            # 앞으로 가다가 코너점 보이면 바로 턴 --> 머리 다 숙였는데 가로선만 보이거나 교차점이 안보이면 턴!! --> 다시 세로선 보이면 스텝 2로
            if self.head_flag and (self.vertle_line_flag or cross_matrix is None):
                # 교차점이 없다면
                if cross_matrix is not None or self.vertle_line_flag:
                    # 코너 점 돌았다.
                    self.conor_flag = True
                    # 코너점 돌기 함수
                    self.corner(line_point, L_R_flag=L_R_flag, e_flag=True)

                # 혹시나 가로선이 보일 때는 좀 더 앞으로 가야하는 경우 발생 가능성 있음
                # --> 일단은 그냥 교차점 안보이면 위의 if문에서 바로 턴하도록
                # elif self.vertle_line_flag:
                #     print("조금 앞으로 가기")

        # 직선 가는거
        elif self.step == 2:
            # 일단은 임시로
            exit_flag = False
            # 앞으로 가다가 --> 코너점에서 멈추고 --> 앞으로 보는거까지

            # 가로선만 보이고 머리 다 숙인거 아니라면
            if not (self.head_flag and self.vertle_line_flag):
                self.walk_line(long_line_point[0], line_point[0], line_m[0], line_b[0], cross_matrix)

            # 머리 상태 신호에 따라 결정
            self.head_flag_dicision()

            # 머리 끝까지 숙였는데도 코너점이 안보이면
            # --> 머리 숙이는거는 라인이 안보이거나 코너점을 목표로 찍었을 때만 작동
            # 따라서 머리 끝까지 숙인 것과 코너점의 존재 여부로만 판단가능
            if self.head_flag and (self.vertle_line_flag or cross_matrix is not None):
                # 멈추는 신호
                TX_append(self.stop)

            # 멈춘 신호 오면 전방 주시
            if get_RX_stop():
                # 전방 주시
                TX_append(self.up_head)
                # 찬호한테 넘기는 부분
                task_step = 2
                # 그 다음에 이 클래스로 들어오면 복귀 부터한다고 명시
                self.step = 3

        # 복귀할 선 탐색하는것도 필요함 --> 근완땅쪽에서 노란색 객체 찾는거 까지 해주면 땡큐
        # 다시 복귀
        elif self.step == 3:
            # 라인 복귀하다가 --> 코너점에서 멈추고 --> 코너점 돌고
            # 머리 다 숙였고 교차점이 없는 상태가 아니라면
            if not (self.head_flag and cross_matrix is None):
                # 라인 복귀 함수 호출
                self.return_line(cross_matrix)

            # 머리 상태 신호에 따라 결정
            self.head_flag_dicision()

            # 머리 다 숙였고 교차점 없다면 --> 코너점 돌기
            if self.head_flag and cross_matrix is not None:
                # 코너점 돌기
                self.corner(line_point, L_R_flag=L_R_flag)

                # 직진 or 나가기 분기점
                if not exit_flag:
                    self.step = 2
                else:
                    self.step = 4

        # 수정 필요 --> step 3에서 왔는데 직진 함수부터 해야할 듯
        # 나가기
        elif self.step == 4:
            # 머리 다 숙이고 교차점이 하나도 없는
            # (세로 선은 있어도 가로선이 거의 사라져서 직선탐색 안될때까지 진행 ㄱㄱ 해야지 딱 턴 하기 좋은 위치) 상태가 아니면 직진
            if not (self.head_flag and cross_matrix is None):
                # 직진
                self.walk_line(long_line_point[0], line_point[0], line_m[0], line_b[0], cross_matrix)

            # 앞으로 가다가 멈추는 신호를 줘야하나... 다른 신호들어가면 제어보드에서 알아서 멈추고 그 신호 동작 하도록 ㄱㄱ
            # 앞으로 가다가 멈추는 신호를 줘야하나... 다른 신호들어가면 제어보드에서 알아서 멈추고 그 신호 동작 하도록 ㄱㄱ

            # 머리 상태 신호에 따라 결정
            self.head_flag_dicision()

            # 코너점 돌기 ( 위의 조건 반대 ) 또 if를 해서 탐색하는 이유는
            # 위의 if와 지금 if 사이에 한번더 신호 왔는지 확인해서 최대한 실시간으로 하려고
            if self.head_flag and cross_matrix is not None:
                # 나가기 턴이니까 90도 턴하기 위해 e_flag=True로 설정
                self.corner(line_point, L_R_flag=L_R_flag, e_flag=True)

            # 다 돌았다는 신호 오면
            if get_RX_turn():
                # 전방 주시
                TX_append(self.up_head)
                # 찬호한테 넘기는 부분
                task_step = 2
                # 만약 그 다음에 이 클래스로 들어오면 직진해야함
                self.step = 2

    # ===================================================================================================================================================================
    # 머리 숙이기 여부 판단
    def head_down(self, line_m, line_b):
        # 머리 숙이기 여부 --> 세로 선이 화면의 가운데 부분에도 걸치는지..
        temp_x = int(self.size_y / 2 / line_m) - int(line_b / line_m)
        if not (0 <= temp_x) and (temp_x < self.size_x):
            # 머리 더 숙이기
            TX_append(self.down_head)
            return True
        return False

    # 앞으로 가면서 코너점 추적하기 위한 머리 숙이기
    def cornor_head_down(self, conor_point):
        if conor_point > int(self.size_y / 4 * 3):
            TX_append(self.down_head)

    # 다리 전송 신호 결정
    def TX_data_decision(self, degree, turn_flag=False, e_flag=False, line_flag=False, frame=None):
        # print("decision degree : ", degree)
        # 입장 or 퇴장 시 코너점 돌때
        if e_flag:
            # degree에 따라 90도 회전
            if degree < 0:
                # print("왼쪽 90도 회전")
                TX_append(self.Left_90_turn)
            else:
                # print("오른쪽 90도 회전")
                TX_append(self.Right_90_turn)

        # 이거는 교차점 턴 할때
        elif turn_flag:
            if degree < 0:
                # print('교차점 왼쪽')
                # 왼쪽 회전
                TX_append(self.Left_turn)

            else:
                # 오른쪽 회전
                # print('교차점 오른쪽')
                TX_append(self.Right_turn)
        # 그냥 직진할 때
        # 1. 꼭지점 아닌 그냥 직진할 때
        # 2. 꼭지점 찾은 후 목표지점으로 직진할 때
        # 그냥 가는 걸로 하고 머리 다 숙였는데 꼭지점 좌표가 일정 좌표 안에 있으면 판단하는 건 메인코드에서 ㄱㄱ
        else:
            # 그냥 직진
            if abs(degree) <= self.degree_range:
                cv2.putText(frame, 'degree : {}'.format(degree), (10, 30), font, fontScale, (0, 0, 255), 2)
                cv2.putText(frame, 'Forward', (10, 50), font, fontScale, (0, 0, 255), 2)
                # print("직진")
                TX_append(self.Forward)
            # 오른쪽 대각선 앞으로 가기
            elif degree < 0:
                cv2.putText(frame, 'degree : {}'.format(degree), (10, 30), font, fontScale, (0, 0, 255), 2)
                cv2.putText(frame, 'Right_turn', (10, 50), font, fontScale, (0, 0, 255), 2)
                if line_flag:
                    # print("오른쪽 회전")
                    TX_append(self.Right_turn)
                else:
                    # print("오른쪽 대각선 직진")
                    # TX_append(self.Right_dia) --> 원래 코드
                    TX_append(self.Right_turn)
            # 왼쪽 대각선 앞으로 가기
            else:
                cv2.putText(frame, 'degree : {}'.format(degree), (10, 30), font, fontScale, (0, 0, 255), 2)
                cv2.putText(frame, 'Left_turn', (10, 50), font, fontScale, (0, 0, 255), 2)
                if line_flag:
                    # print("왼쪽 회전")
                    TX_append(self.Left_turn)
                else:
                    # print("왼쪽 대각선 직진")
                    # TX_append(self.Left_dia) --> 원래 코드
                    TX_append(self.Left_turn)


    # 라인 복귀하는 함수
    def return_line(self, cross_matrix):
        # 코너점으로 탐색된 것들 중에 가장 y값이 큰걸로 하면 되지 않을까...

        max_row = 0
        base_degree = 90
        return_point = None

        # 교차점이 있으면 그 중에 가장 밑에 있는 점을 교차점으로 판단
        for point in cross_matrix:
            if point[1] > max_row:
                max_row = point[1]
                return_point = point

        # 교차점이 탐색 되었다면
        if return_point is not None:
            des_point = return_point
            degree = np.array(
                (np.arctan2(des_point[1] - 0, des_point[0] - int((self.size_x) / 2)) * 180) / np.pi,
                dtype=np.int32)

            degree -= base_degree
            self.TX_data_decision(degree, True)

        # 탐색 안되었다면 주위 둘러봐야 함.
        # 복귀 포인트 탐색해야 함
        else:
            # 근완땅이 어느쪽에 서 있는지의 값을 기준으로 탐색 방향 정함
            # 일단 임시로 왼쪽으로
            # 탐색 키 번호
            TX_append(self.Left_turn)

        # 어느정도 가라는 거를 리턴해야함 (현재는 포인트를 반환)
        # return return_point, False

    # 직선 라인에서 걸을 때 끝지점까지 계산해서 올바른 방향으로 가게하는 함수 --> 꼭지점 까지는 잘감
    def walk_line(self, long_line_point, line_point, line_m, line_b, cross_point, frame):
        global exit_flag
        cols = int(self.size_x / 2)

        # 각도 생성 --> line_m은 좌표계에서의 기울기임
        # 각도로 오차 구하는게 좀더 현실적
        degree = np.array(
            (np.arctan2(long_line_point[1] - long_line_point[3],
                        long_line_point[0] - long_line_point[2]) * 180) / np.pi,
            dtype=np.int32)

        # 음수 기울기이면 양수로 바꿔줌
        if degree < 0:
            degree += 180

        base_degree = 90  # 기본 선 각도 --> 딱 수직
        des_point = [cols, 0]  # 기준점 --> 딱 바텀 가운데
        cor_point = None  # 꼭지점
        bottom_point = None  # 선의 아래쪽점
        top_point = None  # 선의 위쪽점

        # 세로 선분의 하단과 상단을 구분지어서 바텀과 탑의 좌표로 분배
        if line_point[1] > line_point[3]:
            bottom_point = line_point[:2]
            top_point = line_point[2:]
        else:
            bottom_point = line_point[2:]
            top_point = line_point[:2]

        # 그냥 직선 갈때 --> 3개 구역 탐색 끝나고 나가기 플래그 안 생기면 그냥 끝까지 가야함
        # 중간 코너점 무시하고 끝 코너점까지 가야함
        if not exit_flag:
            # 꼭지점 중 가장 위에 있는 꼭지점을 기준으로 감 --> 각 이미지 업데이트마다 가장 상단의 꼭지점 기준으로 ㄱㄱ
            if cross_point is not None:

                # 교차점이 있지만 화면 내에 없을 때.. or 비슷한 라인 2개일 때
                # --> 비슷한 라인이 왜 안되냐?: 경기장내 코너점은 무조건 수직임
                if len(cross_point) != 0:
                    max_point = np.array(cross_point)[:, 1].argmin()
                    max_point = cross_point[max_point]

                # 선분이 여러개 나왔지만 꼭지점이 탐색 안되는 경우
                else:
                    # print("cross_point: ", cross_point)
                    max_point = cross_point[0]

                # 코너점 설정 --> 가장 상단의 코너점으로 설정
                if max_point[1] <= top_point[1]:
                    cor_point = max_point

        # 나갈 때 --> 라인 중간에 있는 꼭지점 잡아야 함
        else:
            # 꼭지점 중 밑에 있는 꼭지점을 기준으로 감
            if len(cross_point) != 0:
                exit_cornor_point = np.array(cross_point)[:, 1].argmax()
                exit_cornor_point = cross_point[exit_cornor_point]

                # 코너점 설정
                if (exit_cornor_point[1] >= top_point[1]) and (exit_cornor_point[1] <= bottom_point[1]):
                    cor_point = exit_cornor_point

        # -------------------------------------------------------------> 코너점 설정 끝
        # -------------------------------------------------------------> 코너점 설정 끝

        flag_bottom_range = False
        # flag_degree_range = False

        # 선 가운데 위치 여부
        diff_bottom_point = bottom_point[0] - cols
        if abs(diff_bottom_point) > self.bottom_range:
            flag_bottom_range = True

        # ------------------------------> 여기부터!!
        # 코너점 안 보이면 현재 라인 기울기만 계산해서 잘 가는지 보고
        if cor_point is None:

            if flag_bottom_range:  # and flag_degree_range:
                # 목표 포인트 잡고 기울기 계산
                # int(cols/2) = line_m*x + line_b --> x = int((cols/2-line_b)/line_m)
                des_point[0] = int((self.size_y / 4 * 3 - line_b) / line_m)
                des_point[1] = int(self.size_y / 4 * 3)
                # 목표 지점 각도 계산
                degree = np.array(
                    (np.arctan2(des_point[1] - 0, des_point[0] - cols) * 180) / np.pi,
                    dtype=np.int32)
                # print("꼭지점 X (바텀 차이)-->", degree)
                # 몇도 차이나는지
                diff_degree = degree - base_degree
                cv2.line(frame, (cols, self.size_y), (des_point[0], des_point[1]), (255, 0, 0), 4)

                # print("x_degree : ", degree, bottom_point, cols)
                # 대각선 앞으로 신호 보내기
                self.TX_data_decision(diff_degree, frame=frame)

            # 바텀이 차이가 안나더라도 회전 차이는 발생할 수 있음 대각선 앞으로
            else:
                # print("꼭지점 X -->", degree)
                # just forward
                degree -= base_degree
                # print("just : ", degree)
                # print("ori_x_degree : ", degree, bottom_point, cols)
                cv2.line(frame, (long_line_point[0], long_line_point[1]), (long_line_point[2], long_line_point[3]), (255, 0, 0), 4)
                self.TX_data_decision(degree, line_flag=True, frame=frame)

        # 코너점 검출 되었을 때 --> 이때만 머리 숙이는 코드 넣어도 될 듯...
        else:
            # print("코너점 검출 됨")
            des_point = cor_point
            self.cornor_head_down(cor_point[1])

            degree = np.array(
                (np.arctan2(des_point[1] - 0, des_point[0] - cols) * 180) / np.pi,
                dtype=np.int32)

            cv2.line(frame, (cols, self.size_x), (des_point[0], des_point[1]), (255, 0, 0), 4)
            # degree -= base_degree
            # 몇도 차이나는지

            cv2.putText(frame, 'POINT 0'.format(degree), (10, 70), font, fontScale, (0, 0, 255), 2)
            # print("꼭지점 O -->", degree)
            diff_degree = degree - base_degree
            if flag_bottom_range:
                # 코너점 발견 되어도 그냥 직진하는거니까..
                self.TX_data_decision(diff_degree)
            else:
                self.TX_data_decision(diff_degree, line_flag=True, frame=frame)

   

    # 코너점 돌기 함수
    def corner(self, line_point, L_R_flag, e_flag=False):
        # 입장시 있던 화살표에 따라 상태값 저장 된 변수를 활용 --> 코너 돔
        # 고개는 90도 숙인 상태!!
        # cols로만 비교해도 좌 우 판단 가능
        # print("corner 하는 중")

        de_line = []
        de_point = {}
        
        # # 라인이 탐색 안된다면
        # if len(line_point) == 0:
        #     self.TX_data_decision(degree, True, e_flag)

        # 라인이 한 개라면
        if len(line_point) == 1:
            de_line = line_point

        # 라인이 두 개 이상이면
        else:
            for index, x in enumerate(line_point):
                # 꼭지점에서 먼 점 가져오기
                # 왼쪽 방향이면
                if L_R_flag:
                    if x[0] > x[2]:
                        de_point[index] = x[2]
                    else:
                        de_point[index] = x[0]
                else:
                    if x[0] < x[2]:
                        de_point[index] = x[2]
                    else:
                        de_point[index] = x[0]

            # 가로선 고르고
            if L_R_flag:
                index = min(de_point.items(), key=operator.itemgetter(1))[0]
                de_line = line_point[index]
            else:
                index = max(de_point.items(), key=operator.itemgetter(1))[0]
                de_line = line_point[index]
        # 일단 임시로
        # de_line 형태 : [array[12,55,69,231], dtype=int32]
        # print("코너에서 라인 : ", de_line[0])
        de_line = de_line[0]
        # print("라인 특정 영역 : ", de_line[0], de_line[1], de_line[2], de_line[3])
        degree = None
        if L_R_flag:
            if de_line[0] < de_line[2]:
                degree = np.array(
                    (np.arctan2(de_line[1] - de_line[3], de_line[0] - de_line[2]) * 180) / np.pi,
                    dtype=np.int32)
            else:
                degree = np.array(
                    (np.arctan2(de_line[3] - de_line[1], de_line[2] - de_line[0]) * 180) / np.pi,
                    dtype=np.int32)
        else:
            if de_line[0] < de_line[2]:
                degree = np.array(
                    (np.arctan2(de_line[3] - de_line[1], de_line[2] - de_line[0]) * 180) / np.pi,
                    dtype=np.int32)
            else:
                degree = np.array(
                    (np.arctan2(de_line[1] - de_line[3], de_line[0] - de_line[2]) * 180) / np.pi,
                    dtype=np.int32)
        degree -= 90
        self.TX_data_decision(degree, True, e_flag)

    def enterence(self, line_point, line_m, cross_point, L_R_flag):
        # 입장시 화살표 값 받고 해당 방향 쪽으로 90도 턴
        # 화살표 방향 변수에 따라서 선 판단...
        # 교차점 기준으로 기울기 낮은 선의 양 끝 점에 따라 판단
        # print("enterence 하는 중")
        min_m = line_m[0]
        min_index = 0

        for index, i in enumerate(line_m):
            if abs(i) < min_m:
                min_m = abs(i)
                min_index = index

        min_point = line_point[min_index]

        cross_point = cross_point[0]

        if min_point[0] < cross_point[0]:
            left_point = min_point[:2]
            right_point = min_point[2:]
        else:
            left_point = min_point[2:]
            right_point = min_point[:2]

        if L_R_flag:
            # degree = np.array(
            #     (np.arctan2(left_point[1] - 0, left_point[0] - int(self.size_y / 2)) * 180) / np.pi,
            #     dtype=np.int32)
            degree = np.array(
                (np.arctan2(left_point[1] - cross_point[1], left_point[0] - cross_point[0]) * 180) / np.pi,
                dtype=np.int32)
        else:
            degree = np.array(
                (np.arctan2(right_point[1] - cross_point[1], right_point[0] - cross_point[0]) * 180) / np.pi,
                dtype=np.int32)

        degree -= 90

        self.TX_data_decision(degree, turn_flag=True)
        return degree

    # 나갈떄 코너 돌기 --> 그냥 직선라인에서 한 번에 조질 예정
    # def exit(self, line_point, line_m, cross_point):
    #     # 화살표 변수에 따라
    #     # 직선 그려지는 선분의 중간에 있는 y값을 가지는 코너점을 중간에 있는 나가는 코너점으로 생각
    #     # 해당 코너점 탐색후 이동
    #     # 해당 코너점에서 화살표 변수에 따라 90도 턴
    #     min_m = line_m[0]
    #     min_index = 0
    #     print("exit 하는 중")
    #     for index, i in enumerate(line_m):
    #         if abs(i) < min_m:
    #             min_m = abs(i)
    #             min_index = index
    #
    #     exit_point = line_point[min_index]
    #     degree = None
    #     cross_point = cross_point[0]
    #     if abs(exit_point[0] - cross_point[0]) > abs(exit_point[2] - cross_point[0]):
    #         degree = np.array(
    #             (np.arctan2(exit_point[1] - 0, exit_point[0] - int(self.size_y / 2)) * 180) / np.pi,
    #             dtype=np.int32)
    #
    #     else:
    #         degree = np.array(
    #             (np.arctan2(exit_point[3] - 0, exit_point[1] - int(self.size_y / 2)) * 180) / np.pi,
    #             dtype=np.int32)
    #
    #     degree -= 90
    #
    #     return degree


if __name__ == '__main__':
    # cap = None
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

    cap = cv2.VideoCapture(0)
    cap.set(3, W_View_size)
    cap.set(4, H_View_size)

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
    TX_data_py2(serial_port, 128)

    time.sleep(1)
    while True:
        # video_load()
        if task_step == 1:
            line_precondition()
        elif task_step == 2:
            task_step = int(input("현재 테스크 2, 다음 테스크 입력바람"))
        elif task_step == 3:
            task_step = int(input("현재 테스크 3, 다음 테스크 입력바람"))
        else:
            task_step = int(input("테스크 잘 못 입력, 다음 테스크 입력바람 (1,2,3)"))

