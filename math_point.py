import numpy as np


# 기울기 차이 매트릭스 생성 함수
def degree_diff(slope_degree):
    diff_degree = slope_degree

    for i in range(len(slope_degree)-1):
        diff_degree = np.append(diff_degree, slope_degree, axis=0)

    diff_degree = np.reshape(diff_degree, (-1, len(slope_degree)))
    diff_degree_2 = diff_degree.T

    result_diff_degree = abs(diff_degree - diff_degree_2)
    result_diff2_degree = abs(result_diff_degree - 180)
    result_diff_degree = np.where(result_diff_degree >= result_diff2_degree, result_diff2_degree, result_diff_degree)


    return result_diff_degree


# 점과 직선의 거리 메트릭스 생성 함수
def line_to_point(line_point):
    # 기울기 --> 같은 경우 x=1이라는 방정식
    m = np.array(np.where(((line_point[:, 2]-line_point[:, 0]) != 0), ((line_point[:, 3]-line_point[:, 1])/(line_point[:, 2]-line_point[:, 0])), -1.1111))

    # 직선과의 거리차이 넣을 매트릭스 생성
    distance = np.zeros((len(line_point), len(line_point)))

    for i in range(len(line_point)-1):
        if m[i] != -1.1111:
            son = np.abs(np.array(-m[i] * line_point[i + 1:, 2] + line_point[i + 1:, 3]) - line_point[i, 1] +(m[i]*line_point[i, 0]))
            mother = np.sqrt(m[i]**2+1)
            temp_distance = son / mother
            distance[i, i+1:] = temp_distance
        else:
            distance[i, i+1:] = abs(line_point[i+1:, 3] - line_point[i, 1])

    # 거리 차이 매트릭스 완성
    distance = distance + distance.T

    return distance

# 교차점 찾는 함수
def cross_point(img, cross_point, cross_m, cross_b):
    cross_pmatrix = []
    diff_m_matrix = []
    rows, cols = img.shape[:2]

    for index, i in enumerate(zip(cross_point, cross_m, cross_b)):
        # i[0] = 좌표값, i[1] = 기울기, i[2] = 절편
        # 딱 수직인 경우
        if i[1] == 10000:

            # y = x - i[0][0] 인 방정식, 기울기 : 1 절편 : -i[0][0]
            for j in zip(cross_point[index+1:], cross_m[index+1:], cross_b[index+1:]):
                if j[1] == 10000:
                    continue
                x = i[0][0]
                y = j[1]*i[0][0] + j[2]

                # 기울기 수직 관계 (수직인 경우이니까 직각 기울기는 0임)
                if 0 < x and x < cols and 0 < y and y < rows:
                    cross_pmatrix.append((int(x), int(y)))
                    # 기울기 수직 관계 (수직인 경우이니까 직각 기울기는 0임)
                    diff_m_matrix.append(abs(j[1]))
        else:
            # y = x - i[0][0] 인 방정식, 기울기 : 1 절편 : -i[0][0]
            for j in zip(cross_point[index+1:], cross_m[index+1:], cross_b[index+1:]):
                if j[1] == 10000:
                    #위의 작업 해줘야 함
                    x = j[0][0]
                    y = i[1]*j[0][0] + i[2]

                else:
                    # 기울기 같으면 찾는거 X
                    if i[1] - j[1] == 0:
                        continue
                    x = -(i[2] - j[2]) / (i[1] - j[1])
                    y = i[1]*(-(i[2] - j[2]) / (i[1] - j[1])) + i[2]

                # 기울기 수직 관계 (직각 기울기는 역수의 마이너스임)
                if (abs(y - (i[1]*x+i[2])) < 20) and 0 < x and x < cols and 0 < y and y < rows:
                    cross_pmatrix.append((int(x), int(y)))
                    # 기울기 수직 관계 (직각 기울기는 역수의 마이너스임)
                    diff_m_matrix.append(abs(-(1/i[1])-j[1]))

        break

    result = []
    if len(diff_m_matrix) != 0:
        minValue = diff_m_matrix[0]     # 가장 작은 수
        minValue_2 = 100   # 두번째로 작은 수
        min_index = 0
        min_index_2 = 0

        for index, i in enumerate(diff_m_matrix):
            # 가장 작은 수이면
            if minValue >= i:

                if min_index == index:
                    continue

                if abs(cross_pmatrix[index][1] - cross_pmatrix[min_index][1]) <= 20:
                    continue

                minValue_2 = minValue
                min_index_2 = min_index
                minValue = i
                min_index = index

            # 가장 작은 수는 아니지만 두번째로 작은 수이면
            elif minValue_2 > i:

                if abs(cross_pmatrix[index][1] - cross_pmatrix[min_index][1]) <=20:
                    continue
                minValue_2 = i
                min_index_2 = index

        if min_index == min_index_2:
            result.append(cross_pmatrix[min_index])
        else:
            result.append(cross_pmatrix[min_index])
            result.append(cross_pmatrix[min_index_2])

    return result



