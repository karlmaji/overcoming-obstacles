#UR3正逆运动学代码
#Created By Dai Cen
#2023.3.6

import numpy as np
import math


PI = math.pi
alphaDH = [0.0, -PI/2.0, 0.0, 0.0, -PI/2, PI/2]  # 连杆长度a  units: radian
aDH = [0,0,250,250,0,0]  # 连杆扭角α  units: mm
dDH = [210,0,0,109.5,107,76.2]  # 连杆距离d  units: mm
a1=0 
a2=0 
a3=250
a4=250
a5=0
a6=0

d1=210
d2=0
d3=0
d4=109.5
d5=107
d6=76.2

q1_range=[-180,180]
q_range=[[-270,90],[-150,150],[-260,80],[-168,168],[-174,174]]
# 输入位姿矩阵T
# 例如：T=np.matrix(np.array([[0.5,-0.7,-0.3,36],[-0.3,-0.6,0.6,-28.3],[-0.7,-0.2,-0.6,355.7],[0,0,0,1]]))
 
# 输出解的列表q_list,里面的一个列表就代表一组解
# 例如：q_list=[[q1_1,q21,……],[],[],……]
 
def SIGN(x):
    return (x > 0)-(x < 0)


class Kinematic:
    def __init__(self):
        global alphaDH
        global aDH
        global dDH
        global PI

    def Forward(self, q):  # q为关节转角 units： radian
        T01 = np.zeros((4, 4))
        T12 = np.zeros((4, 4))
        T23 = np.zeros((4, 4))
        T34 = np.zeros((4, 4))
        T45 = np.zeros((4, 4))
        T56 = np.zeros((4, 4))

        T01 = [[math.cos(q[0]),                     -math.sin(q[0]),                       0,                     aDH[0]],
               [math.sin(q[0])*math.cos(alphaDH[0]), math.cos(q[0])*math.cos(alphaDH[0]), -math.sin(alphaDH[0]), -dDH[0]*math.sin(alphaDH[0])],
               [math.sin(q[0])*math.sin(alphaDH[0]), math.cos(q[0])*math.sin(alphaDH[0]),  math.cos(alphaDH[0]),  dDH[0]*math.cos(alphaDH[0])],
               [0.0, 0.0, 0.0, 1.0]]
        
        T12 = [[math.cos(q[1]),                     -math.sin(q[1]),                       0,                     aDH[1]],
               [math.sin(q[1])*math.cos(alphaDH[1]), math.cos(q[1])*math.cos(alphaDH[1]), -math.sin(alphaDH[1]), -dDH[1]*math.sin(alphaDH[1])],
               [math.sin(q[1])*math.sin(alphaDH[1]), math.cos(q[1])*math.sin(alphaDH[1]),  math.cos(alphaDH[1]),  dDH[1]*math.cos(alphaDH[1])],
               [0.0, 0.0, 0.0, 1.0]]
        
        T23 = [[math.cos(q[2]),                     -math.sin(q[2]),                       0,                     aDH[2]],
               [math.sin(q[2])*math.cos(alphaDH[2]), math.cos(q[2])*math.cos(alphaDH[2]), -math.sin(alphaDH[2]), -dDH[2]*math.sin(alphaDH[2])],
               [math.sin(q[2])*math.sin(alphaDH[2]), math.cos(q[2])*math.sin(alphaDH[2]),  math.cos(alphaDH[2]),  dDH[2]*math.cos(alphaDH[2])],
               [0.0, 0.0, 0.0, 1.0]]

        T34 = [[math.cos(q[3]),                     -math.sin(q[3]),                       0,                     aDH[3]],
               [math.sin(q[3])*math.cos(alphaDH[3]), math.cos(q[3])*math.cos(alphaDH[3]), -math.sin(alphaDH[3]), -dDH[3]*math.sin(alphaDH[3])],
               [math.sin(q[3])*math.sin(alphaDH[3]), math.cos(q[3])*math.sin(alphaDH[3]),  math.cos(alphaDH[3]),  dDH[3]*math.cos(alphaDH[3])],
               [0.0, 0.0, 0.0, 1.0]]
        
        T45 = [[math.cos(q[4]),                     -math.sin(q[4]),                       0,                     aDH[4]],
               [math.sin(q[4])*math.cos(alphaDH[4]), math.cos(q[4])*math.cos(alphaDH[4]), -math.sin(alphaDH[4]), -dDH[4]*math.sin(alphaDH[4])],
               [math.sin(q[4])*math.sin(alphaDH[4]), math.cos(q[4])*math.sin(alphaDH[4]),  math.cos(alphaDH[4]),  dDH[4]*math.cos(alphaDH[4])],
               [0.0, 0.0, 0.0, 1.0]]
        
        T56 = [[math.cos(q[5]),                     -math.sin(q[5]),                       0,                     aDH[5]],
               [math.sin(q[5])*math.cos(alphaDH[5]), math.cos(q[5])*math.cos(alphaDH[5]), -math.sin(alphaDH[5]), -dDH[5]*math.sin(alphaDH[5])],
               [math.sin(q[5])*math.sin(alphaDH[5]), math.cos(q[5])*math.sin(alphaDH[5]),  math.cos(alphaDH[5]),  dDH[5]*math.cos(alphaDH[5])],
               [0.0, 0.0, 0.0, 1.0]]
        
        T06 = np.matmul(
            np.matmul(np.matmul(np.matmul(np.matmul(T01, T12), T23), T34), T45), T56)
        return T06

    def Inverse(self, T):
        errTolerate = 0.000001  # 误差容忍度
    #  q_final = np.zeros([1, 6])
        th = np.zeros([8, 7])  # 返回值，第七列为1表示无解可用
        # n vector
        nx = T[0, 0]
        ny = T[1, 0]
        nz = T[2, 0]
        # o vector
        ox = T[0, 1]
        oy = T[1, 1]
        oz = T[2, 1]
        # a vector
        ax = T[0, 2]
        ay = T[1, 2]
        az = T[2, 2]
        # p vector
        px = T[0, 3]
        py = T[1, 3]
        pz = T[2, 3]

        A = dDH[5]*ay-py
        B = dDH[5]*ax-px
        C = dDH[1]+dDH[3]
        row = math.sqrt(A*A+B*B)

        phi = math.atan2(A, B)
        D = C/row

        if math.fabs(D) > 1:
            print("angle1 solve err,approximate solution")

            if math.fabs(D)-errTolerate > 1:
                th = []
                return(th)
            D = SIGN(D)

        E = math.sqrt(1-D*D)

        th1 = [[phi-math.atan2(D, E)],
               [phi-math.atan2(D, -E)]]
        th1 = np.array(th1)

        th[0:4, 0] = th1[0]
        th[4:8, 0] = th1[1]
        # 到此步无问题
        th5 = np.zeros([2, 2])
        th5 = np.array(th5)
        th6 = np.zeros([2, 2])
        th6 = np.array(th6)

        for i in range(2):  # 输出0，1

            A6 = nx*math.sin(th1[i]) - ny*math.cos(th1[i])  # T23456(2,0) 单值
            B6 = ox*math.sin(th1[i]) - oy*math.cos(th1[i])  # T23456(2,1)  单值
            C6 = ax*math.sin(th1[i]) - ay*math.cos(th1[i])  # T23456(2,2)  单值

            tempTh5 = math.acos(C6)  # 单值
            signTh5_1 = SIGN(math.sin(tempTh5))  # 关节5第一种取值的正弦值的正负号
            signTh5_2 = SIGN(math.sin(-tempTh5))  # 关节5第二种取值的正弦值的正负号
            th5[i, 0] = tempTh5
            th5[i, 1] = -tempTh5
            th6[i, 0] = math.atan2(B6*signTh5_1, -A6*signTh5_1)
            th6[i, 1] = math.atan2(B6*signTh5_2, -A6*signTh5_2)

            th[0, 5] = th6[0, 0]
            th[1, 5] = th6[0, 0]
            th[2, 5] = th6[0, 1]
            th[3, 5] = th6[0, 1]

            th[4, 5] = th6[1, 0]
            th[5, 5] = th6[1, 0]
            th[6, 5] = th6[1, 1]
            th[7, 5] = th6[1, 1]

            th[0, 4] = th5[0, 0]
            th[1, 4] = th5[0, 0]
            th[2, 4] = th5[0, 1]
            th[3, 4] = th5[0, 1]

            th[4, 4] = th5[1, 0]
            th[5, 4] = th5[1, 0]
            th[6, 4] = th5[1, 1]
            th[7, 4] = th5[1, 1]

        for i in range(4):
            th1_single = np.array(th[i*2, 0])
            th5_single = np.array(th[i*2, 4])
            th6_single = np.array(th[i*2, 5])
            s234 = -math.cos(th6_single)*(ox*math.cos(th1_single) + oy*math.sin(th1_single)) - \
                math.sin(th6_single)*(nx*math.cos(th1_single) +
                                      ny*math.sin(th1_single))
            c234 = oz*math.cos(th6_single) + nz*math.sin(th6_single)
            th234 = math.atan2(s234, c234)

            # T234
            A14 = px*math.cos(th1_single) - dDH[5]*(ax*math.cos(th1_single) + ay*math.sin(th1_single)) + py*math.sin(th1_single) - dDH[4]*(math.cos(th6_single)*(
                ox*math.cos(th1_single) + oy*math.sin(th1_single)) + math.sin(th6_single)*(nx*math.cos(th1_single) + ny*math.sin(th1_single)))  # a3*cos(c2 + c3) + a2*cos(c2)
            # a3*sin(c2 + c3) + a2*sin(c2)
            A24 = pz-dDH[0]-az*dDH[5]-dDH[4] * \
                (oz*math.cos(th6_single)+nz*math.sin(th6_single))
            B2 = (A14*A14+A24*A24+aDH[1]*aDH[1]-aDH[2]*aDH[2])/(2*aDH[1])

            row2 = math.sqrt(A14*A14+A24*A24)
            phi2 = math.atan2(A24, A14)
            C2 = B2/row2

            if math.fabs(C2) > 1:
                if math.fabs(C2)-errTolerate > 1:
                    th[2*i:2*i+1, 6] = 1
                C2 = SIGN(C2)
            # print(C2)
            th2_phi = math.acos(C2)
            th2 = np.array([phi2-th2_phi, phi2+th2_phi])

            c23 = np.array([(A14-aDH[1]*math.cos(th2[0]))/aDH[2],
                           (A14-aDH[1]*math.cos(th2[1]))/aDH[2]])
            s23 = np.array([(A24-aDH[1]*math.sin(th2[0]))/aDH[2],
                           (A24-aDH[1]*math.sin(th2[1]))/aDH[2]])

            th23 = np.array([math.atan2(s23[0], c23[0]),
                            math.atan2(s23[1], c23[1])])
            th3 = th23-th2
            th4 = th234-th2-th3

            th[2*i, 1] = th2[0]
            th[2*i+1, 1] = th2[1]
            th[2*i, 2] = th3[0]
            th[2*i+1, 2] = th3[1]
            th[2*i, 3] = th4[0]
            th[2*i+1, 3] = th4[1]
        return th

    def best_q_solution_inverse(self, weights, T, q_front):
        th = self.Inverse(T)
        scores = np.array(np.zeros([8]))

        for i in range(8):
            for j in range(6):
                scores[i] += weights[j]*math.fabs(th[i, j]-q_front[j])

        print(scores)
        index = np.argmin(scores, axis=0)  # 取scores数组中，返回行最大元素的index

        q_final = np.array(np.zeros([6]))
        for i in range(6):
            q_final[i] = th[index, i]

        return q_final
    
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R) :
    assert(isRotationMatrix(R))
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])*180/PI

def angels_to_coords(J):
    J=[x*PI/180 for x in J]
    out=Kinematic().Forward(J)
    coords=[out[0,3],out[1,3],out[2,3]]+list(rotationMatrixToEulerAngles(out[0:3,0:3]))
    return coords

def main():

    q1 = [-10*PI/180, -49.995012*PI/180 ,109.998288*PI/180 ,-45.087891*PI/180 ,75.498047*PI/180 ,0.175781*PI/180]
    q2 = [-10, -49.995012 ,109.998288 ,-45.087891 ,75.498047,0.175781]
    xx=-90
    raw=[-0.000096,-90.000839,-0.000892,-90.263672,-0.351562,-0.087891]
    raw1=[90.2651,98.8476,116.7687,-106.1457,37.1629,142.4798]
    q3=[x*PI/180 for x in raw1]

    c = Kinematic()
    print(c.Forward(q3))
    #print(c.Inverse(c.Forward(q1)))
    out=c.Forward(q3)
    print('x:',out[0,3])
    print('y:',out[1,3])
    print('z:',out[2,3])
    rx =  math.atan2(math.sqrt(out[2,0]*out[2,0]+out[2,1]*out[2,1]),out[2,2])*180/PI
    ry = -math.atan2(out[2,0], out[2,1])*180/PI
    rz =  math.atan2(out[0,2],-out[1,2])*180/PI
    print("rx:",rx)
    print("ry:",ry)
    print("rz:",rz)
    coords=[out[0,3],out[1,3],out[2,3]]+list(rotationMatrixToEulerAngles(out[0:3,0:3]))
    print(coords)
    # f = open('/home/lihaolin/mycobot/src/mycobot_600/scripts/data.txt', 'w')
    # j1=-180
    # j2=-270
    # j3=-150
    # j4=-260
    # j5=-168
    # j6=-174
    # # for i in range (0,int(360/15)):
    # #     print("i:",i)
    # #     print('\n')
    # #     j1=j1+i*15
    # #     for j in range (0,int(360/15)):
    # #         j2=j2+j*15
    # #         for k in range (0,int(300/15)):
    # #             j3=j3+k*15
    # #             for l in range (0,int(340/15)):
    # #                 j4=j4+l*15
    # #                 for z in range (0,int(336/15)):
    # #                     j5=j5+z*15
    # #                     for x in range (0,int(348/15)):
    # #                         j6=j6+x*15
    # #                         f.write(str([round(x,1) for x in list(angels_to_coords([j1,j2,j3,j4,j5,j6]))]))
    # #                         f.write(' ')
    # #                         f.write(str([j1,j2,j3,j4,j5,j6]))
    # #                         f.write('\n')
    # # f.close()
    T=np.matrix(np.array([
        [0.5,-0.7,-0.3,36],
        [-0.3,-0.6,0.6,-28.3],
        [-0.7,-0.2,-0.6,355.7],
        [0,0,0,1]
        ]))


if __name__ == '__main__':
    main()