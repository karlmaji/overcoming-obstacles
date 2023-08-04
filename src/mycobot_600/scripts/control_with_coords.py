import numpy as np
import math
from scipy.optimize import fsolve
import time
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

theta1_l=-180/180*PI
theta1_u=180/180*PI
theta2_l=-270/180*PI
theta2_u=90/180*PI
theta3_l=-150/180*PI
theta3_u=150/180*PI
theta4_l=-260/180*PI
theta4_u=80/180*PI
theta5_l=-168/180*PI
theta5_u=168/180*PI
theta6_l=-99999/180*PI
theta6_u=99999/180*PI

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
    
    def isRotationMatrix(self,R) :
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    def rotationMatrixToEulerAngles(self,R) :
        assert(self.isRotationMatrix(R))
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
    
    def f(self,X,a,b):
        x = X[0]
        y = X[1]
        z = X[2]
        w = X[3]
        return [250*y+250*w-a,
            -250*x-250*z-b,
            x*x+y*y-1,
            z*z+w*w-1]
        
    def Inverse(self, coords):
          R_x = np.array([[1,     0,         0          ],
          [0,     math.cos(coords[3]/180*PI), -math.sin(coords[3]/180*PI) ],
          [0,     math.sin(coords[3]/180*PI), math.cos(coords[3]/180*PI) ]
          ])
          R_y = np.array([[math.cos(coords[4]/180*PI),  0,   math.sin(coords[4]/180*PI) ],
          [0,           1,   0          ],
          [-math.sin(coords[4]/180*PI),  0,   math.cos(coords[4]/180*PI) ]
          ])
         
          R_z = np.array([[math.cos(coords[5]/180*PI),  -math.sin(coords[5]/180*PI),  0],
          [math.sin(coords[5]/180*PI),  math.cos(coords[5]/180*PI),   0],
          [0,           0,           1]
          ])
          R = np.dot(R_z, np.dot( R_y, R_x ))
          T=[[R[0,0],R[0,1],R[0,2],coords[0]],
             [R[1,0],R[1,1],R[1,2],coords[1]],
             [R[2,0],R[2,1],R[2,2],coords[2]],
             [0,0,0,1]]
          return T
    
    def computeparm(self,r11,r12,r13,px,r21,r22,r23,py,r31,r32,r33,pz,theta1,theta6,theta1_6,pram1_6):
        A1=[]
        B1=[]
        for i in theta1_6:
            x=theta1[i[0]]
            y=theta6[i[1]]
            A1.append(px*math.cos(x) + py*math.sin(x) - d6*r13*math.cos(x) - d6*r23*math.sin(x) - d5*r21*math.sin(x)*math.sin(y) - d5*r12*math.cos(x)*math.cos(y) - d5*r11*math.cos(x)*math.sin(y) - d5*r22*math.cos(y)*math.sin(x))
            B1.append(pz - d1 - d6*r33 - d5*r32*math.cos(y) - d5*r31*math.sin(y))
            pram1_6.append(i)
        pram=[]
        pram.append(A1)
        pram.append(B1)
        return pram
    def asin_to_angels(self,raw,lower,upper):
        result=[]
        if math.asin(raw)>lower and math.asin(raw)<upper:
            result.append(math.asin(raw))
        if PI-math.asin(raw)>lower and PI-math.asin(raw)<upper:
            result.append(PI-math.asin(raw))
        if -PI-math.asin(raw)>lower and -PI-math.asin(raw)<upper:
            result.append(-PI-math.asin(raw))
        return result
    def toangels(self,T):
          r11=T[0][0]
          r12=T[0][1]
          r13=T[0][2]
          px =T[0][3]
          r21=T[1][0]
          r22=T[1][1]
          r23=T[1][2]
          py =T[1][3]
          r31=T[2][0]
          r32=T[2][1]
          r33=T[2][2]
          pz =T[2][3]
          
          #theta
          theta1_5=[]
          theta1_6=[]
          #theta1
          #B+/B-
          D=d4
          A=py-d6*r23
          B=d6*r13-px
          theta1=[]
          if B**2-D**2+A**2 <0:
              #print("coords error")
              return 0
          if 2*math.atan((B-math.sqrt(B**2-D**2+A**2))/(D+A))>theta1_l and 2*math.atan((B-math.sqrt(B**2-D**2+A**2))/(D+A))<theta1_u:
            theta1.append(2*math.atan((B-math.sqrt(B**2-D**2+A**2))/(D+A)))
          if 2*math.atan((B+math.sqrt(B**2-D**2+A**2))/(D+A))>theta1_l and 2*math.atan((B+math.sqrt(B**2-D**2+A**2))/(D+A))<theta1_u:
            theta1.append(2*math.atan((B+math.sqrt(B**2-D**2+A**2))/(D+A)))

          #theta5
          theta5=[]
          index1=0
          index5=0
          for x in theta1:
            if math.acos(r23*math.cos(x)-r13*math.sin(x))>theta5_l and math.acos(r23*math.cos(x)-r13*math.sin(x))<theta5_u:
                theta5.append(math.acos(r23*math.cos(x)-r13*math.sin(x)))
                theta1_5.append([index1,index5])
                index5=index5+1
            if -math.acos(r23*math.cos(x)-r13*math.sin(x))>theta5_l and -math.acos(r23*math.cos(x)-r13*math.sin(x))<theta5_u:
                theta5.append(-math.acos(r23*math.cos(x)-r13*math.sin(x)))
                theta1_5.append([index1,index5])
                index5=index5+1
            index1=index1+1
          #theta6 +-180
          index1=0
          index6=0
          theta6=[]
          for x in theta1:
            if math.atan((r22*math.cos(x)-r12*math.sin(x))/(r11*math.sin(x)-r21*math.cos(x)))>theta6_l and math.atan((r22*math.cos(x)-r12*math.sin(x))/(r11*math.sin(x)-r21*math.cos(x)))<theta6_u:
                theta6.append(math.atan((r22*math.cos(x)-r12*math.sin(x))/(r11*math.sin(x)-r21*math.cos(x))))
                theta1_6.append([index1,index6])
                index6=index6+1
                if math.atan((r22*math.cos(x)-r12*math.sin(x))/(r11*math.sin(x)-r21*math.cos(x)))+PI<theta6_u:
                    theta6.append(math.atan((r22*math.cos(x)-r12*math.sin(x))/(r11*math.sin(x)-r21*math.cos(x)))+PI)
                    theta1_6.append([index1,index6])
                    index6=index6+1
                index1=index1+1
          #theta2,theta3
          theta1_2_3_6=[]
          pram1_6=[]
          pram=self.computeparm(r11,r12,r13,px,r21,r22,r23,py,r31,r32,r33,pz,theta1,theta6,theta1_6,pram1_6)

          result=[]         
          theta2=[]
          theta3=[] 
          X0 = [0, 0,0,0]
          index=0
          for i in range(0,len(pram[0])):
                x=pram[0][i]
                y=pram[1][i]
                result.append([fsolve(self.f, X0,args=(x,y)).tolist(),pram1_6[i]]) 
                
          index2=0
          index3=0
        #   for x in result:
        #       print(x)
          for i in range(0,len(result)):
              x=result[i][0]
              #asin <1
              if abs(x[2])<=1:
                result3 = self.asin_to_angels(x[2],theta2_l,theta2_u)
                for y in result3:
                    if y>theta2_l and y<theta2_u:
                        if abs(x[0])<=1:
                            result33=self.asin_to_angels(x[0],theta3_l+y,theta3_u+y)
                            for z in result33:
                                if z-y>theta3_l and z-y<theta3_u:
                                    theta3.append(z-y)
                                    theta1_2_3_6.append([result[i][1][0],index2,index3,result[i][1][1]])
                                    index3=index3+1
                                if PI-z-y>theta3_l and PI-z-y<theta3_u:
                                    theta3.append(PI-z-y)
                                    theta1_2_3_6.append([result[i][1][0],index2,index3,result[i][1][1]])
                                    index3=index3+1
                                theta2.append(y)
                                index2=index2+1
          
          #print("1236:",theta1_2_3_6)
          #change
          for i in range(0,len(result)):
              x=result[i][0]
              if abs(x[0])<=1:
                result0= self.asin_to_angels(x[0],theta2_l,theta2_u)
                for y in result0:
                    if y>theta2_l and y<theta2_u:
                        if abs(x[2])<=1:
                            result00=self.asin_to_angels(x[2],theta3_l+y,theta3_u+y)
                            for z in result00:
                                if z-y>theta3_l and z-y<theta3_u:
                                    theta3.append(z-y)
                                    theta1_2_3_6.append([result[i][1][0],index2,index3,result[i][1][1]])
                                    index3=index3+1
                                if PI-z-y>theta3_l and PI-z-y<theta3_u:
                                    theta3.append(PI-z-y)
                                    theta1_2_3_6.append([result[i][1][0],index2,index3,result[i][1][1]])
                                    index3=index3+1
                                theta2.append(y)
                                index2=index2+1

          theta4=[]
          theta234=[]
          theta1_2_3_4_6=[]
          index4=0
          for i in theta1_2_3_6:  
            x=theta1[i[0]]
            y=theta6[i[3]]
            
            theta234_raw= -(r12*math.cos(x)*math.cos(y) + r11*math.cos(x)*math.sin(y) + r22*math.cos(y)*math.sin(x) + r21*math.sin(x)*math.sin(y))
            #print("raw:",theta234_raw)
            
            result_234=self.asin_to_angels(theta234_raw,theta2_l+theta3_l+theta4_l,theta2_u+theta3_u+theta4_u)
            for x in result_234:
                if x-theta2[i[1]]-theta3[i[2]]>theta4_l and x-theta2[i[1]]-theta3[i[2]]<theta4_u:
                    theta234.append(x)   
                    theta4.append(x-theta2[i[1]]-theta3[i[2]])
                    theta1_2_3_4_6.append([i[0],i[1],i[2],index4,i[3]])
                    index4=index4+1
          theta1_2_3_4_5_6=[]
          for x in theta1_5:
              for y in theta1_2_3_4_6:
                  if x[0]==y[0]:
                      theta1_2_3_4_5_6.append([y[0],y[1],y[2],y[3],x[1],y[4]])
            #print(theta1_2_3_4_5_6)
        #   print("theta234:",[x*180/PI for x in theta234])
        #   print("theta1:",[x*180/PI for x in theta1])
        #   print("theta2:",[x*180/PI for x in theta2])
        #   print("theta3:",[x*180/PI for x in theta3])
        #   print("theta4:",[x*180/PI for x in theta4])
        #   print("theta5:",[x*180/PI for x in theta5])
        #   print("theta6:",[x*180/PI for x in theta6])
        #   print("num:",len(theta1_2_3_4_5_6))
        #   print("123456")
          theta_result=[]
          for x in theta1_2_3_4_5_6:
              #print(theta1[x[0]]*180/PI,theta2[x[1]]*180/PI,theta3[x[2]]*180/PI,theta4[x[3]]*180/PI,theta5[x[4]]*180/PI,theta6[x[5]]*180/PI)
              theta_result.append([theta1[x[0]],theta2[x[1]],theta3[x[2]],theta4[x[3]],theta5[x[4]],theta6[x[5]]])
          return theta_result
  
def coords_to_angels(coords,current_angels):        
    c=Kinematic()
    T=c.Inverse(coords)
    theta_raw=c.toangels(T)
    if theta_raw==0:
        print("coords error")
        return []
    theta_result=[]
    for x in theta_raw:
        value=True
        T_raw=c.Forward(x).tolist()
        for i in range(0,4):
            for j in range(0,4):
                if (T_raw[i][j]-T[i][j])>10:
                     value=False
        if value==True:
            theta_result.append(x)
    weight=[100,10,1,0.1,0.01,0.001]
    diff=[]   
    for i in range(0,len(theta_result)):
        ddiff=0
        for j in range(0,6):
            ddiff=weight[j]*abs(theta_result[i][j]-current_angels[j])+ddiff
        diff.append(ddiff)
    
    return [q/PI*180 for q in theta_result[diff.index(min(diff))]]




def main():
    #strat_time=time.time()
    # raw=[90.2651,0,116.7687,-106.1457,37.1629,142.4798]
    # raw1=[-20.488214,-45.000017,65.004723,24.785156,70.576172,0.175781]
    raw2=[0.000281,-89.999594,-0.000357,-89.824219,0.351563,-0.087891]
    # raw3=[2.198478,-45.642092,18.047614,-152.314453,-82.001953,-0.175781]
    # raw4=[-3.383341,-77.357570,56.844555,-162.861328,-86.748047,0.527344]
    # raw5=[-3.385380,5.313992,6.663513,-96.503906,-86.923828,0.527344]
    # raw6=[-88.691130,5.314060,6.663342,-96.503906,-86.923828,0.527344]
    # raw7=[0.000281,-89.999594,-0.000357,-89.824219,0.351563,-0.087891]
    # raw8=[0.000281,-89.999594,-0.000357,-89.824219,0.351563,-0.087891]

    q3=[x*PI/180 for x in raw2]
    c = Kinematic()
    out=c.Forward(q3)
    coords=[out[0,3],out[1,3],out[2,3]]+list(c.rotationMatrixToEulerAngles(out[0:3,0:3]))
    print("coords:",coords)
    # #coords=[-0.129576,183.858571,808.440915,89.998921,-0.087936,-179.648156]
    # theta=[x/180*math.pi for x in coords[3:6]]
    # T=c.Inverse(coords)
    # theta_raw=c.toangels(T)
    # theta_result=[]
    # for x in theta_raw:
    #     value=True
    #     T_raw=c.Forward(x).tolist()
    #     for i in range(0,4):
    #         for j in range(0,4):
    #             if (T_raw[i][j]-T[i][j])>1e-6:
    #                  value=False
    #     if value==True:
    #         theta_result.append(x)
    # print("theta_result")    
    # for x in theta_result:
    #     print([y*180/PI for y in x])          
        
    # end_time=time.time()
    # print("time:",(end_time-strat_time)*1000)
    
    
    ####################################################
    # coords=[125.629526,-592.144788,38.492960,-174.490805,-3.011632,0.489510]
    # current_angels=[3.202796,-38.172124,-15.242044,-106.787109,-84.375000,28.916016]
    # theta_result=coords_to_angels(coords,current_angels)
    # print("theta_result")   
    # print(theta_result) 


if __name__ == '__main__':
    main()