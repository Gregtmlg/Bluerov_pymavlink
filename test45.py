import math
import numpy as np
from random import *
import matplotlib.pyplot as plt

def dotproduct(v1, v2):
    return sum((a*b) for a, b in zip(v1, v2))

def length(v):
    return math.sqrt(dotproduct(v, v))


def trouve_angle(v1, v2):
    return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))

class Angle_calcule: 
   

    def __init__(self,x0,y0,x1,y1,angle_base_degret,range_radar) :

        self.x0=x0
        self.y0=y0

        self.x1=x1
        self.y1=y1

        self.x2=0
        self.y2=0

        self.x3=0
        self.y3=0
        
        self.angle_base_degret=angle_base_degret
        self.angle_base=angle_base_degret*math.pi/180
        self.range_radar=range_radar

        self.angle_final_degretx=0
        self.angle_final_degrety=0
        self.angle_finalx=0
        self.angle_finaly=0

    def determineration_ps(self):



        
        self.y2=self.y0+((self.range_radar/2)*math.cos(self.angle_base))
        #print("ps : X= " + str(self.x2)+",Y= "+str(self.y2))

    def calcul_angle(self) :
        add=10000000
        origine = [0,add]
        v1_2=np.array([abs(self.x1)-abs(self.x2),abs(self.y1)-abs(self.y2)])
        
        v2_y=np.array([self.x2-origine[0],self.y2-origine[1]]) 


        
        self.angle_finaly=trouve_angle(v1_2, v2_y)
       
        self.angle_final_degretx=self.angle_finalx*180/math.pi
        self.angle_final_degrety=self.angle_finaly*180/math.pi
        #print("angle finale x = "+str( self.angle_final_degretx))
        #print("angle finale y = "+str( self.angle_final_degrety))



    def point_de_passsage(self) :
        print("angle appliqué "+str(self.angle_finaly*180/math.pi))
        self.x3=self.x2+((self.range_radar/2)*math.sin(self.angle_finaly))
        self.y3=self.y2+((self.range_radar/2)*math.cos(self.angle_finaly))

        self.x4=self.x2+((-self.range_radar/2)*math.sin(self.angle_finaly))
        self.y4=self.y2+((-self.range_radar/2)*math.cos(self.angle_finaly))


        v2_x=np.array([self.x1-self.x2,self.y1-self.y2]) 
        v3_x=np.array([self.x4-self.x3,self.y4-self.y3]) 
        angle_entre=trouve_angle(v2_x,v3_x)
        print("base "+str(angle_entre*180/math.pi))

        if angle_entre*180/math.pi> 1 :
            print("in1")   
            plt.plot(self.x3,self.y3,'Dr')  
            plt.plot(self.x4,self.y4,'dr')
            
            self.angle_final_degrety=abs(self.angle_final_degrety-180)
            self.angle_finaly=self.angle_final_degrety*math.pi/180

            self.x3=self.x2+((self.range_radar/2)*math.sin(self.angle_finaly))
            self.y3=self.y2+((self.range_radar/2)*math.cos(self.angle_finaly))

            self.x4=self.x2+((-self.range_radar/2)*math.sin(self.angle_finaly))
            self.y4=self.y2+((-self.range_radar/2)*math.cos(self.angle_finaly))

            v2_x=np.array([self.x1-self.x2,self.y1-self.y2]) 
            v3_x=np.array([self.x4-self.x3,self.y4-self.y3]) 
            angle_entre=trouve_angle(v2_x,v3_x)
            print("correction "+str(angle_entre*180/math.pi))

        

    def dessin(self) :
        
        # plt.plot(self.x0,self.y0,'ob')
        plt.plot(self.x1,self.y1,'or')
        # plt.plot(self.x2,self.y2,'xk')
        # plt.plot(self.x3,self.y3,'Dg')  
        # plt.plot(self.x4,self.y4,'dg')

        plt.axis("equal")
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        plt.show()

    def calcule_ps(self):
        self.determineration_ps()
        self.calcul_angle()
        self.point_de_passsage()
        self.dessin()
        return self.x3,self.y3,self.x4,self.y4


point1= Angle_calcule(0, 0,   0,   0, 45, 2)
point2= Angle_calcule(0, 0,  10,  10, 45, 2)
point3= Angle_calcule(0, 0,  10,  1, 45, 2)
point4= Angle_calcule(0, 0,  10, -10, 45, 2)
point5= Angle_calcule(0, 0,   0, -10, 45, 2)    
point6= Angle_calcule(0, 0, -10, -10, 45, 2)
point7= Angle_calcule(0, 0, -10,   0, 45, 2)
point8= Angle_calcule(0, 0, -10,  10, 45, 2)


point1.calcule_ps()
point2.calcule_ps()
point3.calcule_ps()
point4.calcule_ps()
point5.calcule_ps()
point6.calcule_ps()
point7.calcule_ps()
point8.calcule_ps()
