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
        self.x4=0
        self.y4=0

        self.angle_base_degret=angle_base_degret
        self.angle_base=angle_base_degret*math.pi/180
        self.range_radar=range_radar


    def determineration_ps(self):

        self.x2=self.x0+((self.range_radar/2)*math.sin(self.angle_base))
        self.y2=self.y0+((self.range_radar/2)*math.cos(self.angle_base))
        #print("ps : X= " + str(self.x2)+",Y= "+str(self.y2))



    def point_de_passsage(self) :



            r=self.range_radar
            B=[self.x2,self.y2]
            A=[self.x1,self.y1]

            CA=B[0]-A[0]
            CB=B[1]-A[1]


            AB=math.sqrt(CB*CB+CA*CA)
            CD=(CA*r)/AB
            AD=CA-CD
            DE=AD*CB/CA
            DE_suivantx=DE-CB
           
            AE=math.sqrt(CD*CD+DE*DE)
            self.x3=self.x2-CD
            self.y3=self.y2+DE_suivantx

            self.x4=self.x2+CD
            self.y4=self.y2-DE_suivantx

            # print("CA = "+str(CA))
            # print("CB = "+str(CB))
            # print("AD = "+str(AD))
            # print("DE = "+str(DE))
            # print("")
            # print("AB = "+str(AB))
            # print("AE = "+str(AE))
            # print("")
            # print("x3 = "+str(self.x3))
            # print("y3 = "+str(self.y3))

    def calcule_ps(self):
        self.determineration_ps()
        
        self.point_de_passsage()
        
        return self.x2,self.y2,self.x3,self.y3,self.x4,self.y4



