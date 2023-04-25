import numpy as np
import matplotlib.pyplot as plt
from random import *
from time import *
import matplotlib.pyplot as plt


coordonnee= [
[0, -0.5, 'scan'], [0, 0.5, 'scan'], [0, 1.5, 'evite'], [0, 2.5, 'scan'], [0, 3.5, 'scan'], 
[0, 4.5, 'evite'], [1, -0.5, 'evite'], [1, 0.5, 'scan'], [1, 1.5, 'scan'], [1, 2.5, 'evite'], 
[1, 3.5, 'evite'], [1, 4.5, 'scan'], [2, -0.5, 'scan'], [2, 0.5, 'scan'], [2, 1.5, 'evite'],
[2, 2.5, 'scan'], [2, 3.5, 'scan'], [2, 4.5, 'evite'], [3, -0.5, 'scan'], [3, 0.5, 'evite'],
[3, 1.5, 'scan'], [3, 2.5, 'evite'], [3, 3.5, 'scan'], [3, 4.5, 'scan'], [4, -0.5, 'evite'],
[4, 0.5, 'evite'], [4, 1.5, 'scan'], [4, 2.5, 'evite'], [4, 3.5, 'evite'], [4, 4.5, 'scan'],
[5, -0.5, 'scan'], [5, 0.5, 'scan'], [5, 1.5, 'scan'], [5, 2.5, 'evite'], [5, 3.5, 'scan'],
[5, 4.5, 'evite']]
case=[
[[0.0, 0], [0.5, -0.5], [0.5, 0.5], [1.0, 0]],
[[0.0, 1], [0.5, 0.5], [0.5, 1.5], [1.0, 1]],
[[0.0, 2], [0.5, 1.5], [0.5, 2.5], [1.0, 2]],
[[0.5, 1.5], [1.0, 1], [1.0, 2], [1.5, 1.5]], 
[[0.5, 2.5], [1.0, 2], [1.0, 3], [1.5, 2.5]], 
[[0.5, 3.5], [1.0, 3], [1.0, 4], [1.5, 3.5]], 
[[0.5, 4.5], [1.0, 4], [1.5, -0.5], [1.5, 4.5]], 
[[1.0, 4], [1.5, 3.5], [1.5, 4.5], [2.0, 4]], 
[[1.5, -0.5], [1.5, 4.5], [2.0, 0], [2.5, -0.5]], 
[[1.5, 0.5], [2.0, 0], [2.0, 1], [2.5, 0.5]], 
[[1.5, 1.5], [2.0, 1], [2.0, 2], [2.5, 1.5]], 
[[2.0, 1], [2.5, 0.5], [2.5, 1.5], [3.0, 1]], 
[[2.0, 2], [2.5, 1.5], [2.5, 2.5], [3.0, 2]], 
[[2.0, 3], [2.5, 2.5], [2.5, 3.5], [3.0, 3]], 
[[2.0, 4], [2.5, 3.5], [2.5, 4.5], [3.0, 4]], 
[[2.5, 3.5], [3.0, 3], [3.0, 4], [3.5, 3.5]], 
[[2.5, 4.5], [3.0, 4], [3.5, -0.5], [3.5, 4.5]], 
[[3.0, 0], [3.5, -0.5], [3.5, 0.5], [4.0, 0]], 
[[3.0, 1], [3.5, 0.5], [3.5, 1.5], [4.0, 1]], 
[[3.5, 0.5], [4.0, 0], [4.0, 1], [4.5, 0.5]], 
[[3.5, 1.5], [4.0, 1], [4.0, 2], [4.5, 1.5]], 
[[3.5, 2.5], [4.0, 2], [4.0, 3], [4.5, 2.5]], 
[[3.5, 3.5], [4.0, 3], [4.0, 4], [4.5, 3.5]], 
[[4.0, 3], [4.5, 2.5], [4.5, 3.5], [5.0, 3]], 
[[4.0, 4], [4.5, 3.5], [4.5, 4.5], [5.0, 4]]]

def type_case(nbr_case=25,robot=[2.5,2]) :
    
    for i in range(0,(nbr_case+1)*(nbr_case+1)) :
        print("coordonnee",coordonnee[i])
        if coordonnee[i][0]>= robot[0] and  coordonnee[i][1]>= robot[1]  and  flag==False : 
        
            print("nos coordonnee",coordonnee[i])
            print("i",i)
            flag=True
            type=coordonnee[i][2]
            print("le robot est dans la case =",i)
            print("case de type :", type)
            print("les sorties/entrée disponible de cette cas sont les suivante : ",case[i-1])
            if type=='evite' :

                return 1,case[i-1]

            else :
                
                return 0,case[i-1]
