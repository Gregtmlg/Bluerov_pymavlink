
import math
from random import *
import tirage_code as tr
import point_de_pasage_cible as pdpc
import historique_rov as hts_rov
def coordonees_part_2() : 
    print("recherche des cooonnées des cibles ")
    tr.tirage_pos() 
    fichier = open("tirage.txt", "r")
    time=fichier.readlines()
    
    fichier.close()

    fichier = open("log_position.txt", "r")
    log=fichier.readlines() 
    fichier.close()
        

    ptn_suppx=[]
    ptn_suppy=[]
    ptn_intpx=[]
    ptn_intpy=[]
    x=[]
    y=[]
    donee_ligne=[]
    #print(len(time))
    for i in range(0,len(time)) :
       
        j=time[i].split(',')

        for l in range(0, len(log)) :
            
            if j[0]==log[l].split(',')[0] :
                
                #print( str(j[0])+"=="+str(log[l].split(',')[0])+'\n')
                donee_ligne.append(log[l])
        
        x1=float(log[-1].split(',')[1])
        y1=float(log[-1].split(',')[2] )
        #print(donee_ligne)
        

    #print("liste="+str(donee_ligne))
    donnee=[]
    
    print("calcule des points de passages")
    for i in range(0,len(time)) :

        donnee=donee_ligne[i].split(',')
        angle=float(donnee[4])
        x0=float(donnee[1])
        y0=float(donnee[2])
        

        point=pdpc.Angle_calcule(x0,y0,x1,y1,(angle*180/math.pi),1)
        x2,y2,x3,y3,x4,y4=point.calcule_ps()

        x1,y1=x4,y4
        ptn_intpx.append(x2)
        ptn_intpy.append(y2)
        x.append(x3)
        y.append(y3)    
        ptn_suppx.append(x3)
        ptn_suppy.append(y3)
        x.append(x4)
        y.append(y4)
        ptn_suppx.append(x4)
        ptn_suppy.append(y4)
    print("démarrage")
    hts_rov.dessin_pos(ptn_suppx,ptn_suppy,ptn_intpx,ptn_intpy)
    return x,y


