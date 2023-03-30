

import matplotlib.pyplot as plt

from random import *


def dessin_pos(suppx,suppy,ptn_intpx,ptn_intpy) :



    #---------------------------recuperation des logs x,y-------------------------------------
    fichier = open("log_position.txt", "r")
    log=fichier.readlines() 
    fichier.close()

    trajx=[]   
    trajy=[]

    for i in range(1, len(log)) :
    
        if i!=1 :  

            if log[i].split(',')[1]!=log[i-1].split(',')[1]:
                trajx.append(log[i].split(',')[1])
                trajy.append(log[i].split(',')[2])
                

#---------------------plot,x,xlog-------------------------------------------------------------
    for i in range(0,len(trajx)) :
        xi = float(trajx[i]) 
        yi = float(trajy[i]) 
        plt.plot(xi,yi,'xb')

    j=-1
    #print(suppx)
    #print(suppy)
    #print(len(suppx))
    for i in range(0,len(suppx)) :
        j=j+1
        xi = float(suppx[i]) 
        yi = float(suppy[i]) 

        
        
        if (j%2)==0 :
            plt.plot(xi,yi,'pg')  
        else :
            plt.plot(xi,yi,'pg') 

    for i in range(0,len(ptn_intpx)) :
        j=j+1
        xi = float(ptn_intpx[i]) 
        yi = float(ptn_intpy[i])  
        plt.plot(xi,yi,'or') 




    plt.axis("equal")
    plt.gcf().canvas.mpl_connect(
        'key_release_event',
        lambda event: [exit(0) if event.key == 'escape' else None])
    plt.show()


    # def dessin_historique() :

    # #---------------------------recuperation des logs x,y-------------------------------------
    #     fichier = open("log_position.txt", "r")
    #     log=fichier.readlines() 
    #     fichier.close()

    #     trajx=[]   
    #     trajy=[]

    #     for i in range(1, len(log)) :
    
    #         if i!=1 :  

    #             if log[i].split(',')[1]!=log[i-1].split(',')[1]:
    #                 trajx.append(log[i].split(',')[1])
    #                 trajy.append(log[i].split(',')[2])
                
    #     for i in range(0,len(trajx)) :
    #         xi = float(trajx[i]) 
    #         yi = float(trajy[i]) 
    #         plt.plot(xi,yi,'xb')

    #     plt.axis("equal")
    #     plt.gcf().canvas.mpl_connect(
    #         'key_release_event',
    #         lambda event: [exit(0) if event.key == 'escape' else None])
    #     plt.show()