from random import *


def tirage_pos() :
    
    fichier = open("log_position.txt", "r")

    
    coordonee=fichier.readlines()
    
    fichier.close()

    n=[]
    flag=True


    x = input('nombre decibles ? (numero entier) : ')
   
    for i in range(0,int(x)) : 

        n.append(randint(1,len(coordonee)-1))
        
    n.sort(reverse=True)
    #print(n)
    

    fichier = open("tirage.txt", "w")
    for i in range(0,int(x) ):
        j=coordonee[n[i]].split(',')
        fichier.write(str(j[0]))
        fichier.write(',')
        fichier.write('\n')


