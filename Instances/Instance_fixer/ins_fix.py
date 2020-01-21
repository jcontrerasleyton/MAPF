import os
import shutil
import sys

def folder_creator(path):

    if not os.path.exists(path):
        os.mkdir(path)
        print("Directory " , path ,  " Created ")
    else:    
        print("Directory " , path ,  " already exists")


for filename in os.listdir("../Input_fix/"):
    shutil.rmtree("../Input_fix/"+filename)
#sys.exit(0)

for filename in os.listdir("../Input_original"):
    print filename

    archivo = open("../Input_original/"+filename, "r") 
    name = filename.split(".")[0]
    path = "../Input_fix/"+name
    folder_creator(path)
    path = path+"/Input_10"
    folder_creator(path)
    

    cont = 0
    x=0
    y=0
    top_down = ""
    old=-1

    for linea in archivo:
        cont = cont + 1

        if cont > 1:
            parts = linea.strip().split("\t")

            instance = int(parts[0])

            output = open(path+"/"+name+"_"+str(instance)+".agents", "a")
            
            if old != instance:
                old = instance                
                output.write("10\n")

           # break

            # X e Y se encuentran invertidos

            sx = int(parts[5])+1
            sy = int(parts[4])+1
            gx = int(parts[7])+1
            gy = int(parts[6])+1
            
            #print str(instance)+" "+str(sx)+","+str(sy)+","+str(gx)+","+str(gy)
            output.write(str(sx)+","+str(sy)+","+str(gx)+","+str(gy)+"\n")

            output.close()
            #print linea

        #print(linea) 
    #output.write(top_down)

    print "----------------------------------------------"

    archivo.close()
