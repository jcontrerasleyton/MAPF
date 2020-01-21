import os

#Modifica formato de mapas

for filename in os.listdir("Maps_fix"):
    os.remove("Maps_fix/"+filename)

for filename in os.listdir("Maps_original"):
    print filename

    archivo = open("Maps_original/"+filename, "r") 
    output = open("Maps_fix/"+filename, "w")

    cont = 0
    x=0
    y=0
    top_down = ""

    for linea in archivo:
        cont = cont + 1
        if cont == 2:
            x = int(linea.strip().split(" ")[1])
        if cont == 3: 
            y = int(linea.strip().split(" ")[1])   
            linea = str(x+2)+","+str(y+2)
            #print linea
            output.write(linea+"\n")  

        if cont > 4:
            linea = linea.strip().replace(".","0,")
            linea = linea.strip().replace("@","1,")
            linea = linea.strip().replace("T","1,")
            linea = linea.strip().replace("W","1,")
            linea = "1,"+linea+"1"+"\n"

            if cont == 5:
                top_down = linea.replace("0","1")
                output.write(top_down)

            output.write(linea)
            #print linea

        #print(linea) 
    output.write(top_down)

    archivo.close()
    output.close()
