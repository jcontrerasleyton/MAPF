#!/bin/bash  

#Cuenta la cantidad de celdas libres en el mapa

for file in `ls -v Maps_original`
do
	echo -e $file '\t' '\t' $(grep -o -i \\. Maps_original/$file | wc -l)
done

				
