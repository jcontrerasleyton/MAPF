#!/bin/bash        

rm -r ./Results/*
rm -r ./Nohup/*

cd MAPF_stripped_WHCA
#make clean
#make executable
cd ..

time=300
#input="Input"
input="Input_patched"
#input="Input_fix"

for file in `ls -v ../../Maps/Maps_fix/*`
do
	#echo $file 

	#if [[ $file != */2_* ]] && [[ $file != */1_* ]]
	if [[ $file != */1_* ]]
	then 
		if [[ $file != */Shanghai_0_256* ]] && [[ $file != */Berlin_0_256* ]] && [[ $file != */Boston_0_256* ]]
		then
			continue
		fi
	fi
	
	echo $file 
		
	./main_cbs.sh $file $time $input 

done

echo "All instances done"