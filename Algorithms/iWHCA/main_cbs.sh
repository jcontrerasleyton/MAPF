#!/bin/bash     

rm ./Results/*.csv

cust_func(){
  	echo "agents_file;w_limit;status;agents;cost;makespand;mov_cost;max_mov;low_exp;low_gen;iterations;cbs_time;total_time" >> ./Results/whca_$1-w_$2.csv
	for file in `ls -v ../../Instances/$3/$4/$1/`
	do
		#fil=$file
		file=../../Instances/$3/$4/$1/$file
		#echo $5 - $1 - $w - $file
			
		#break

		cd MAPF_stripped_WHCA/ #Windowed Cooperative A* (w=-1 is Cooperative A*)
		#./main.sh $5 $file $2 $6
		echo $(./main.sh $5 $file $2 $6) >> ../Nohup/$3/$4/$1/nohup_$1_$2.dat
		#./main.sh "$5" "$file" "$2" "$6" > ../Nohup/$3/$4/$1/nohup_$1_$2_$fil.dat
		cd ..

		#break
	done
	mv ./Results/whca_$1-w_$2.csv ./Results/$3/$4/$1/whca_$1-w_$2.csv
	#break
}


echo $1

map=$1
w_window="4 8 16 32" #-1 is HCA*
#w_window="4 32" #-1 is HCA*
time=$2
input=$3 

folder=$(basename $map)
folder="${folder%.*}"

echo Map: $map - Folder: $folder 
 
mkdir -p ./Results/$input/$folder
mkdir -p ./Nohup/$input/$folder

#if [[ $folder != 2_free_8x8 ]]
#then
#	agent="Input_20 Input_60 Input_100 Input_140 Input_180"
#else	
#	agent="Input_3 Input_4 Input_5 Input_6 Input_7 Input_8 Input_9 Input_10 Input_11 Input_12 Input_13 Input_14 Input_15 Input_16 Input_17 Input_18 Input_19 Input_20 Input_21"
#fi

for agents in `ls -v ../../Instances/$input/$folder/`
#for agents in $agent
do
	mkdir -p ./Results/$input/$folder/$agents
	mkdir -p ./Nohup/$input/$folder/$agents

	for w in $w_window
	do
		cust_func "$agents" "$w" "$input" "$folder" "$map" "$time" &
	done
	wait
	echo Done
	#break
done
