#!/bin/bash        

clear

#make executable

map=../$1
agents=../$2
#time=$3

echo
echo $map
echo $agents

#timeout 30m 
#./driver --map example/corridor_2.map --agents example/corridor_2_10_0.agents

agent=$(awk 'BEGIN{FS="Input_"}{print $3}' <<< "$agents")
agent=$(awk 'BEGIN{FS="/"}{print $1}' <<< "$agent")

file="../Results/wcbs_Input_"$agent"-w_"$3".csv"
t=400
t_=$t"s"

timeout $t_ ./driver --map $map --agents $agents --highway_w 1 --focal_w 1 --window_limit_w $3 --time_limit $4 
RETVAL=$?

if [[ $RETVAL -eq "124" ]] 
then
    echo "FAIL;"$agent";-1;-1;-1;-1;-1;-1;-1;-1;-1;"$t";"$t >> $file
fi

