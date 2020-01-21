#!/bin/bash        

clear

#rm -r ../Input_old/*/

make clean
make executable

echo 
#read -p 'Instances: ' instances
instances=100

for map in `ls -v ../../Maps/Maps_fix/*.map`
do
    loop="{20..200..20}"
    
    if [[ $map != *Maps_fix/2_* ]]; then continue; fi

    if [[ $map == *Maps_fix/2_* ]]; then loop="{3..21..1}"; fi
        
    name=$(basename $map)
    name="${name%.*}"
    echo $name
    
    for i in $(eval echo $loop)
    do
        echo Name: $name, Agents: $i, Instances: $instances, Map: $map
        ./driver --map $map --name $name --agents $i --instances $instances
    done
done

echo 
echo All instances generated