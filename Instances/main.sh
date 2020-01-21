#!/bin/bash   

generators="Instance_fixer Instance_generator Instance_generator_patched Instance_generator_astar"
arr=($generators)

run () {
	cd ${arr[$1]}
	echo $(pwd)
	if (($1 == 0))
	then
		#echo "hello"
		python *.py
	else
		./main.sh
	fi
	cd ..
}

echo "----- Choose an Instance Generator  -----"
echo
echo "1) Instance Fixer "
echo "   Results in Input_fix"
echo "   (Translate the original instances)"
echo
echo "2) Instance Generator "
echo "   Results in Input_old"
echo "   (Random instances, may not reach the goal)"
echo
echo "3) Instance Generator patched"
echo "   Results in Input_patched"
echo "   (Patches the holes in the map, it always reach the goal)"
echo
echo "4) Instance Generator Astar"
echo "   Results stored in Input"
echo "   (Uses A* to validate if the agent can reach the goal)"
echo
echo "5) Use all Instance generators"
echo

#read option
option=5
echo
((option = option - 1))

if ((option < 4)) && ((option >= 0))
then
	run "$option"
else
	if ((option == 4))
	then 
		for i in {0..3}
		do
			#echo $i
			run "$i"
		done
	else
		echo ERROR!!! - Incorrect Argument
	fi
fi

echo End