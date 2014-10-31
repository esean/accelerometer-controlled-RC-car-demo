#!/bin/bash
first_is_time=1
case X"$1" in 
	X-h|X--help)	echo "$0 {[-n]}"
			echo "Converts a CSV stream to plotit.pl compatible stream."
			echo "Makes the first column the x-axis (handy if timestamp"
			echo "is first CSV field). Use the [-n] option to turn this off."
			exit 0;;
	X-n)	first_is_time=0;;
	X-*)	echo "ERROR: unknown param:$1 -> Use $0 --help"; exit 1;;
esac
if [ $first_is_time -eq 1 ]; then
	sed -e 's/, */:/g' | sed -e 's/^\([0-9][^:]*\):/\1 /g'
else
	sed -e 's/, */:/g' 
fi
