#!/bin/bash
if [ X"$1" = X ]; then
	echo "ERROR: no filename" 
	exit 1
fi
./pwatch.pl -c -f $1 | get_csv_column.rb gyro_curr | ./csv_integral.pl | uniq | csv2plotit.sh | plotit.pl -T "$1" 

