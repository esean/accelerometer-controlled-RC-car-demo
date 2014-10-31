#!/bin/bash

fields="timestamp accelX_curr accelY_curr accelZ_curr"
#winsize=1
winsize=50

for i in *.senbin; do
	#watch.sh -X $i $fields &
	# ok, but never returns
	#watch.sh -p $i | csv_winavg.pl -w $winsize | csv2plotit.sh | plotit.pl -T "$i" &

	# so, instead...

# NOTE: get_csv_column _HAS_ to go after csv_winavg, as _winavg expects all fields present for processing
	pwatch.pl -c -f $i | csv_winavg.pl -w $winsize | get_csv_column.rb $fields | csv2plotit.sh | plotit.pl -T "$i AVG" &
	pwatch.pl -c -f $i | csv_winavg.pl -w $winsize | csv_derivative.pl -w 20 | get_csv_column.rb $fields | csv2plotit.sh | plotit.pl -T "$i AVG DERIV" &

	sleep 0.1
done
