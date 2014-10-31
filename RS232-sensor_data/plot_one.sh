#!/bin/bash
##
## Takes an raw .senbin file and does post-processing & plots
## By default, do not apply avg&deriv filters, just plots.
##
## Flags:
##    -p   - do avg. and deriv processing before plotting
##

fields="timestamp accelX_curr accelY_curr accelZ_curr"
winsize=50

usage() {
	echo "$0 {[-p]} [file]{[file]...}"
	grep ^## $0
}

fltr1() {
	if [ $proc -eq 1 ]; then
		csv_winavg.pl -w $winsize
	else
		# pass thru
		cat
	fi
}
fltr2() {
	if [ $proc -eq 1 ]; then
		csv_winavg.pl -w $winsize | csv_derivative.pl -w 20
	else
		# pass thru
		cat
	fi
}
outformat() {
	get_csv_column.rb $fields | csv2plotit.sh
}

proc=0
case X"$1" in
	X-h|X--help)	usage;exit 0;;
	X-p)		proc=1;shift;;
	X-*)		usage; exit 1;;
esac
if [ X"$1" = X ]; then
	usage
	exit 1
fi

#-#########################
for i in $@; do
	if [ ! -f $i ]; then
		echo "ERROR:$0:$@: Could not find file $i!"
		exit 1
	fi
	if [ $proc -eq 1 ]; then
		pwatch.pl -c -f $i | fltr1 | outformat | plotit.pl -T "$i AVG" &
		pwatch.pl -c -f $i | fltr2 | outformat | plotit.pl -T "$i AVG DERIV" &
	else
		pwatch.pl -c -f $i | outformat | plotit.pl -T "$i - RAW" &
	fi
	sleep 0.1
done
