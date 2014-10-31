#!/bin/bash
usage() {
	echo
	echo "$0 [-c|-p|-x|-X] [file]"
	echo
	echo "For capturing or processing data from sensor module board."
	echo
	echo "Requirements:"
	echo "1) kermit"
	echo "2) for capture, requires logport kermit script,"
	echo "  http://www.columbia.edu/kermit/ftp/scripts/ckermit/logport" 
	echo "3) for parsing, requires pwatch.pl parser script"
	echo "4) alarmed"
	echo
	echo "Examples:"
	echo "   $0 -c [fn]   -> start a capture logging to [fn]"
	echo "   $0 -p [fn]   -> process sensor data in [fn]"
	echo "   $0 -x [fn]{[fields]}   -> while(1) plot sensor data in [fn], if [fields] is provided, only show those"
	echo "   $0 -X [fn]{[fields]}   -> plot sensor data in [fn], if [fields] is provided, only show those"
	echo
}
die() {
	echo "[`date`]:ERROR:$0:$@" >&2
	exit 1
}
capture=0
parse=0
plot=0
fields=''
fn=''
timeout=5
whileone=0;
case X"$1" in
	X-h|X--help)	usage;exit 0;;
	X-c)	capture=1;shift;fn="$1";;
	X-p)	parse=1;shift;fn="$1";;
	X-x)	plot=1;shift;fn="$1";shift;fields="$@";whileone=1;;
	X-X)	plot=1;shift;fn="$1";shift;fields="$@";timeout=65530;;
	X|X*)	 usage;exit 1;;
esac
if [ X"$fn" = X ]; then
	die "No file was provided on cmd-line"
fi


if [ $capture -eq 1 ]; then
	# need this kermit script file
	if [ ! -f "logport.kermit" ]; then
		die "Could not find logport.kermit script - get here:http://www.columbia.edu/kermit/ftp/scripts/ckermit/logport"
	fi
	logport.kermit $fn &>/dev/null
	exit
fi
if [ $parse -eq 1 -o $plot -eq 1 ]; then
	# need this file
	if [ ! -f $fn ]; then
		die "Could not find file $fn to parse"
	fi
	if [ $plot -eq 0 ]; then
		# output in csv format
		pwatch.pl -b -c -f $fn
	else
		# plotit.pl output
		while [ 1 ]; do
			t=`/usr/bin/mktemp /tmp/watch.tmp.XXXXXXXX`
			if [ X"$fields" = X ]; then
				pwatch.pl -f $fn > $t
			else
				# we dump it in CSV format, get the fields we want, then convert to plotit.pl format
				pwatch.pl -c -f $fn | get_csv_column.rb $fields | sed -e 's/, */:/g' | sed -e 's/^\([0-9][^:]*\):/\1 /g' > $t
			fi
			alarmed $timeout plotit.pl -T "$fn" -t $timeout -f $t
			rm -f $t
			if [ $whileone -eq 0 ]; then
				exit
			fi
		done
	fi
fi

