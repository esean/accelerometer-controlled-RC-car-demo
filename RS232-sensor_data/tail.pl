#!/usr/bin/env perl
$| = 1;	# turn on hotflush
#--------------------------------------------------------------------------------
# Just 'tail' a file.
#
#--------------------------------------------------------------------------------
use strict;
use Getopt::Std;
use File::Tail;

my $debug = 0;

&usage unless ($ARGV[0]);
my %opt;
getopts('dhf:cb',\%opt);
&usage if ($opt{'h'});
&usage if (!($opt{'f'}));
$debug=1 if ($opt{'d'});
my ($file) = $opt{'f'} if ($opt{'f'});
die "Cannot find file $file!" if (! -f $file);
my $format = 0;	# output formatted for plotit.pl, 1=strict csv file output
$format=1 if ($opt{'c'});
my $blocking = 0;
$blocking = 1 if ($opt{'b'});

my @A = ();
my @B = ();
my @C = ();
my $in_A;
my $in_B;
my $in_C;
my $sync = 0;
# now sent as ascii
my @sync_pattern = ('+','1',0,0,'2',0,0,'3',0,0,'4',0,0,'5',0,0,'6',0,0,'7',0,0,'8',0,0,'9',0,0,'A',0,0,'B',0,0);	# hex
my $first;
my $just_sync;
my $next_is_section;
my $dump_A = 0;
my $dump_B = 0;
my $dump_C = 0;
my $first_dump = 1;	# print csv header on first output
my $t;
my $ts = 0;
my $tindex = 0;
my @tinfo = (8,4,4,8,0);	# 0ms at end since last sample has both accel and gyro data at same time, thus add 0ms
my ($lastx,$lasty,$lastz,$lastg);
$lastx = 0;
$lasty = 0;
$lastz = 0;
$lastg = 0;
my $u_last = 'nil';
my @dline_last = ();
my $byte_cnt = 0;
my $start = 0;
my $dec;

my $fn;
if ($blocking == 1) {
	# normal tail open
	$fn = File::Tail->new(name=>$file,maxinterval=>1,interval=>1,tail=>-1);
	while (defined($t=$fn->read)) {
		chomp $t;
		print "$t\n"; 
	}
} else {
	open F, "<$file";
	while ($t = <F>) {
		chomp $t;
		print "$t\n";
	}
}
exit 0;

##################

sub usage {
	print "$0 [-h] [-b] [-c] [-f [file]]\n";
	print "Processes binary datafile from sensor module either as standalone file or in\n";
	print "real-time as data is being captured. Datafile is produced either from\n";
	print "serial cable capture or 'watch.sh [fn]'.\n";
	print "Flags:\n";
	print "   -c - outputs in CSV format, otherwise defaults to plotit.pl formatting\n";
	print "   -b - blocking read of [file], otherwise defaults to non-blocking (for use with plotit.pl)\n";
	exit 0;
}

