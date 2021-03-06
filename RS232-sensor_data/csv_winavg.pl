#!/usr/bin/env perl
$| = 1;	# turn on hotflush
#--------------------------------------------------------------------------------
# Average CSV fields in moving window.
#
# Assumptions:
#	- samples are in signed decimal
#	- first field is timestamp, don't average
#	- delimt = ,
#	- order is Ax,Ay,Az,G
#--------------------------------------------------------------------------------
use strict;
use Getopt::Std;

my $debug = 0;

#&usage unless ($ARGV[0]);
my %opt;
getopts('dhf:w:',\%opt);
&usage if ($opt{'h'});
$debug=1 if ($opt{'d'});
#my ($file) = $opt{'f'} if ($opt{'f'});
#die "Cannot find file $file!" if (! -f $file);

my $winsize = 10;
$winsize = $opt{'w'} if ($opt{'w'});

my @T = ();
my @Ax = ();
my @Ay = ();
my @Az = ();
my @G = ();
my $AxS = 0;
my $AyS = 0;
my $AzS = 0;
my $GS = 0;
my $tT = 0;
my $tAx = 0;
my $tAy = 0;
my $tAz = 0;
my $tG = 0;

while(my $ln = <>) {
	chomp $ln;
	print "DBG:newLine:ln=$ln\n" if ($debug>0);
	if ($ln =~ /^#/) {
		print "$ln\n";
		next;
	}
	my (@elem) = split(/,/,$ln);

	# elem[0] is timestamp
	if (scalar(@Ax) == $winsize) {
		die "Ouch, array is already $winsize before push's";
	}
	push @T,$elem[0];
	push @Ax,$elem[1];
	$AxS += $elem[1];
	push @Ay,$elem[2];
	$AyS += $elem[2];
	push @Az,$elem[3];
	$AzS += $elem[3];
	push @G, $elem[4];
	$GS += $elem[4];

	# equal num of each field on each pass (non-sparse) so ok to test just one
	if (scalar(@Ax) == $winsize) {
		$tT = shift(@T);
		$tAx = shift(@Ax);
		$tAy = shift(@Ay);
		$tAz = shift(@Az);
		$tG = shift(@G);
		my $AVGax = $AxS / $winsize; 
		my $AVGay = $AyS / $winsize; 
		my $AVGaz = $AzS / $winsize; 
		my $AVGg = $GS / $winsize; 
		print "DBG:shift: $tT $tAx($AVGax) $tAy($AVGay) $tAz($AVGaz) $tG($AVGg)\n" if ($debug>0);
		print "$tT,$AVGax,$AVGay,$AVGaz,$AVGg\n";
		$AxS -= $tAx;
		$AyS -= $tAy;
		$AzS -= $tAz;
		$GS -= $tG;
	} 
}
exit 0;

##################

sub usage {
	print "$0 [-h] [-w window size]\n";
	print "Feed input CSV file in as pipe.\n";
	print "Output is averaged CSV values using sliding window.\n";
	print "Window size defaults to $winsize, set with -w option.\n";
	print "First CSV field should always be timestamp.\n";
	print "CSV delimiter should be ','\n";
	print "Order of samples should be Ax, Ay, Az, G\n";
	exit 0;
}

