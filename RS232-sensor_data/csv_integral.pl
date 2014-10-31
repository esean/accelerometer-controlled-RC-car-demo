#!/usr/bin/env perl
$| = 1;	# turn on hotflush
#--------------------------------------------------------------------------------
# Make integral of one-column CSV field.
#--------------------------------------------------------------------------------
use strict;
use Getopt::Std;

my $debug = 0;

my %opt;
getopts('dh',\%opt);
&usage if ($opt{'h'});
$debug=1 if ($opt{'d'});

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
my $AxL = 'nil';
my $AyL = 'nil';
my $AzL = 'nil';
my $GL = 'nil';
my $sum = 0;
my $t;

while(my $ln = <>) {
	chomp $ln;
	print "DBG:newLine:ln=$ln\n" if ($debug>0);
	if ($ln =~ /^#/) {
		print "$ln\n";
		next;
	}
	my (@elem) = split(/,/,$ln);
	$sum += $elem[0];
	print "ELEM=@elem:sum=$sum\n" if ($debug>0);
	$t = $sum / 200;
	print "$t\n";
	next;
}
exit 0;

##################

sub usage {
	print "$0 [-h] [-w window size]\n";
	print "Feed input CSV file in as pipe.\n";
	print "Output is integral of one-column CSV file.\n";
	exit 0;
}

