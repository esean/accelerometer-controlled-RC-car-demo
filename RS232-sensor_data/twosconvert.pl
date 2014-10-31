#!/usr/bin/env perl
$| = 1;	# turn on hotflush
#--------------------------------------------------------------------------------
# Turn two's complement into signed decimal
#--------------------------------------------------------------------------------
use strict;
use Getopt::Std;
use File::Tail;

my $debug = 0;
my $t;
my $bits = 8;
my $bitf = 0xff;
my $bitsu = 0x80;

#&usage unless ($ARGV[0]);
my %opt;
getopts('dhb:',\%opt);
&usage if ($opt{'h'});
$debug=1 if ($opt{'d'});

if ($opt{'b'}) {
	$bits = $opt{'b'};
	if ($bits == 4) {
		$bitf =  0xf;
		$bitsu = 0x8;
	} elsif ($bits == 8) {
		$bitf =  0xff;
		$bitsu = 0x80;
	} elsif ($bits == 12) {
		$bitf =  0xfff;
		$bitsu = 0x800;
	} elsif ($bits == 16) {
		$bitf =  0xffff;
		$bitsu = 0x8000;
	} elsif ($bits == 20) {
		$bitf =  0xfffff;
		$bitsu = 0x80000;
	} elsif ($bits == 24) {
		$bitf =  0xffffff;
		$bitsu = 0x800000;
	} elsif ($bits == 28) {
		$bitf =  0xfffffff;
		$bitsu = 0x8000000;
	} elsif ($bits == 32) {
		$bitf =  0xffffffff;
		$bitsu = 0x80000000;
	} elsif ($bits == 64) {
		$bitf =  0xffffffffffffffff;
		$bitsu = 0x8000000000000000;
	} else {
		die "Unsupported bit number";
	}
	print "For $bits bits, using $bitf w/ MSB=$bitsu\n" if ($debug>0);
}

while ($t = <>) {
	chomp($t);
	$t =~ s/\r//;
	if ($t =~ /^[^0-9a-fA-F]*([0-9a-fA-F]+)[^0-9a-fA-F]*$/) {
		$t = $1;
	}
	$t = hex($t);
	print "READ:$t\n" if ($debug>0);
	if ($t & $bitsu) {
		$t = -1 * (($t ^ $bitf)+1);
	}
	print "$t\n";
}
exit 0;

##################

sub usage {
	print "$0 [-h] [-b [num bits]]\n";
	print "Converts a two's complement number to signed decimal.\n";
	print "Default bits = $bits\n";
	print "Flags:\n";
	print "   -b - number of bits w/ MSB being sign bit\n";
	exit 0;
}

