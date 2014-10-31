#!/usr/bin/env perl
$| = 1;	# turn on hotflush
#--------------------------------------------------------------------------------
# Parse data from serial sensor module.
#
# Data format:
#   A [H] [L] B [H] [L] C [H] [L]
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
my @sync_pattern = (41,0,0,42,0,0,43,0,0);	# hex
#my @sync_pattern = (65,0,0,0,71,0,0,65,0,0,0,65,0,0,0,71,0,0);	# dec
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

my $fn;
if ($blocking == 1) {
	# normal tail open
	$fn = File::Tail->new(name=>$file,maxinterval=>1,interval=>1,tail=>-1);
	while (defined($t=$fn->read)) {
		&proc($t);
	}
} else {
	open F, "<$file";
	while ($t = <F>) {
		chomp $t;
		&proc($t);
	}
}
sub proc {
	my ($t) = @_;
	chomp($t);
	print "DBG:proc():at top:t=$t\n" if ($debug>0);
	my (@ln) = split(//,$t);
	# skip line number of each new line
	$first = 1;
	foreach my $h (@ln) {

		# skip first line number of each line
		if ($first == 1) {
			$first = 0;
			next;
		}

		$h = sprintf("%02X",ord($h));

		#
		# if we matched all sync patterns, ok
		#
		if ($sync == scalar(@sync_pattern)) {
			print "X:$sync:$h\n" if ($debug>0);
			if ($just_sync == 1) {
				# we just did a sync, so next one is a section header
				print "just_sync=1, next\n" if ($debug>0);
				$just_sync = 0;

				# if we just sync'd, this one is a header
				if ($h eq '41') {
					print "++++++++++++++++++ A +++++++++++++++++++\n" if ($debug>0);
					$in_A = 1;
				} elsif ($h eq '42') {
					print "++++++++++++++++++ B +++++++++++++++++++\n" if ($debug>0);
					$in_B = 1;
				} elsif ($h eq '43') {
					print "++++++++++++++++++ C +++++++++++++++++++\n" if ($debug>0);
					$in_C = 1;
				} elsif (($h eq '0D') || ($h eq '0C') || ($h eq '0A')) {
					# we were expecting an 'A' or 'G' but found CR, there are sometimes
					# sent to allow File::Tail to notice the file is changing
					print "+++++<<<<< CR >>>>>>+++++\n" if ($debug>0);
					# this one is out-of-band, so don't add in ts
					$just_sync = 1;	# we need to do this section again
					next;
				} else {
					#print "ERROR: LOST SYNC!! - found h=$h\n" if ($debug>0);
					$sync = 0;	# lost sync, re-sync
					print "!!!!!!!!!!!!!!!!!!!!!!!! LOST SYNC #1 !!!!!!!!!!!!!!!!!!!!!!!!!!!\n" if ($debug>0);
					#$ts = 0;
					$u_last = 'nil';
					#die "Oops, lost our sync! file may be corrupt or was xmitted from module board incorrectly! (DBG:found $h)";
					# never gets here, but if we ever do, 'next' instead of exec'ing below
					next;
				}
				print "TIME INDEX=$tindex adding $tinfo[$tindex] to $ts\n" if ($debug>0);
				$ts += $tinfo[$tindex];
				$tindex++;
				$tindex = 0 if ($tindex >= scalar(@tinfo));
				next;
			}
			if ($in_A) {
				if (scalar(@A) == 2) {
					die "A already has 2 at start";
				}
				push @A,$h;
				if (scalar(@A) == 2) {
					# dump
					$dump_A = 1;
				}
			} elsif ($in_B == 1) {
				if (scalar(@B) == 2) {
					die "B already has 2 at start";
				}
				push @B,$h;
				if (scalar(@B) == 2) {
					# dump
					$dump_B = 1;
				}
			} elsif ($in_C == 1) {
				if (scalar(@C) == 2) {
					die "C already has 2 at start";
				}
				push @C,$h;
				if (scalar(@C) == 2) {
					# dump
					$dump_C = 1;
				}
			} else {
				die "No in_A or in_B, in_C set in parse!";
			}
			my @dline = ();
			if ($dump_A == 1) {
				print "A:@A\n" if ($debug>0);
				my ($y1,$y2);
				$y1 = hex($A[0]);
				$y2 = hex($A[1]);
				my $d = ($y1<<8) | $y2;
				if ($d & 0x8000) {
					$d = -1 * (($d ^ 0xffff)+1);
				}
				push @dline,$d;
				@A = ();
				$in_A = 0;
				$just_sync = 1;	# will re-detect new section at top
			}
			if ($dump_B == 1) {
				print "B:@B\n" if ($debug>0);
				my ($y1,$y2);
				$y1 = hex($B[0]);
				$y2 = hex($B[1]);
				my $d = ($y1<<8) | $y2;
				if ($d & 0x8000) {
					$d = -1 * (($d ^ 0xffff)+1);
				}
				push @dline,$d;
				@B = ();
				$in_B = 0;
				$just_sync = 1;	# will re-detect new section at top
			}
			if ($dump_C == 1) {
				print "C:@C\n" if ($debug>0);
				my ($y1,$y2);
				$y1 = hex($C[0]);
				$y2 = hex($C[1]);
				my $d = ($y1<<8) | $y2;
				if ($d & 0x8000) {
					$d = -1 * (($d ^ 0xffff)+1);
				}
				push @dline,$d;
				@C = ();
				$in_C = 0;
				$just_sync = 1;	# will re-detect new section at top
			}
			if ($dump_A || $dump_B || $dump_C) {
				# in some cases we can lose sync (232 garbage) and right before we find out we lost sync
				# we parse the garbage and print out a line of junk. so always print the last meas. and
				# save the current for the next loop. thus if we lose sync we won't print junk!
				if ($dump_A) {
					printf("\n");
				} 

				if ($format == 0) {
					# plotit.pl
					print "@dline:";
				} else {
					# csv
					print "@dline,";
				}
				$dump_A = 0;
				$dump_B = 0;
				$dump_C = 0;
			}
		} else {

			$next_is_section = 0;
			$just_sync = 1;

			#
			# else we are looking for sync
			#
			print "sync?:is [fn]:$h == sync pattern:$sync_pattern[$sync]?\n" if ($debug>0);
			if ($sync_pattern[$sync] eq '0') {
				# same as '*', any pattern
				print "STAR\n" if ($debug>0);
				$sync++;
			} else {
				if ($h ne $sync_pattern[$sync]) {
					print "RESYNC: looking for pattern, not a match ($h != $sync_pattern[$sync])\n" if ($debug>0);
					$sync = 0;
				} else {
					# match, advance
					print "YES($h)" if ($debug>0);
					$sync++;
				}
			}
		}
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

