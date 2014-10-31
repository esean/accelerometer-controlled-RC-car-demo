#!/usr/bin/env perl
$| = 1;	# turn on hotflush
#--------------------------------------------------------------------------------
# Parse data from serial sensor module.
#
# NEW WAY:
#-------------
# Just send in file with binary data from 'watch -c [file]' logger. The tail
# and od have been removed, it's all done in this script.
#
# OLD WAY:
#-------------
# data from sensor stick looks like this (after 'tail -f [fn] | od -Ax -txC'):
# 0006a90    02  09  41  f4  e7  46  47  02  09  41  f6  e7  45  41  f5  e5
# 0006aa0    47  47  02  08  41  f5  e9  3d  47  02  0a  41  f4  e6  4b  41
# 0006ab0    f6  e4  47  47  02  09  41  f5  e8  42  47  02  08  41  f5  e6
#
# data from sensor stick looks like this (after 'tail -f [fn] | od -Ax -tdC'):
# 0006a90     2   9  65 -12 -25  70  71   2   9  65 -10 -25  69  65 -11 -27
# 0006aa0    71  71   2   8  65 -11 -23  61  71   2  10  65 -12 -26  75  65
# 0006ab0   -10 -28  71  71   2   9  65 -11 -24  66  71   2   8  65 -11 -26
# 0006ac0    69  65 -12 -27  68  71   2   9  65 -10 -27  69  71   2   8  65
#
# 65(0x41) is 'A'
# 71(0x47) is 'G'
#
# Data format:
#   A [X] [Y] [Z] G [H] [L]...
#
# Because of sample timing, it does not go A->G->A->G->A, but instead
# is more like A->G->A->A->G->... 
#
# Timestamp is like:
#	8ms	A
#	12ms	G
#	16ms	A
#	24ms	A & G	
#	...
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
my @G = ();
my $in_A;
my $in_G;
my $sync = 0;
my @sync_pattern = (41,0,0,0,47,0,0,41,0,0,0,41,0,0,0,47,0,0);	# hex
#my @sync_pattern = (65,0,0,0,71,0,0,65,0,0,0,65,0,0,0,71,0,0);	# dec
my $first;
my $just_sync;
my $next_is_section;
my $dump_A = 0;
my $dump_G = 0;
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
					print "NEW SYNC: starting ACCEL sample data\n" if ($debug>0);
					print "++++++++++++++++++ ACCEL +++++++++++++++++++\n" if ($debug>0);
					$in_A = 1;
				} elsif ($h eq '47') {
					print "NEW SYNC: starting GYRO sample data\n" if ($debug>0);
					print "++++++++++++++++++ GYRO +++++++++++++++++++\n" if ($debug>0);
					$in_G = 1;
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
				if (scalar(@A) == 3) {
					die "A already has 3 at start";
				}
				push @A,$h;
				if (scalar(@A) == 3) {
					# dump
					$dump_A = 1;
				}
			} elsif ($in_G == 1) {
				if (scalar(@G) == 2) {
					die "G already has 2 at start";
				}
				push @G,$h;
				if (scalar(@G) == 2) {
					# dump
					$dump_G = 1;
				}
			} else {
				die "No in_A or in_G set in parse!";
			}
			my @dline = ();
			if ($dump_A == 1) {
				print "A:@A\n" if ($debug>0);
				my $yt;
				my $ct2 = 0;
				foreach my $d (@A) {
					$yt = hex($d);
					# convert two's compl.
					if ($yt & 0x80) {
						$yt = -1 * (($yt ^ 0xff)+1);
					}
					push @dline,$yt;
					if ($ct2==0) {
						$lastx = $yt;
					} elsif ($ct2==1) {
						$lasty = $yt;
					} elsif ($ct2==2) {
						$lastz = $yt;
					} else {
						die "accel last stuff incorrect!";
					}
					$ct2++;
				}
				@A = ();
				$in_A = 0;
				$just_sync = 1;	# will re-detect new section at top
			} else {
				# csv file blanks
				push @dline,$lastx;
				push @dline,$lasty;
				push @dline,$lastz;
			}
			if ($dump_G == 1) {
				print "G:@G\n" if ($debug>0);
				my ($y1,$y2);
				####### OLD WAY ##########
				### used to send out 10bit signed (or at
				### least I thought it was!)
				### Now a 16bit signed is xmitted.
				####### OLD WAY ##########
				###$y1 = hex($G[0]);
				###$y2 = hex($G[1]);
				###my $d = 0x03ff & (($y1<<8) | $y2);
				###if ($d & 0x200) {
				###	$d = -1 * (($d ^ 0x3ff)+1);
				###}
				####### OLD WAY ##########
				$y1 = hex($G[0]);
				$y2 = hex($G[1]);
				my $d = ($y1<<8) | $y2;
				if ($d & 0x8000) {
					$d = -1 * (($d ^ 0xffff)+1);
				}
				push @dline,$d;
				$lastg = $d;
				@G = ();
				$in_G = 0;
				$just_sync = 1; # redetect at top
			} else {	
				# csv blanks
				push @dline,$lastg;
			}
			if ($dump_G || $dump_A) {
				if ($first_dump == 1) {
					$first_dump = 0;
					if ($format == 0) {
						# plotit.pl
						print "# timestamp:accelX_curr:accelY_curr:accelZ_curr:gyro_curr\n";
					} else {
						# csv
						print "# timestamp,accelX_curr,accelY_curr,accelZ_curr,gyro_curr\n";
					}
				}
				my $u = $ts / 1000.0;
				# in some cases we can lose sync (232 garbage) and right before we find out we lost sync
				# we parse the garbage and print out a line of junk. so always print the last meas. and
				# save the current for the next loop. thus if we lose sync we won't print junk!
				if ($u_last eq 'nil') {
					print ">>>>>>>>>>>> NO dline TO PRINT, WAIT FOR NEXT SAMPLE\n" if ($debug>0);
				} else {
					if ($format == 0) {
						# plotit.pl
						print $u_last . " " . join(":",@dline_last) . "\n";
					} else {
						# csv
						print $u_last . "," . join(",",@dline_last) . "\n";
					}
				} 
				print ">>>>>++++>>>> NEXT PRINT = $u @dline\n" if ($debug>0);
				$u_last = $u;
				@dline_last = @dline;
				$dump_A = 0;
				$dump_G = 0;
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

