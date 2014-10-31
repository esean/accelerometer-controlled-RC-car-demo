#!/usr/bin/env perl
$| = 1;	# turn on hotflush
#--------------------------------------------------------------------------------
# Parse data from serial sensor module.
#
# Data format:
#   + 1 [var] 2 [var] 3 [var]....
#
#--------------------------------------------------------------------------------
use strict;
use Getopt::Std;
use File::Tail;

my $debug = 0;

my $sync = 0;
# now sent as ascii
my @sync_pattern = ('+','1',0,0,'2',0,0,'3',0,0,'4',0,0,'5',0,0,'6',0,0,'7',0,0,'8',0,0,'9',0,0,'A',0,0,'B',0,0,'C',0,0,'D',0,0,'E',0,0,'F',0,0);	# ascii
my $num_of_fields = 16;
my $first;
my $just_sync;
my $next_is_section;
my $first_dump = 1;	# print csv header on first output
my $t;
my $byte_cnt = 0;
my $start = 0;
my $dec;
my $v2;
my @v = ();
my @vlast = ();
my $print_hdr = 0;

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
	$byte_cnt = 0;
	$start = 0;
	foreach my $h (@ln) {

		#
		# if we matched all sync patterns, ok
		#
		if ($sync == scalar(@sync_pattern)) {
			if ($just_sync == 1) {
				$just_sync = 0;
				if ($print_hdr == 0) {
					$print_hdr = 1;
					if ($format == 0) {
						# plotit
						print "# SLOPE_MULTXZ:SLOPE_MULTYZ:AXZ_MOTION_OFF_MIN:AYZ_MOTION_OFF_MIN:GYRO_ROT_MIN_TURN_STEERING:ROT_MULT:GYRO_ROT_HARD_STEER:ROT_MULT_HARD:PWM1_MULT:PWM2_MULT:PR2:PWM1_DYN_THRES:PWM2_DYN_THRES:PWM1_DYN_MULT:PWM1_DYN_MULT\n";
					} else {
						# CSV
						print "# SLOPE_MULTXZ,SLOPE_MULTYZ,AXZ_MOTION_OFF_MIN,AYZ_MOTION_OFF_MIN,GYRO_ROT_MIN_TURN_STEERING,ROT_MULT,GYRO_ROT_HARD_STEER,ROT_MULT_HARD,PWM1_MULT,PWM2_MULT,PR2,PWM1_DYN_THRES,PWM2_DYN_THRES,PWM1_DYN_MULT,PWM1_DYN_MULT\n";
					}
				}
			}
			#print "+++> $byte_cnt:$h\n";
			if (($byte_cnt>1) && ($h eq '+')) {
				print "ERROR: lost sync, found $h\n" if ($debug>0);
				$sync = 0;	# force resync
				next;
			}
			print "X:$byte_cnt:$sync:$h\n" if ($debug>0);
			if ($start > 0) {
				if ($start == 1) {
					# ignore the num label
					$start++;
				} elsif ($start == 2) {
					# high byte
					$v2 = hex($h) << 4;
					#print "$h";
					$start++;
				} else {
					# low byte
					$v2 |= hex($h);
					#print "$h:$v2\n";
					push @v,$v2;
					$start = 1;
				}
			}
			if ($byte_cnt == 1) {
				# 2nd byte is the start of data with '+' label
				$start = 1;
			}
			$byte_cnt++;
			if ($byte_cnt > scalar(@sync_pattern)) {
				print "#-------------------\n" if ($debug>0);
				if (&compare_arrays(\@v,\@vlast) == 1) {
					print "INFO: lines are the same, not printing (| uniq)\n" if ($debug>0);
				} else {
					if (scalar(@v) == ($num_of_fields-1)) {
						print join(",",@v) . "\n";
					} else {
						print "ERROR: this line had " . (scalar(@v)) . " values. should be " . ($num_of_fields-1) . ":@v\n" if ($debug>0);
					}
				}
				@vlast = @v;
				@v = ();
			}
		} else {

			$next_is_section = 0;
			$just_sync = 1;
			$byte_cnt = 0;
			@v = ();

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

sub compare_arrays {
	my ($first, $second) = @_;
	no warnings;  # silence spurious -w undef complaints
	return 0 unless @$first == @$second;
	for (my $i = 0; $i < @$first; $i++) {
		return 0 if $first->[$i] ne $second->[$i];
		}
	return 1;
}
sub usage {
	print "$0 [-h] [-b] [-c] [-f [file]]\n";
	print "Processes a DBG_CONST_ADC_ADJUST dump. This is an ASCII-encoded stream of $num_of_fields variables with labels.\n";
	print "Flags:\n";
	print "   -c - outputs in CSV format, otherwise defaults to plotit.pl formatting\n";
	print "   -b - blocking read of [file], otherwise defaults to non-blocking (for use with plotit.pl)\n";
	exit 0;
}

