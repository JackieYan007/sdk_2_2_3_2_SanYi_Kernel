#! /usr/bin/perl

my $in_txt = shift;

open(FH_OUT, ">SnrCfgParam.bin") or die;

binmode(FH_OUT);
my $RegCnt = 0;

my $RegAddr = 0;
my $RegOffset = 0;
my $RegDescrip;
my $RegType;
my $RegVal = 0;

open(FH_IN, "<$in_txt") or die "Error: Cannot open $in_txt!\n";
while(<FH_IN>) {
  @regstr = split(' ', $_);
  if($regstr[0] =~ m/(64)/){
    $regstr[2] = substr $regstr[2], 0, 8;
    $RegVal = hex($regstr[2]);
    print (FH_OUT pack("I", $RegVal));##u32
  }    
}
close(FH_IN);
close(FH_OUT);

