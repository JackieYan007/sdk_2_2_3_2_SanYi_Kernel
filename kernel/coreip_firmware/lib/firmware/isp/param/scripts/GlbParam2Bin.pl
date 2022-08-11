#! /usr/bin/perl

$in_txt = shift;

open(FH_OUT, ">GlbCfgParam.bin") or die;

binmode(FH_OUT);
my $RegCnt = 0;

my $RegAddr = 0;
my $RegOffset = 0;
my $RegDescrip;
my $RegType;
my $RegVal = 0;

##fill 4 line data, first line is used for regcnt
##seek(FH_OUT, 8*4, SEEK_SET);
open(FH_IN, $in_txt) or die "Error: Cannot open $in_txt!\n";
while(<FH_IN>) {
  @regstr = split(',', $_);
  if(@regstr == 6) {
##  print $regstr[0];
##  print ":$regstr[4]";
##  print "\n";
    $RegAddr = hex($regstr[0]);
    $RegOffset = hex($regstr[1]);
    $RegType = $regstr[3];
    seek(FH_OUT, $RegOffset, SEEK_SET);
    if($regstr[4] =~ m/(0x)/){
      $regstr[4] = substr $regstr[4], 2, 8;
      $RegVal = hex($regstr[4]);
    }else{
      $RegVal=$regstr[4];
    }
    if($RegType =~ m/(u32)/) {
      print (FH_OUT pack("I", $RegVal));##u32
    }elsif($RegType =~ m/(u16)/) {
      print (FH_OUT pack("S", $RegVal));##u16
    }elsif($RegType =~ m/(u08)/) {
      print (FH_OUT pack("C", $RegVal));##u08
    }elsif($RegType =~ m/(s32)/) {
      print (FH_OUT pack("i", $RegVal));##i32
    }elsif($RegType =~ m/(s16)/) {
      print (FH_OUT pack("s", $RegVal));##i16
    }elsif($RegType =~ m/(s08)/) {
      print (FH_OUT pack("c", $RegVal));##i08
    }
  }
}
close(FH_IN);
close(FH_OUT);

