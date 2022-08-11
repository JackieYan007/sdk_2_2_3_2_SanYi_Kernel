my ($inFPGA,$frameW,$frameH,$frameNum) = @ARGV;
die "Missing input file name.\n" unless $inFPGA;
die "Missing frame width.\n" unless $frameW;
die "Missing frame height.\n" unless $frameH;

my $pixW = 12;
my $oneFrameByteCnt = $frameW*$frameH*$pixW/8;

open(INFPGA, "< $inFPGA") or die "cannot open input file $inFPGA\n";
open(OUT_RAW_TXT, "> $inFPGA.raw.txt") or die "can't create file\n";
open(OUT_RAW, "> $inFPGA.raw") or die "can't create file\n";
binmode(INFPGA);
binmode(OUT_RAW_TXT);
binmode(OUT_RAW);
my @dumpd;
my @data;
seek(INFPGA, $oneFrameByteCnt*$frameNum, SEEK_SET);

my $byteCnt = 0;
while ($byteCnt < $oneFrameByteCnt) {
  $index = 0;
  while ($index < 16*3) {
    read(INFPGA, $dumpd[$index], 1);
	$dumpd[$index] = hex(unpack("H*", $dumpd[$index]));
	$index = $index + 1;
  }
  
  ## patch for pixel 10 and pixel 21
  $i=10;
  $subIndex = ($i*3)%2;
  $mainIndex = int($i*3/2);
  #printf("before %04x, %02x, %02x\n", $i, $dumpd[$mainIndex], $dumpd[$mainIndex+1]);
  if($subIndex != 0){
    $patchData = (($dumpd[$mainIndex]&0xF0)<<4) + ($dumpd[$mainIndex+1]);
	$dumpd[$mainIndex] = ($dumpd[$mainIndex]&0x0F) + (($patchData&0x0F)<<4);
	$dumpd[$mainIndex+1] = $patchData>>4;
  } else {
    $patchData = ($dumpd[$mainIndex]<<4) + ($dumpd[$mainIndex+1]&0x0F);
    $dumpd[$mainIndex] = $patchData&0xFF;
	$dumpd[$mainIndex+1] = ($dumpd[$mainIndex+1]&0xF0) + ($patchData>>8);
  }
  #printf("after %04x, %02x, %02x, %04x\n", $i, $dumpd[$mainIndex], $dumpd[$mainIndex+1], $patchData);
  
  $i = 21;
  $subIndex = ($i*3)%2;
  $mainIndex = int($i*3/2);
  #printf("before %04x, %02x, %02x\n", $i, $dumpd[$mainIndex], $dumpd[$mainIndex+1]);
  if($subIndex != 0){
    $patchData = (($dumpd[$mainIndex]&0xF0)<<4) + ($dumpd[$mainIndex+1]);
	$dumpd[$mainIndex] = ($dumpd[$mainIndex]&0x0F) + (($patchData&0x0F)<<4);
	$dumpd[$mainIndex+1] = $patchData>>4;
  } else {
    $patchData = ($dumpd[$mainIndex]<<4) + ($dumpd[$mainIndex+1]&0x0F);
    $dumpd[$mainIndex] = $patchData&0xFF;
	$dumpd[$mainIndex+1] = ($dumpd[$mainIndex+1]&0xF0) + ($patchData>>8);
  }
  #printf("after %04x, %02x, %02x\n", $i, $dumpd[$mainIndex], $dumpd[$mainIndex+1], $patchData);

  for($i=0;$i<32;$i=$i+1){
	$subIndex = ($i*3)%2;
	$mainIndex = int($i*3/2);
	
	if($subIndex != 0){
	  $data[$i] = ($dumpd[$mainIndex]>>4) + ($dumpd[$mainIndex+1]<<4);
	} else {
	  $data[$i] = $dumpd[$mainIndex] + (($dumpd[$mainIndex+1]&0x0F)<<8);
	}
	#printf("%04x, %01x, %01x, %04x, %02x, %02x\n" ,$i, $subIndex, $mainIndex, $data[$i], $dumpd[$mainIndex], $dumpd[$mainIndex+1]);
  }

#  print "$data0, $data1, $data2, $data3\n";
#  print "0:$data[0], 1:$data[1], 2:$data[2], 3:$data[3]\n";
#  print "4:$data[4], 5:$data[5], 6:$data[6], 7:$data[7]\n";
#  print "8:$data[8], 9:$data[9]\n";
  for($i=0;$i<32;$i=$i+2) {
    $data[$i] = $data[$i]<<4;
	$data[$i+1] = $data[$i+1]<<4;
    printf(OUT_RAW_TXT "%04x\n", $data[$i]);
    printf(OUT_RAW_TXT "%04x\n", $data[$i+1]);
	print OUT_RAW pack("C", ($data[$i]>>8));
	print OUT_RAW pack("C", ($data[$i]&0xFF));
	print OUT_RAW pack("C", ($data[$i+1]>>8));
	print OUT_RAW pack("C", ($data[$i+1]&0xFF));
  }
  $byteCnt += 16*3;
}
close(INFPGA);
close(OUT_RAW_TXT);
binmode(OUT_RAW);

my $line_num = int($byteCnt/($frameW*$pixW/8));
print "Number of bytes converted = $byteCnt, line num=$line_num\n";
print "Check data cmd:\n  ffplay -f rawvideo -s 1920x1080 -pix_fmt bayer_grbg16be $inFPGA.raw\n";
exit;