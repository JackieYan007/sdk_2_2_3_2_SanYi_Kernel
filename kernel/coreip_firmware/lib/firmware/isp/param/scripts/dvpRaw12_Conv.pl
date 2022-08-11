my ($inFPGA, $width) = @ARGV;
die "Missing input file name.\n" unless $inFPGA;
die "Missing width.\n" unless $width;
my $byteCount = 0;
open(INFPGA, "< $inFPGA") or die "cannot open input file $inFPGA\n";
open(OUT_RAW_TXT, "> $inFPGA.raw.txt") or die "can't create file\n";
open(OUT_RAW, "> $inFPGA.raw") or die "can't create file\n";
binmode(INFPGA);
binmode(OUT_RAW_TXT);
binmode(OUT_RAW);
my $data0 = 0;
my $data1 = 0;
my $data2 = 0;
my $data3 = 0;
my @data = (0,0,0,0,0,0,0,0,0,0);
seek(INFPGA, 0, SEEK_SET);

my $index = 0;
while (read(INFPGA, $data0, 4)) {
  read(INFPGA, $data1, 4);
  read(INFPGA, $data2, 4);
  read(INFPGA, $data3, 4);
  $data0 = hex(unpack("H*", $data0));
  $data1 = hex(unpack("H*", $data1));
  $data2 = hex(unpack("H*", $data2));
  $data3 = hex(unpack("H*", $data3));
  $data[0] = (($data3&0xFF)<<4)+(($data3&0xF000)>>12);
  $data[1] = ($data3&0xF00)+(($data3&0xFF0000)>>16);
  $data[2] = (($data3&0xFF000000)>>20)+(($data2&0xF0)>>4);
  $data[3] = (($data2&0xF)<<8)+(($data2&0xFF00)>>8);
  $data[4] = (($data2&0xFF0000)>>12)+(($data2&0xF0000000)>>28);
  $data[5] = (($data2&0xF000000)>>16)+($data1&0xFF);
  $data[6] = (($data1&0xFF00)>>4)+(($data1&0xF00000)>>20);
  $data[7] = (($data1&0xF0000)>>8)+(($data1&0xFF000000)>>24);
  $data[8] = (($data0&0xFF)<<4)+(($data0&0xF000)>>12);
  $data[9] = ($data0&0xF00)+(($data0&0xFF0000)>>16);

#  print "$data0, $data1, $data2, $data3\n";
#  print "0:$data[0], 1:$data[1], 2:$data[2], 3:$data[3]\n";
#  print "4:$data[4], 5:$data[5], 6:$data[6], 7:$data[7]\n";
#  print "8:$data[8], 9:$data[9]\n";
  for($i=0;$i<10;$i=$i+2) {
    $data[$i] = $data[$i]<<4;
	$data[$i+1] = $data[$i+1]<<4;
    printf(OUT_RAW_TXT "%04x\n", $data[$i]);
    printf(OUT_RAW_TXT "%04x\n", $data[$i+1]);
	print OUT_RAW pack("C", ($data[$i]>>8));
	print OUT_RAW pack("C", ($data[$i]&0xFF));
	print OUT_RAW pack("C", ($data[$i+1]>>8));
	print OUT_RAW pack("C", ($data[$i+1]&0xFF));
  }
  $byteCount += 10;
}
close(INFPGA);
close(OUT_RAW_TXT);
binmode(OUT_RAW);

my $line_num = int($byteCount/($width));
print "Number of bytes converted = $byteCount, line num=$line_num\n";
print "Check data cmd:\n  ffplay -f rawvideo -s 1920x1080 -pix_fmt bayer_grbg16be $inFPGA.raw\n";
exit;