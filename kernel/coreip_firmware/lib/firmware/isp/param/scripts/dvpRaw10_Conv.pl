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
my @data = (0,0,0,0,0,0,0,0,0,0,0,0);
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
  $data[0] = (($data3&0xFF)<<2)+(($data3&0xC000)>>14);
  $data[1] = (($data3&0x3F00)>>4)+(($data3&0xF00000)>>20);
  $data[2] = (($data3&0x0F0000)>>10)+(($data3&0xFC000000)>>26);
  $data[3] = (($data3&0x03000000)>>16)+($data2&0xFF);
  $data[4] = (($data2&0xFF00)>>6)+(($data2&0xC00000)>>22);
  $data[5] = (($data2&0x3F0000)>>12)+(($data2&0xF0000000)>>28);
  $data[6] = (($data2&0x0F000000)>>18)+(($data1&0xFC)>>2);
  $data[7] = (($data1&0x03)<<8)+(($data1&0xFF00)>>8);
  $data[8] = (($data1&0xFF0000)>>14)+(($data1&0xC0000000)>>30);
  $data[9] = (($data1&0x3F000000)>>20)+(($data0&0xF0)>>4);
  $data[10] = (($data0&0x0F)<<6)+(($data0&0xFC00)>>10);
  $data[11] = ($data0&0x0300)+(($data0&0xFF0000)>>16);

  #printf ("%x, %x, %x, %x\n", $data0, $data1, $data2, $data3);
  #printf ("0:%x, 1:%x, 2:%x, 3:%x\n", $data[0], $data[1], $data[2], $data[3]);
  #print "4:$data[4], 5:$data[5], 6:$data[6],  7:$data[7]\n";
  #print "8:$data[8], 9:$data[9], 10:$data[10], 11:$data[11]\n";
  #if($byteCount > 24) 
  #{
  #  return;
  #}
  for($i=0;$i<12;$i=$i+2) {
    $data[$i] = $data[$i]<<6;
	$data[$i+1] = $data[$i+1]<<6;
    printf(OUT_RAW_TXT "%04x\n", $data[$i]);
    printf(OUT_RAW_TXT "%04x\n", $data[$i+1]);
	print OUT_RAW pack("C", ($data[$i]>>8));
	print OUT_RAW pack("C", ($data[$i]&0xFF));
	print OUT_RAW pack("C", ($data[$i+1]>>8));
	print OUT_RAW pack("C", ($data[$i+1]&0xFF));
  }
  $byteCount += 12;
}
close(INFPGA);
close(OUT_RAW_TXT);
binmode(OUT_RAW);

my $line_num = int($byteCount/($width));
print "Number of bytes converted = $byteCount, line num=$line_num\n";
print "Check data cmd:\n  ffplay -f rawvideo -s 1920x1080 -pix_fmt bayer_grbg16be $inFPGA.raw\n";
exit;