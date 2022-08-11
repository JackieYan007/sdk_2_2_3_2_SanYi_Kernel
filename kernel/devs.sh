#!/bin/bash

#################################### 1
sudo mknod mem     c 1 1;
sudo mknod kmem    c 1 2;
sudo mknod null    c 1 3;
sudo mknod port    c 1 4;
sudo mknod zero    c 1 5;
sudo mknod core    c 1 6;
sudo mknod full    c 1 7;
sudo mknod random  c 1 8;
sudo mknod urandom c 1 9;
sudo mknod aio     c 1 10;
sudo mknod kmsg    c 1 11;
sudo mknod oldmem  c 1 12;

sudo mknod ram0    b 1 0;
sudo mknod ram1    b 1 1;
sudo mknod ram2    b 1 2;
sudo mknod ram3    b 1 3;
sudo mknod ram4    b 1 4;
sudo mknod ram5    b 1 5;
sudo mknod ram6    b 1 6;
sudo mknod ram7    b 1 7;
ln -s ram0 ram;


#################################### 2
sudo mknod ptyp0   c 2 0;
sudo mknod ptyp1   c 2 1;
sudo mknod ptyp2   c 2 2;
sudo mknod ptyp3   c 2 3;
sudo mknod ptyp4   c 2 4;
sudo mknod ptyp5   c 2 5;
sudo mknod ptyp6   c 2 6;
sudo mknod ptyp7   c 2 7;

sudo mknod fd0     b 2 0;
sudo mknod fd1     b 2 1;
sudo mknod fd2     b 2 2;
sudo mknod fd3     b 2 3;
sudo mknod fd4     b 2 4;
sudo mknod fd5     b 2 5;
sudo mknod fd6     b 2 6;
sudo mknod fd7     b 2 7;

#################################### 3
sudo mknod hda     b 3 0;
sudo mknod hda1    b 3 1;
sudo mknod hda2    b 3 2;
sudo mknod hda3    b 3 3;
sudo mknod hda4    b 3 4;
sudo mknod hdb     b 3 64;
sudo mknod hdb1    b 3 65;
sudo mknod hdb2    b 3 66;
sudo mknod hdb3    b 3 67;
sudo mknod hdb4    b 3 68;


#################################### 4
sudo mknod tty0    c 4 0;
sudo mknod tty1    c 4 1;
sudo mknod tty2    c 4 2;
sudo mknod tty3    c 4 3;
sudo mknod tty4    c 4 4;
sudo mknod tty5    c 4 5;
sudo mknod tty6    c 4 6;
sudo mknod tty7    c 4 7;

sudo mknod ttyS0   c 4 64;
sudo mknod ttyS1   c 4 65;
sudo mknod ttyS2   c 4 66;
sudo mknod ttyS3   c 4 67;
sudo mknod ttyS4   c 4 68;
sudo mknod ttyS5   c 4 69;
sudo mknod ttyS6   c 4 70;
sudo mknod ttyS7   c 4 71;

sudo mknod root    b 4 0;

#################################### 5
sudo mknod tty     c 5 0;
sudo mknod console c 5 1;
sudo mknod multiplex c 5 2;
sudo mknod ttyprintk c 5 3;

sudo mknod cua0    c 5 64;
sudo mknod cua1    c 5 65;
sudo mknod cua2    c 5 66;
sudo mknod cua3    c 5 67;
sudo mknod cua4    c 5 68;
sudo mknod cua5    c 5 69;
sudo mknod cua6    c 5 70;
sudo mknod cua7    c 5 71;


#################################### 6
sudo mknod lp0     c 6 0;
sudo mknod lp1     c 6 1;
sudo mknod lp2     c 6 2;
sudo mknod lp3     c 6 3;
sudo mknod lp4     c 6 4;
sudo mknod lp5     c 6 5;
sudo mknod lp7     c 6 6;
sudo mknod lp8     c 6 7;


#################################### 7
sudo mknod vcs     c 7 0;
sudo mknod vcs1    c 7 1;
sudo mknod vcs2    c 7 2;
sudo mknod vcs3    c 7 3;

sudo mknod vcsu    c 7 64;
sudo mknod vcsu1   c 7 65;
sudo mknod vcsu2   c 7 66;
sudo mknod vcsu3   c 7 67;

sudo mknod vcsa    c 7 128;
sudo mknod vcsa1   c 7 129;
sudo mknod vcsa2   c 7 130;
sudo mknod vcsa3   c 7 131;

sudo mknod loop0   b 7 0;
sudo mknod loop1   b 7 1;
sudo mknod loop2   b 7 2;
sudo mknod loop3   b 7 3;
sudo mknod loop4   b 7 4;
sudo mknod loop5   b 7 5;
sudo mknod loop6   b 7 6;
sudo mknod loop7   b 7 7;


#################################### 8
sudo mknod sda     b 8 0;
sudo mknod sda1    b 8 1;
sudo mknod sda2    b 8 2;
sudo mknod sda3    b 8 3;
sudo mknod sda4    b 8 4;
sudo mknod sdb     b 8 64;
sudo mknod sdb1    b 8 65;
sudo mknod sdb2    b 8 66;
sudo mknod sdb3    b 8 67;
sudo mknod sdb4    b 8 68;
sudo mknod sdc     b 8 128;
sudo mknod sdc1    b 8 129;
sudo mknod sdc2    b 8 130;
sudo mknod sdc3    b 8 131;
sudo mknod sdc4    b 8 132;


#################################### 9
sudo mknod st0     c 9 0;
sudo mknod st1     c 9 1;
sudo mknod st2     c 9 2;
sudo mknod st3     c 9 3;
sudo mknod st4     c 9 4;
sudo mknod st5     c 9 5;
sudo mknod st6     c 9 6;
sudo mknod st7     c 9 7;

sudo mknod st0l    c 9 32;
sudo mknod st1l    c 9 33;
sudo mknod st2l    c 9 34;
sudo mknod st3l    c 9 35;
sudo mknod st4l    c 9 36;
sudo mknod st5l    c 9 37;
sudo mknod st6l    c 9 38;
sudo mknod st7l    c 9 39;

sudo mknod st0m    c 9 64;
sudo mknod st1m    c 9 65;
sudo mknod st2m    c 9 66;
sudo mknod st3m    c 9 67;
sudo mknod st4m    c 9 68;
sudo mknod st5m    c 9 69;
sudo mknod st6m    c 9 70;
sudo mknod st7m    c 9 71;

sudo mknod st0a    c 9 96;
sudo mknod st1a    c 9 97;
sudo mknod st2a    c 9 98;
sudo mknod st3a    c 9 99;
sudo mknod st4a    c 9 99;
sudo mknod st5a    c 9 100;
sudo mknod st6a    c 9 101;
sudo mknod st7a    c 9 102;

sudo mknod nst0    c 9 128;
sudo mknod nst1    c 9 129;
sudo mknod nst2    c 9 130;
sudo mknod nst3    c 9 131;
sudo mknod nst4    c 9 132;
sudo mknod nst5    c 9 133;
sudo mknod nst6    c 9 134;
sudo mknod nst7    c 9 135;

sudo mknod nst0l   c 9 160;
sudo mknod nst1l   c 9 161;
sudo mknod nst2l   c 9 162;
sudo mknod nst3l   c 9 163;
sudo mknod nst4l   c 9 164;
sudo mknod nst5l   c 9 165;
sudo mknod nst6l   c 9 166;
sudo mknod nst7l   c 9 167;

sudo mknod nst0m   c 9 192;
sudo mknod nst1m   c 9 193;
sudo mknod nst2m   c 9 194;
sudo mknod nst3m   c 9 195;
sudo mknod nst4m   c 9 196;
sudo mknod nst5m   c 9 197;
sudo mknod nst6m   c 9 198;
sudo mknod nst7m   c 9 199;

sudo mknod nst0a   c 9 224;
sudo mknod nst1a   c 9 225;
sudo mknod nst2a   c 9 226;
sudo mknod nst3a   c 9 227;
sudo mknod nst4a   c 9 228;
sudo mknod nst5a   c 9 229;
sudo mknod nst6a   c 9 230;
sudo mknod nst7a   c 9 231;


sudo mknod md0     b 9 0;
sudo mknod md1     b 9 1;
sudo mknod md2     b 9 2;
sudo mknod md3     b 9 3;
sudo mknod md4     b 9 4;

#################################### 10
sudo mknod beep        c 10 128;
sudo mknod watchdog    c 10 130;
sudo mknod temperature c 11 131;
sudo mknod hwtrap      c 11 133;
sudo mknod apm_bios    c 11 134;
sudo mknod pciconf     c 11 143;
sudo mknod nvram       c 11 144;
sudo mknod graphics    c 11 146;
sudo mknod opengl      c 11 147;
sudo mknod gfx         c 11 148;
sudo mknod led         c 11 151;
sudo mknod userdma     c 11 161;

sudo mkdir net watchdogs input shm;



#################################### 11
sudo mknod kbd     c 11 0;
sudo mknod ttyB0   c 11 0;
sudo mknod ttyB1   c 11 1;
sudo mknod ttyB2   c 11 2;
sudo mknod ttyB3   c 11 3;


#################################### 21
sudo mknod sg0     c 11 0;
sudo mknod sg1     c 11 1;
sudo mknod sg2     c 11 2;
sudo mknod sg3     c 11 3;



#################################### 22
sudo mknod hdc     b 3 0;
sudo mknod hdc1    b 3 1;
sudo mknod hdc2    b 3 2;
sudo mknod hdc3    b 3 3;
sudo mknod hdc4    b 3 4;
sudo mknod hdd     b 3 64;
sudo mknod hdd1    b 3 65;
sudo mknod hdd2    b 3 66;
sudo mknod hdd3    b 3 67;
sudo mknod hdd4    b 3 68;



#################################### 29
sudo mknod fb0     c 29 0;
sudo mknod fb1     c 29 1;
sudo mknod fb2     c 29 2;
sudo mknod fb3     c 29 3;


#################################### 31
sudo mknod mtdblock0   b 31 0;
sudo mknod mtdblock1   b 31 1;
sudo mknod mtdblock2   b 31 2;
sudo mknod mtdblock3   b 31 3;

sudo mknod rrom0   b 31 8;
sudo mknod rrom1   b 31 9;
sudo mknod rrom2   b 31 10;
sudo mknod rrom3   b 31 11;

sudo mknod flash0  b 31 16;
sudo mknod flash1  b 31 17;
sudo mknod flash2  b 31 18;
sudo mknod flash3  b 31 19;



#################################### 36
sudo mknod route   c 36 0;
sudo mknod skip    c 36 1;
sudo mknod fwmonitor c 36 2;
sudo mknod tap0    c 36 16;
sudo mknod tap1    c 36 17;
sudo mknod tap2    c 36 18;
sudo mknod tap3    c 36 19;

#################################### 46
sudo mknod ttyR0   c 46 0;
sudo mknod ttyR1   c 46 1;
sudo mknod ttyR2   c 46 2;
sudo mknod ttyR3   c 46 3;
sudo mknod ttyR4   c 46 4;
sudo mknod ttyR5   c 46 5;
sudo mknod ttyR6   c 46 6;
sudo mknod ttyR7   c 46 7;

#################################### 56
sudo mknod adb     c 56 0;


#################################### 89
sudo mknod i2c-0   c 89 0;
sudo mknod i2c-1   c 89 1;
sudo mknod i2c-2   c 89 2;
sudo mknod i2c-3   c 89 3;


#################################### 90
sudo mknod mtd0   c 89 0;
sudo mknod mtdr0  c 89 1;
sudo mknod mtd1   c 89 2;
sudo mknod mtdr1  c 89 3;
sudo mknod mtd2   c 89 4;
sudo mknod mtdr2  c 89 5;
sudo mknod mtd3   c 89 6;
sudo mknod mtdr3  c 89 7;


#################################### 91
sudo mknod can0   c 91 0;
sudo mknod can1   c 91 1;
sudo mknod can2   c 91 2;
sudo mknod can3   c 91 3;

#################################### 116
sudo mkdir -p dev/snd
sudo mknod dev/snd/pcmC0D0p  c 116 2;
sudo mknod dev/snd/pcmC0D0c  c 116 3;
sudo mknod dev/snd/pcmC0D1p  c 116 4;
sudo mknod dev/snd/pcmC0D1c  c 116 5;
sudo mknod dev/snd/controlC0 c 116 6;
sudo mknod dev/snd/pcmC1D0p  c 116 7;
sudo mknod dev/snd/controlC1 c 116 8;
sudo mknod dev/snd/pcmC2D0c  c 116 9;
sudo mknod dev/snd/controlC2 c 116 10;
sudo mknod dev/snd/timer     c 116 33;

#################################### 153
sudo mknod dev/spidev0.0 c 153 0;
sudo mknod dev/spidev1.0 c 153 1;
sudo mknod dev/qspi c 153 2;

#################################### 179
sudo mknod mmcblk0   b 179 0;
sudo mknod mmcblk0p1 b 179 1;
sudo mknod mmcblk1   b 179 2;
sudo mknod mmcblk1p1 b 179 3;
sudo mknod mmcblk2   b 179 4;
sudo mknod mmcblk2p1 b 179 5;
sudo mknod mmcblk3   b 179 6;
sudo mknod mmcblk3p1 b 179 7;




