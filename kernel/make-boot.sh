#!/bin/sh

if [ "$(id -u)" -ne 0 ]; then
    echo ""
    echo "This script must be run as root."
    exit 1
fi

if [ -e bootfs ]
then
    rm -rf bootfs
fi

if [ ! -f ./boot_src.img ]; then
    echo "./boot_src.img not found!"
    exit 1
fi

DIST_FILE=./boot.img
rm -f $DIST_FILE

mkdir bootfs
simg2img ./boot_src.img boot.img.ext4
mount -t ext4 -o loop boot.img.ext4 ./bootfs
if [ -n "$SUDO_USER" ]; then
    chown $SUDO_USER -R ./bootfs
fi

cp -fr build/arch/arm64/boot/dts/bst/*.dtb ./bootfs
cp -fr build/Image.itb ./bootfs
make_ext4fs -l 128M -s $DIST_FILE ./bootfs
if [ -n "$SUDO_USER" ]; then
    sudo umount ./bootfs
else
    umount ./bootfs
fi
rm -rf boot.img.ext4 ./bootfs
if [ -n "$SUDO_USER" ]; then
    sudo chown -R $SUDO_USER:`id $SUDO_USER -gn` $DIST_FILE
fi

# mv boot.img /tmp/
echo ""
echo "Done. The new kernel file \"boot.img\" is in ./"

