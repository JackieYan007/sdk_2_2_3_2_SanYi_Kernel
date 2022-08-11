#
# arch/arm64/boot/Makefile
#
# This file is included by the global makefile so that you can add your own
# architecture-specific flags and dependencies.
#
# This file is subject to the terms and conditions of the GNU General Public
# License.  See the file "COPYING" in the main directory of this archive
# for more details.
#
# Copyright (C) 2012, ARM Ltd.
# Author: Will Deacon <will.deacon@arm.com>
#
# Based on the ia64 boot/Makefile.
#

OBJCOPYFLAGS_Image :=-O binary -R .note -R .note.gnu.build-id -R .comment -S

targets := Image Image.bz2 Image.gz Image.lz4 Image.lzma Image.lzo Image.itb #Image_a.raw Image_b.raw

# yi.zhang add itb configure
quiet_cmd_mkitb = MKITB   $@
      cmd_mkitb = mkimage -f $< $@ > /dev/null 2>&1

quiet_cmd_copy_itb = COPY    $@
	  cmd_copy_itb = cp $< $@ $(objtree)

#OBJCOPYFLAGS_Image_a.raw := -b 0 -i 4 --interleave-width 2 -I binary
#OBJCOPYFLAGS_Image_b.raw := -b 2 -i 4 --interleave-width 2 -I binary



ifeq ($(CONFIG_ARCH_BSTA1000A),y)
$(obj)/Image.itb: $(srctree)/kernel_a1000.its FORCE
	$(call cmd,mkitb)
	$(call cmd,copy_itb)
else ifeq ($(CONFIG_ARCH_BSTA1000B),y)
$(obj)/Image.itb: $(srctree)/kernel_a1000b.its FORCE
	$(call cmd,mkitb)
	$(call cmd,copy_itb)
endif


#$(obj)/Image_a.raw: $(obj)/Image.itb FORCE
#	$(call if_changed,objcopy)
#	$(call if_changed,copy_itb)

#$(obj)/Image_b.raw: $(obj)/Image.itb FORCE
#	$(call if_changed,objcopy)
#	$(call if_changed,copy_itb)

$(obj)/Image: vmlinux FORCE
	$(call if_changed,objcopy)

$(obj)/Image.bz2: $(obj)/Image FORCE
	$(call if_changed,bzip2)

$(obj)/Image.gz: $(obj)/Image FORCE
	$(call if_changed,gzip)

$(obj)/Image.lz4: $(obj)/Image FORCE
	$(call if_changed,lz4)

$(obj)/Image.lzma: $(obj)/Image FORCE
	$(call if_changed,lzma)

$(obj)/Image.lzo: $(obj)/Image FORCE
	$(call if_changed,lzo)

install:
	$(CONFIG_SHELL) $(srctree)/$(src)/install.sh $(KERNELRELEASE) \
	$(obj)/Image System.map "$(INSTALL_PATH)"

zinstall:
	$(CONFIG_SHELL) $(srctree)/$(src)/install.sh $(KERNELRELEASE) \
	$(obj)/Image.gz System.map "$(INSTALL_PATH)"
