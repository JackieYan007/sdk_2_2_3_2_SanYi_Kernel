#ccflags-$(CONFIG_VIDEO_A1000_DEBUG) += -DDEBUG

a1000-isp-objs += \
	isp_core.o isp_video.o isp_sysfile.o isp_fw_loader.o

obj-$(CONFIG_VIDEO_BST_CSI) += csi_dphy.o csi.o csi_phy_b0.o
obj-$(CONFIG_VIDEO_BST_ISP) += a1000-isp.o

subdir-ccflags-y := -Wall -Werror
