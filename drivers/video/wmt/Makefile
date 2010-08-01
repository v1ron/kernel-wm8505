#
# Makefile for the Wonder Media framebuffer driver
#

obj-$(CONFIG_FB_WMT) += vout.o vpp.o govrh.o govw.o govm.o scl.o vppm.o parse-edid.o bus.o
obj-$(CONFIG_FB_WMT) += disp.o disp-attr.o vpu.o
obj-$(CONFIG_FB_WMT) += ad9389.o vt1632.o
obj-$(CONFIG_HDMI_CAT6610_WMT) += cat6612.o cat6610/
obj-$(CONFIG_LCD_WMT) += lcd.o lcd-oem.o lcd-INNOLUX-AT070TN83.o lcd-CHILIN-LW700at9003.o lcd-AUO-A080SN01.o lcd-a070vw04.o lcd-M101NWT1.o lcd-at080tn52.o lcd-KD101N1-24NAV1.o lcd-CHIHSIN-LW700AT9005.o
obj-$(CONFIG_FB_WMT) += dev-vpp.o
obj-$(CONFIG_FB_WMT_GE) += gefb.o
obj-$(CONFIG_FB_WMT) += fb-vpu.o
obj-$(CONFIG_VT9043A1) += vout-wm8510.o
obj-$(CONFIG_WM0001) += vout-wm8435.o ad7393.o
#obj-$(CONFIG_FB_WMT_VID)		+= vid.o cmos.o
#obj-$(CONFIG_VIDDEV_AD7180)		+= ad7180.o

gefb-objs := ge_main.o ge_accel.o ge_regs.o
