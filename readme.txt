(These are notes about the WM8505 kernel port, please see readme.linux
for the original linux readme.txt.)

* This port is based on kernel 2.6.29 sources released by VIA.

* The sources currently do not include some binary object files (under via_obj.arm_v5t-2629.01) but we're hoping that will change.


******** PREREQUISITES ********

- Appropriate ARM cross-compiler toolchain (I've been using Emdebian's but you can use whatever you want.)


******** CONFIG **********

make menuconfig ARCH=arm


Pre-existing configs are:

* arch/arm/configs/android_wm8505_config
* arch/arm/configs/wm8505_initial_config
* arch/arm/configs/wmt_defconfig

  All came with the source tarball.

* arch/arm/configs/eken_m001_factory_configuration
  
  /proc/config.gz on a factory Eken M001 kernel.


******** BUILDING ********

  make via_obj

(Copies the binary blob .os in to place)

  make ubin modules ARCH=arm CROSS_COMPILE=arm-linux-gnueabi-

This builds a uzImage.bin for uboot. Replace CROSS_COMPILE with your x-compiler as appropriate.

  make modules_install ARCH=arm INSTALL_MOD_PATH=/some/staging/path


******* u-boot script ******

U-boot script saved in 'script/scriptcmd' on SD card, to read a scriptcmd:

  tail -c +72 scriptcmd

To write a scriptcmd:

	mkimage -A arm -O linux -T script -C none -a 1 -e 0 -n "script image" -d cmd scriptcmd

(Where 'cmd' is your source scriptcmd script)


******* kernel arguments ******

Scriptcmd has a line for kernel args:

  setenv bootargs mem=112M root=/dev/mmcblk0p2 lcdid=1 console=ttyS0,115200n

WMT "special" args:

* mem=112M 
  is also mem=109M on non-Android boots. Suspect extra 3M is video memory?!?

* console=ttyS0,115200n 
  gives you kernel console on serial

* lcdid=1 enables VOUT to LCD on tablets. Some other machines
  (netbooks?) use VT1632 which is the default if lcdid is not set (in
  this case, you won't see any video on a tablet.)

