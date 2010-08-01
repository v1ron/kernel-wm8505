
Æô¶¯²ÎÊý£º


# for VGA and NFS, if use LCD, add "lcdid=1"
setenv bootargs mem=106M root=/dev/nfs rw nfsroot=10.1.8.9:/home/xxx ip=dhcp console=ttyS0,115200n8 noinitrd  init=/init androidboot.console=ttyS0

# for Nand and LCD
setenv bootargs mem=106M root=/dev/mmcblk0p1 rw noinitrd console=ttyS0,115200n8 init=/init androidboot.console=ttyS0 lcdid=1

