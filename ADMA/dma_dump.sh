#!/bin/bash
if [ -z "$1" ] ||  [ -z "$2" ]
then
        echo -e "Please provide arguments: \n"
	echo -e "Usage ./dma_dump.sh ADDRESS SIZE\n"
	echo "ADDRESS: RAM address from which memory must be dumped"
	echo -e "SIZE: Size of memory to dump (in KB)\n"
	echo "Example to dump 10 KB of RAM at adress 0xFDC0E000: ./dma_dump.sh \"\x00\xe0\xc0\xfd\" 10"
        exit
fi
dmesg -n 1
echo "SDHC: Dumping $2 KB of RAM at address $3 by DMA..."
mount /dev/mmcblk2p2 /mnt/ && ls /mnt/
echo -n -e $1  > /sys/devices/platform/soc\@0/soc\@0\:bus\@30800000/30b50000.mmc/adma_read && dd if=/dev/random of=/mnt/out bs=1K count=$2 && cat /sys/devices/platform/soc\@0/soc\@0\:bus\@30800000/30b50000.mmc/adma_rst_addr && umount /mnt/ && mount /dev/mmcblk2p2 /mnt/ && hexdump -C /mnt/out
echo -e	"Memory dumped into /mnt/out \n"
