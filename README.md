# Breaking isolation of Jailhouse with DMA and I/O passthrough
This repository contains a demo and the source code of a DMA attack on the hypervisor Jailhouse on a platform without IOMMU (IMX8QMEVK).  The attacking VM has directly assigned an SD Host Controller (I/O passthrough), which allows it to access an SD card. Through the DMA-capabilities of the SD Host controller and a modified SDHCI driver, the attacking VM is able to read or write to any memory location and bypass the hypervisor.  The driver allows low-level control of ADMA descriptor tables to perform DMA operations. 

The code is given as is and serves simply as a demonstration of a straightforward attack in a virtualized system. For this reason, this modified driver is neither optimized nor developed to run on different environment (e.g. ADMA 64-bit).

**Demonstration of the attack allowing a VM to dump and patch hypervisor code and data through DMA of the SD host controller:
*


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
	BUILD_SYS            = "x86_64-linux"
	NATIVELSBSTRING      = "universal"
	TARGET_SYS           = "aarch64-poky-linux"
	MACHINE              = "imx8mqevk"
	DISTRO               = "fsl-imx-wayland"
	DISTRO_VERSION       = "6.1-mickledore"
	TUNE_FEATURES        = "aarch64 armv8a crc crypto"

