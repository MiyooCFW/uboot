# U-BOOT MiyooCFW

This custom u-boot version has applied patch:  
- https://github.com/aodzip/buildroot-tiny200/blob/master/board/allwinner/suniv-f1c100s/patch/u-boot/0001-v2020.07.11.patch

Other significant changes are from (starting from most recent):  
- tiopex: https://github.com/MiyooCFW/uboot/commit/e58568e0e  
- gameblabla: https://github.com/bittboy/uboot/commit/ad2bbd7
- steward-fu: https://github.com/MiyooCFW/uboot/commit/67e0b15  

## Build instructions:
Usually you have to be the root user.

- grab source & cd:
```
git clone https://github.com/MiyooCFW/uboot
cd uboot
```
- set environment variables for e.g.:
```
export PATH=$PATH:/opt/miyoo/bin
export ARCH=arm
export CROSS_COMPILE=arm-miyoo-linux-uclibcgnueabi-
```
- write configuration
```
make miyoo_defconfig
```
- build
```
make
```
- edit configuration if needed & rebuild
```
make menuconfig
make clean
make
```
- grab output u-boot binary & move to ``./dist``:
```
mkdir -p dist
mv dist/u-boot-sunxi-with-spl.bin
```

### Compile speed:
If you have a multicore CPU, you can increase build speed with:
```
make -j ${YOUR_CPU_COUNT}
```
---
# _U-BOOT v2020.07_ (docs: https://u-boot.readthedocs.io/en/v2020.07/)
---
