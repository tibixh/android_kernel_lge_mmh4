#!/bin/bash

DEFCONFIG_FILE=$1

if [ -z "$DEFCONFIG_FILE" ]; then
    echo "Need defconfig file(xxx_defconfig)!"
    exit -1
fi

if [ ! -e arch/arm/configs/$DEFCONFIG_FILE ]; then
    if [ ! -e arch/arm64/configs/$DEFCONFIG_FILE ]; then
        echo "No such file : $DEFCONFIG_FILE"
        exit -1
    else
        DEFCONFIG_ARCH=arm64
        DEFCONFIG_CROSS_COMPILE=aarch64-linux-gnu-
    fi
else
    DEFCONFIG_ARCH=arm
    DEFCONFIG_CROSS_COMPILE=arm-eabi-
fi

# make .config
env KCONFIG_NOTIMESTAMP=true \
make ARCH=${DEFCONFIG_ARCH} CROSS_COMPILE=${DEFCONFIG_CROSS_COMPILE} ${DEFCONFIG_FILE}

# run menuconfig
env KCONFIG_NOTIMESTAMP=true \
make menuconfig ARCH=${DEFCONFIG_ARCH}

make savedefconfig ARCH=${DEFCONFIG_ARCH}
# copy .config to defconfig
mv defconfig arch/${DEFCONFIG_ARCH}/configs/${DEFCONFIG_FILE}
# clean kernel object
make mrproper
