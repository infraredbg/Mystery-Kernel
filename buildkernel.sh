#!/bin/bash

# Get kernel configuration
if [ -f mystery_kernel.conf ]
  then
    source "mystery_kernel.conf"
  else
	echo "Kernel configuration file (mystery_kernel.conf) does not exist!"
	exit -1
fi

# Get available CPU cores
CORES = getconf _NPROCESSORS_ONLN

export PATH=$PATH:$TOOLCHAIN_PATH
export CROSS_COMPILE=arm-eabi-

make -C $PWD O=$PWD/out ARCH=arm x510_defconfig
make -j$CORES
