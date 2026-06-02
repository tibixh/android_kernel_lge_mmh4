#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

#
# Makefile for misc devices that really don't fit anywhere else.
#

MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
ARCH_MTK_PROJECT := $(subst ",,$(CONFIG_ARCH_MTK_PROJECT))

MTK_CUSTOM_PATH  := $(srctree)/drivers/misc/mediatek


subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/main/inc
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/main/common/lc898122af
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/main/common/lc898212xdaf/inc
ifeq ($(CONFIG_MTK_LENS_BU63165AF_SUPPORT), y)
  subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/main/common/bu63165af/inc
else
  subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/main/common/bu63169af/inc
endif
ccflags-y += -I$(srctree)/drivers/misc/mediatek/timer/timesync/

obj-$(CONFIG_MTK_LENS)    += mainaf.o

mainaf-y  += main_lens.o
mainaf-y  += common/dw9763af_1c/DW9763AF_1C.o
mainaf-y  += common/zc535af/ZC535AF.o
mainaf-y  += common/fm50af/FM50AF.o
mainaf-y  += common/fp5510e2af/FP5510E2AF.o
mainaf-y  += common/dw9714af/DW9714AF.o
mainaf-y  += common/gt9772af/GT9772AF.o
mainaf-y  += common/fma02af/FMA02.o
mainaf-y  += common/vv02af/VV02AF.o

mainaf-y  += common/gt9768af/GT9768AF.o
mainaf-y  += common/dw9768vaf/DW9768VAF.o
