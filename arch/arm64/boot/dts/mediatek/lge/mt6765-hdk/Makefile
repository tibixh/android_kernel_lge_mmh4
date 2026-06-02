ifeq ($(strip $(CONFIG_BUILD_ARM64_DTB_OVERLAY_IMAGE)), y)
LGE_DTS_DIR := $(subst $\",,$(CONFIG_BUILD_ARM64_DTB_OVERLAY_IMAGE_NAMES))/
LGE_DTS_FILES := $(notdir $(basename $(wildcard $(srctree)/arch/arm64/boot/dts/$(LGE_DTS_DIR)*.dts)))
LGE_DTB_FILES := $(addsuffix .dtb, $(LGE_DTS_FILES))
$(info add lge dtbs: $(LGE_DTB_FILES))

dtb-y += $(addsuffix .dtb, $(LGE_DTS_FILES))
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
