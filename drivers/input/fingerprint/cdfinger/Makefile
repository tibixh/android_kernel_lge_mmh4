#
# Makefile for cdfinger drivers.
#
ccflags-$(CONFIG_CDFINGER_FINGERPRINT_ID_HI) += -DSUPPORT_FP_ID_DUAL -DFP_ID_VALUE=1
ccflags-$(CONFIG_CDFIGNER_FINGERPRINT_ID_LO) += -DSUPPORT_FP_ID_DUAL -DFP_ID_VALUE=0

obj-y += cdfinger-mtk-tee.o
