ifeq ($(CONFIG_MACH_XIAOMI_SDM845),y)
obj-$(CONFIG_SND_SOC) += audio/
else
ccflags-y := -Wno-unused-function
obj-y := stub.o
endif
