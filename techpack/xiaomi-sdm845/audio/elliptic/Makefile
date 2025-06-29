ccflags-y := -I$(src) -Wall -Werror

# Keep these header paths
LINUXINCLUDE += \
    -I$(srctree)/techpack/audio/include/dsp \
    -I$(srctree)/techpack/audio/include/uapi \
    -I$(srctree)/techpack/audio/include \
    -I$(srctree)/techpack/mi845/include/dsp \
    -I$(srctree)/techpack/mi845/include/uapi \
    -I$(srctree)/techpack/mi845/include

IO_MODULE := msm

obj-$(CONFIG_SND_ELLIPTIC) += elliptic_core.o

elliptic_core-objs := \
    apr_elliptic.o \
    elliptic.o \
    elliptic_mixer_controls.o \
    elliptic_sysfs.o \
    io_modules/$(IO_MODULE)/elliptic_data_$(IO_MODULE)_io.o \
    io_modules/userspace/elliptic_data_userspace_io.o \
    io_modules/userspace/elliptic_data_userspace_ctrl.o
