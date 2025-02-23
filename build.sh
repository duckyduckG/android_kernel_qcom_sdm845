#Creating "Out" Dir
mkdir -p out

#exportpath
export PATH=$HOME/toolchains/zyc/bin:$PATH

#defconfigs
#DEFCONFIG=vendor/sdm845-d8g_defconfig
DEFCONFIG=vendor/xiaomi/mi845_defconfig
FG_DEFCON=vendor/xiaomi/$1.config
#\\ for using wth device specific config with main defconfig for common devices like sdm660

# EnvSetup
KBUILD_BUILD_USER="ducky"
KBUILD_BUILD_HOST=lotsofduck
export KBUILD_BUILD_HOST KBUILD_BUILD_USER

# procs \\ setting cpu cores for compilatioon
PROCS=$(nproc --all)
export PROCS

#create defconfig \\ CLANG_TRIPLE=aarch64-linux-gnu-
make ARCH=arm64 CC=clang CROSS_COMPILE=aarch64-linux-gnu- CROSS_COMPILE_ARM32=arm-linux-gnueabi- O=out $DEFCONFIG $FG_DEFCON CC="ccache clang"

#for 4.4.x kernels \\older
# disable "-Werror" flag which makes all warnings into errors
#sed -i 's/CONFIG_CC_WERROR=y/CONFIG_CC_WERROR=n/' out/.config

# remove "-Werror" flag for "qcacld" driver
#sed -i '/-Werror/d' drivers/staging/qcacld-3.0/Kbuild

#compiling \\ CLANG_TRIPLE=aarch64-linux-gnu-
# CONFIG_DEBUG_SECTION_MISMATCH=y // add after make to see modpost: Found 2 section mismatch(es).
make ARCH=arm64 CC=clang CROSS_COMPILE=aarch64-linux-gnu- CROSS_COMPILE_ARM32=arm-linux-gnueabi- O=out CC="ccache clang" -j"$PROCS"
