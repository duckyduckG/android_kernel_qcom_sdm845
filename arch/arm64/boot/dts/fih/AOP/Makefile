ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
	dtbo-$(CONFIG_ARCH_SDM845) += \
		sdm845-v2-mtp-overlay_aop-evb.dtbo \
		sdm845-v2-mtp-overlay_aop-pre-evt.dtbo \
		sdm845-v2-mtp-overlay_aop-evt.dtbo \
		sdm845-v2-mtp-overlay_aop-evt1-5.dtbo \
		sdm845-v2-mtp-overlay_aop-evt2_row.dtbo \
		sdm845-v2-mtp-overlay_aop-evt2_gcr.dtbo \
		sdm845-v2-mtp-overlay_aop-evt3.dtbo \
		sdm845-v2-mtp-overlay_aop-evt4_row.dtbo \
		sdm845-v2-mtp-overlay_aop-evt4_gcr.dtbo \
		sdm845-v2-mtp-overlay_aop-dvt_ros.dtbo \
		sdm845-v2-mtp-overlay_aop-dvt_row.dtbo \
		sdm845-v2-mtp-overlay_aop-dvt_gcr.dtbo \
		sdm845-v2-mtp-overlay_aop-pvt_ros.dtbo \
		sdm845-v2-mtp-overlay_aop-pvt_row.dtbo \
		sdm845-v2-mtp-overlay_aop-pvt_gcr.dtbo \
		sdm845-v2-mtp-overlay_aop-pvt2_row.dtbo \
		sdm845-v2-mtp-overlay_aop-pvt2_gcr.dtbo \
		sdm845-v2-mtp-overlay_aol-evt2_row.dtbo \
		sdm845-v2-mtp-overlay_aol-evt2_gcr.dtbo

sdm845-v2-mtp-overlay_aop-evb.dtbo-base := sdm845-v2_fih.dtb
sdm845-v2-mtp-overlay_aop-pre-evt.dtbo-base := sdm845-v2_fih.dtb
sdm845-v2-mtp-overlay_aop-evt.dtbo-base := sdm845-v2_fih.dtb
sdm845-v2-mtp-overlay_aop-evt1-5.dtbo-base := sdm845-v2_fih.dtb
sdm845-v2-mtp-overlay_aop-evt2_row.dtbo-base := sdm845-v2_fih.dtb
sdm845-v2-mtp-overlay_aop-evt2_gcr.dtbo-base := sdm845-v2_fih.dtb
sdm845-v2-mtp-overlay_aop-evt3.dtbo-base := sdm845-v2_fih.dtb
sdm845-v2-mtp-overlay_aop-evt4_row.dtbo-base := sdm845-v2_fih.dtb
sdm845-v2-mtp-overlay_aop-evt4_gcr.dtbo-base := sdm845-v2_fih.dtb
sdm845-v2-mtp-overlay_aop-dvt_ros.dtbo-base := sdm845-v2_fih.dtb
sdm845-v2-mtp-overlay_aop-dvt_row.dtbo-base := sdm845-v2_fih.dtb
sdm845-v2-mtp-overlay_aop-dvt_gcr.dtbo-base := sdm845-v2_fih.dtb
sdm845-v2-mtp-overlay_aop-pvt_ros.dtbo-base := sdm845-v2_fih.dtb
sdm845-v2-mtp-overlay_aop-pvt_row.dtbo-base := sdm845-v2_fih.dtb
sdm845-v2-mtp-overlay_aop-pvt_gcr.dtbo-base := sdm845-v2_fih.dtb
sdm845-v2-mtp-overlay_aop-pvt2_row.dtbo-base := sdm845-v2_fih.dtb
sdm845-v2-mtp-overlay_aop-pvt2_gcr.dtbo-base := sdm845-v2_fih.dtb
sdm845-v2-mtp-overlay_aol-evt2_row.dtbo-base := sdm845-v2_fih.dtb
sdm845-v2-mtp-overlay_aol-evt2_gcr.dtbo-base := sdm845-v2_fih.dtb
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
