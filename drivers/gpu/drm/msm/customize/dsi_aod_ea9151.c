
#define pr_fmt(fmt)	"msm-dsi-panel:[%s:%d] " fmt, __func__, __LINE__
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>

#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>

#include <video/mipi_display.h>
#include "dsi_panel.h"
#include "dsi_ctrl_hw.h"
#include "msm_drv.h"
#include "sde_connector.h"
#include "msm_mmu.h"
#include "dsi_display.h"
#include "dsi_panel.h"
#include "dsi_ctrl.h"
#include "dsi_ctrl_hw.h"
#include "dsi_drm.h"
#include "dsi_clk.h"
#include "dsi_pwr.h"
#include "sde_dbg.h"

#define AOD_10NITS 0x40B1
#define AOD_35NITS 0xCCB4
#define AOD_100NITS 0xFCBF
#define ENABLE_AOD_BRANCH 0x02
#define LGD_EA9151_BRIGHTNESS_L1 234
#define LGD_EA9151_BRIGHTNESS_L2 390
#define LGD_EA9151_BRIGHTNESS_L3 813
#define LGDEA9151_AOD_BRIGHTNESS_REG 0xBF
/**
 * mipi_dsi_dcs_set_display_aod_brightness() - sets the brightness value of the
 *    display
 * @dsi: DSI peripheral device
 * @brightness: brightness value
 *
 * Return: 0 on success or a negative error code on failure.
 */
int mipi_dsi_dcs_set_display_aod_brightness(struct mipi_dsi_device *dsi,
					u16 brightness)
{
	u8 payload[2] = { brightness & 0xff, brightness >> 8 };
	ssize_t err;

	err = mipi_dsi_dcs_write(dsi, LGDEA9151_AOD_BRIGHTNESS_REG,
				 payload, sizeof(payload));
	if (err < 0)
		return err;

	return 0;
}
EXPORT_SYMBOL(mipi_dsi_dcs_set_display_aod_brightness);



int dsi_panel_aod_brightness(struct dsi_panel *panel,
	u32 bl_lvl)
{
	int rc = 0;
	struct mipi_dsi_device *dsi;
	u32 aod_level=0;
	if (!panel || (bl_lvl > 0xffff)) {
		pr_err("invalid params\n");
		return -EINVAL;
	}

	dsi = &panel->mipi_device;


	if(bl_lvl<=LGD_EA9151_BRIGHTNESS_L1){
		aod_level=AOD_10NITS;
	}else if(bl_lvl>LGD_EA9151_BRIGHTNESS_L1 && bl_lvl<=LGD_EA9151_BRIGHTNESS_L2){
		aod_level=AOD_35NITS;
	}else{
		aod_level=AOD_100NITS;
	}
	rc = mipi_dsi_dcs_set_display_aod_brightness(dsi, aod_level);
	if (rc < 0)
		pr_err("failed to update dcs aod backlight:%d\n", aod_level);

	return rc;

}
EXPORT_SYMBOL(dsi_panel_aod_brightness);

int dsi_panel_aod_brightness_trigger(struct dsi_panel *panel, u32 bl_lvl)
{
	int rc = 0;

	if(panel->aod_independent_brightness&&panel->aod_is_ready&&bl_lvl!=0){
		pr_err("AOD Backlight (%d) \n", bl_lvl);
		dsi_panel_aod_brightness(panel,bl_lvl);
		rc=true;
	}else{
		if(panel->aod_is_ready&&bl_lvl==0)
			rc=true;
		else
			rc=false;
	}
	return rc;

}
EXPORT_SYMBOL(dsi_panel_aod_brightness_trigger);

