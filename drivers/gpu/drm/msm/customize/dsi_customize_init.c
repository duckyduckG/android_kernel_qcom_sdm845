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

static int previous_bl_level = 0;

extern int dsi_panel_tx_cmd_set(struct dsi_panel *panel,
				enum dsi_cmd_set_type type);


int dsi_panel_parse_customize_panel_props(struct dsi_panel *panel,
				      struct device_node *of_node)
{
	int panel_id=0,rc=0;
	u32 val= 0;
	of_property_read_u32(of_node, "qcom,mdss-dsi-panel-id",
				  &panel_id);
	panel->panel_id= panel_id;

	panel->bl_reg_swap=of_property_read_bool(of_node,
			"qcom,mdss-dsi-bl-reg-swap");
	pr_info("The brightness reg swap(%d)\n",panel->bl_reg_swap);

	panel->aod_independent_brightness=of_property_read_bool(of_node,
			"qcom,mdss-dsi-aod-independent-brightness");
	pr_info("AOD independent control(%d)\n",panel->aod_independent_brightness);

	rc = of_property_read_u32(of_node, "qcom,mdss-dsi-power-off-reset-timing",
				  &val);
	if(rc)
		panel->reset_offtime=1;
	else
		panel->reset_offtime = val;

	panel->lp11_init = of_property_read_bool(of_node,
			"qcom,mdss-dsi-lp11-init");

	pr_info("MIPI-DSI is %s LP11\n",panel->lp11_init?"USE":"UN-USE");

	panel->panel_on_after_pixel_out = of_property_read_bool(of_node,
			"qcom,mdss-panel-on-after-pixel-out");

	pr_info("DSI Panel on command must PIXEL OUT %s\n",panel->panel_on_after_pixel_out?"After":"Before");

	printk("BBox::EHCS;51304:i:Vendor information=%s\n", panel->name);

	return 0;
}

void dsi_panel_show_brightness(u32 bl_lvl)
{
	if ((bl_lvl == 0) || ((previous_bl_level == 0) && (bl_lvl != 0))){
		pr_err("level=%d\n", bl_lvl);
		printk("BBox::EHCS;51401:i:Backlight status=%d\n", (bl_lvl == 0)?0:1);
	}

	previous_bl_level = bl_lvl;
	return;
}

void dsi_panel_set_brightness_state(u32 bl_lvl)
{
	previous_bl_level = bl_lvl;
	return;
}

void dsi_panel_power_off_reset_timing(struct dsi_panel *panel)
{
	if(!panel)
		return;

	usleep_range(panel->reset_offtime*1000,panel->reset_offtime*1000);
	return;
}

static int dsi_panel_display_on(struct dsi_panel *panel)
{
	int rc = 0;
	struct mipi_dsi_device *dsi;

	pr_err("+\n");

	if (!panel) {
		pr_err("invalid params\n");
		return -EINVAL;
	}
	dsi = &panel->mipi_device;


	rc = mipi_dsi_dcs_set_display_on(dsi);
	if (rc < 0) {
		pr_err("[%s] failed to send DSI_CMD_SET_POST_ON cmds, rc=%d\n",
		       panel->name, rc);
		goto error;
	}
error:
	pr_err("-\n");

	return rc;
}

int dsi_panel_display_pixel_on(struct dsi_panel *panel, int bl)
{
	int rc = 0;

	if (!panel) {
		pr_err("invalid params\n");
		return -EINVAL;
	}
	if(!panel->panel_on_after_pixel_out)
	{
		pr_debug("The pixel on to turn on display not definition\n");
		return rc;
	}
	if(previous_bl_level == 0 && panel->panel_initialized && bl!=0){
		dsi_panel_display_on(panel);
		if(panel->panel_power_mode==PANEL_LP_TO_ON_MODE)
			panel->panel_power_mode = PANEL_ON_MODE;
	}
	return rc;
}
EXPORT_SYMBOL(dsi_panel_display_pixel_on);


