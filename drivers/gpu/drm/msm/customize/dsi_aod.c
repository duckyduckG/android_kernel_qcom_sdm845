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
#include "dsi_customize_init.h"

static int aod_en=0;
extern int mxt_touch_state(int);
#if defined(CONFIG_FIH_BATTERY)
extern int panel_notifier_call(int power_mode); /* defined in qpnp-smb2.c */
#endif /* CONFIG_FIH_BATTERY */

int fih_set_glance(int enable)
{
	aod_en = enable;
	pr_err("*** AOD enabled(%d)***\n",aod_en);
	return 0;
}
EXPORT_SYMBOL(fih_set_glance);

int fih_get_glance(void)
{
	pr_debug("*** Is AOD enabled(%d)***\n",aod_en);
	return aod_en;
}
EXPORT_SYMBOL(fih_get_glance);

void dsi_panel_power_mode(struct dsi_panel *panel,int pass,int mode)
{
	if (!panel) {
		pr_err("Invalid params\n");
		return;
	}
	if(pass){
		pr_err("The panel can't change mode\n");
		return;
	}
	panel->panel_power_mode=mode;
	return;
}
EXPORT_SYMBOL(dsi_panel_power_mode);


int dsi_display_set_power(struct drm_connector *connector,
		int power_mode, void *disp)
{
	struct dsi_display *display = disp;
	struct mipi_dsi_device *dsi;
	int rc = 0;

	if (!display || !display->panel) {
		pr_err("invalid display/panel\n");
		return -EINVAL;
	}

	dsi = &display->panel->mipi_device;

	pr_err("Panel Power Mode (%d)\n",power_mode);
	printk("BBox::EHCS;51301:i:LCM Power Mode=%d\n", power_mode);
#if defined(CONFIG_FIH_BATTERY)
	panel_notifier_call(power_mode);
#endif /* CONFIG_FIH_BATTERY */
	if (power_mode == SDE_MODE_DPMS_ON) {
		mxt_touch_state(1);
	} else if (power_mode == SDE_MODE_DPMS_OFF) {
		mxt_touch_state(0);
	} else if (power_mode == SDE_MODE_DPMS_LP1 && display->panel->panel_power_mode==PANEL_LP_TO_ON_MODE){
		//This path is added for glance mode  with under display finger print enabled.
		//Glance mode : LP -> ON -> LP
		mxt_touch_state(0);
	}

	if(!fih_get_glance()){
		pr_err("The Glance Mode is not enabled by UI control\n");
		return rc;
	}
	switch (power_mode) {
	case SDE_MODE_DPMS_ON:
		if(display->panel->aod_is_ready&&display->panel->panel_power_mode==PANEL_LOW_POWER_MODE){
			pr_err("LP-->ON\n");
			mipi_dsi_dcs_set_display_brightness(dsi,0);
			dsi_panel_set_brightness_state(0);
			dsi_panel_set_nolp(display->panel);
			display->panel->aod_is_ready=false;
			dsi_panel_power_mode(display->panel,0,PANEL_LP_TO_ON_MODE);
		}else{
			pr_err("OFF-->ON\n");
		}
		break;
	case SDE_MODE_DPMS_LP1:
		rc = dsi_panel_set_lp1(display->panel);
		display->panel->aod_is_ready=true;
		break;
	case SDE_MODE_DPMS_LP2:
		rc = dsi_panel_set_lp2(display->panel);
		break;
	case SDE_MODE_DPMS_OFF:
		if(display->panel->aod_is_ready&&display->panel->panel_power_mode==PANEL_LOW_POWER_MODE){
			pr_err("LP-->LP OFF(DDIC ON state)\n");
			mipi_dsi_dcs_set_display_brightness(dsi,0);
			dsi_panel_set_nolp(display->panel);
			display->panel->aod_is_ready=false;
		}else{
			pr_err("ON-->OFF\n");
		}
		break;
	default:
		rc = dsi_panel_set_nolp(display->panel);
		break;
	}
	return rc;
}

