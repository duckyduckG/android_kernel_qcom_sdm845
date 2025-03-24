
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

#define LGD_EA9151_HDR 0x3EE
#define LGD_EA9151_HBM2 0x3FF
#define HBM_NONE 0
#define HDR_LEVEL1 2
#define HMB_LEVEL1 1
static unsigned int hbm_type=0;
unsigned int dsi_enable_hbm(u32 type)
{
	hbm_type = type;
	return 0;
}

unsigned int dsi_get_hbm_state(void)
{
	return hbm_type;
}

int dsi_panel_hbm_verify(struct dsi_panel *panel,u32 bl_lvl)
{
	struct dsi_backlight_config *bl = &panel->bl_config;
	if(bl_lvl==bl->bl_max_level){
		if(hbm_type==HDR_LEVEL1){
			bl_lvl=LGD_EA9151_HDR;
		}else if(hbm_type==HMB_LEVEL1){
			bl_lvl=LGD_EA9151_HBM2;
			pr_info("HBM Enable\n");
		}else{
		}
	}
	return bl_lvl;
}

