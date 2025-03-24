extern int mipi_dsi_dcs_set_display_aod_brightness(struct mipi_dsi_device *dsi,
					u16 brightness);
extern int dsi_panel_aod_brightness(struct dsi_panel *panel,
	u32 bl_lvl);

extern int dsi_panel_aod_brightness_trigger(struct dsi_panel *panel, u32 bl_lvl);

