

extern int dsi_panel_parse_customize_panel_props(struct dsi_panel *panel,
				      struct device_node *of_node);

extern void dsi_panel_show_brightness(u32 bl_lvl);

extern void dsi_panel_power_off_reset_timing(struct dsi_panel *panel);

extern int dsi_panel_display_pixel_on(struct dsi_panel *panel, int bl);

extern void dsi_panel_set_brightness_state(u32 bl_lvl);
