
#define pr_fmt(fmt)	"[msm-dsi-panel:%s:%d] " fmt, __func__, __LINE__
#include "msm_drv.h"
#include "sde_dbg.h"

#include "sde_kms.h"
#include "sde_connector.h"
#include "sde_encoder.h"
#include <linux/backlight.h>
#include <linux/string.h>
#include "dsi_drm.h"
#include "dsi_display.h"
#include "sde_crtc.h"
#include "sde_rm.h"

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <video/mipi_display.h>
#include "dsi_panel.h"
#include "dsi_ctrl_hw.h"
#include "msm_mmu.h"
#include "dsi_ctrl.h"
#include "dsi_clk.h"
#include "dsi_pwr.h"

#define DEFAULT_BASE_REG_CNT 0x100
#define PANEL_TX_MAX_BUF 256
#define PANEL_REG_FORMAT_LEN 5
#define DEFAULT_READ_PANEL_POWER_MODE_REG 0x0A
#define PANEL_REG_ADDR_LEN 8
static DEFINE_MUTEX(mdss_debug_lock);

static char panel_reg[2] = {DEFAULT_READ_PANEL_POWER_MODE_REG, 0x00};
bool dsi_panel_parse_customize_status_len(struct device_node *np,
	char *prop_key, u32 **target, u32 cmd_cnt);

bool dsi_panel_parse_customize_check_valid_params(struct dsi_panel *panel, u32 count);

int dsi_panel_parse_cmd_customize_sets_sub(struct dsi_panel_cmd_set *cmd,
					enum dsi_cmd_set_type type,
					struct device_node *of_node);

int dsi_customize_host_alloc_cmd_tx_buffer(struct dsi_display *display);
int dsi_display_customize_cmd_engine_disable(struct dsi_display *display);
int dsi_display_customize_cmd_engine_enable(struct dsi_display *display);
//==========================================================================================

int dsi_panel_parse_ddic_reg_read_configs(struct dsi_panel *panel,
				struct device_node *of_node)
{
	struct drm_panel_reg_config *reg_config;
	int rc = 0;
	u32 i, status_len, *lenp;

	if (!panel || !of_node) {
		pr_err("Invalid Params\n");
		return -EINVAL;
	}

	reg_config = &panel->reg_config;
	if (!reg_config)
		return -EINVAL;

	dsi_panel_parse_cmd_customize_sets_sub(&reg_config->status_cmd,
				DSI_CMD_SET_PANEL_STATUS, of_node);
	if (!reg_config->status_cmd.count) {
		pr_err("panel status command parsing failed\n");
		rc = -EINVAL;
		goto error;
	}

	if (!dsi_panel_parse_customize_status_len(of_node,
		"qcom,mdss-dsi-panel-status-read-ddic-reg-length",
			&panel->reg_config.status_cmds_rlen,
				reg_config->status_cmd.count)) {
		pr_err("Invalid status read length\n");
		rc = -EINVAL;
		goto error1;
	}

	if (dsi_panel_parse_customize_status_len(of_node,
		"qcom,mdss-dsi-panel-status-valid-params",
			&panel->reg_config.status_valid_params,
				reg_config->status_cmd.count)) {
		if (!dsi_panel_parse_customize_check_valid_params(panel,
					reg_config->status_cmd.count)) {
			rc = -EINVAL;
			goto error2;
		}
	}

	status_len = 0;
	lenp = reg_config->status_valid_params ?: reg_config->status_cmds_rlen;
	for (i = 0; i < reg_config->status_cmd.count; ++i)
		status_len += lenp[i];

	if (!status_len) {
		rc = -EINVAL;
		goto error2;
	}


	reg_config->groups=1;

	reg_config->status_value =
		kzalloc(sizeof(u32) * status_len * reg_config->groups,
			GFP_KERNEL);
	if (!reg_config->status_value) {
		rc = -ENOMEM;
		goto error2;
	}

	reg_config->return_buf = kcalloc(status_len * reg_config->groups,
			sizeof(unsigned char), GFP_KERNEL);
	if (!reg_config->return_buf) {
		rc = -ENOMEM;
		goto error3;
	}

	reg_config->status_buf = kzalloc(SZ_4K, GFP_KERNEL);
	if (!reg_config->status_buf) {
		rc = -ENOMEM;
		goto error4;
	}

	rc = of_property_read_u32_array(of_node,
		"qcom,mdss-dsi-panel-status-value",
		reg_config->status_value, reg_config->groups * status_len);
	if (rc) {
		pr_debug("error reading panel status values\n");
		memset(reg_config->status_value, 0,
				reg_config->groups * status_len);
	}

	return 0;

error4:
	kfree(reg_config->return_buf);
error3:
	kfree(reg_config->status_value);
error2:
	kfree(reg_config->status_valid_params);
	kfree(reg_config->status_cmds_rlen);
error1:
	kfree(reg_config->status_cmd.cmds);
error:
	return rc;
}

int dsi_panel_parse_debug_reg_config(struct dsi_panel *panel,
				     struct device_node *of_node)
{
	struct drm_panel_reg_config *reg_config;

	reg_config = &panel->reg_config;
	reg_config->reg_enabled = of_property_read_bool(of_node,
		"qcom,reg-check-enabled");

	if (!reg_config->reg_enabled)
		return 0;

	dsi_panel_parse_ddic_reg_read_configs(panel, of_node);

	pr_info("DDIC REG enabled with mode: %d\n", reg_config->reg_enabled);

	return 0;

}


static int dsi_display_read_panel_reg_status(struct dsi_display_ctrl *ctrl,
		struct dsi_panel *panel,char cmd0,char *rbuf, int len)
{
	int i, rc = 0, count = 0, start = 0, *lenp;
	struct drm_panel_reg_config *config;
	struct dsi_cmd_desc *cmds;
	u32 flags = 0;

	if (!panel || !ctrl || !ctrl->ctrl)
		return -EINVAL;

	if (dsi_ctrl_validate_host_state(ctrl->ctrl))
		return 1;

	/* acquire panel_lock to make sure no commands are in progress */
	dsi_panel_acquire_panel_lock(panel);

	config = &(panel->reg_config);
	lenp = config->status_valid_params ?: config->status_cmds_rlen;
	count = config->status_cmd.count;
	cmds = config->status_cmd.cmds;
	flags |= (DSI_CTRL_CMD_FETCH_MEMORY | DSI_CTRL_CMD_READ);

	for (i = 0; i < count; ++i) {
		memset(config->status_buf, 0x0, SZ_4K);
		if (cmds[i].last_command) {
			cmds[i].msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;
			flags |= DSI_CTRL_CMD_LAST_COMMAND;
		}
		cmds[i].msg.tx_buf = &cmd0;
		cmds[i].msg.rx_buf = rbuf;
		cmds[i].msg.rx_len = len;
		rc = dsi_ctrl_cmd_transfer(ctrl->ctrl, &cmds[i].msg, flags);
		if (rc <= 0) {
			pr_err("rx cmd transfer failed rc=%d\n", rc);
			goto error;
		}

		memcpy(config->return_buf + start,
			config->status_buf, lenp[i]);
		start += lenp[i];
	}

error:
	/* release panel_lock */
	dsi_panel_release_panel_lock(panel);
	return rc;
}


static int dsi_display_validate_read_status(struct dsi_display_ctrl *ctrl,
		struct dsi_panel *panel,char cmd0,char *rbuf, int len)
{
	return dsi_display_read_panel_reg_status(ctrl, panel,cmd0,rbuf, len);

}


static int dsi_display_panel_reg_read(struct dsi_display *display,char cmd0,char *rbuf, int len)
{
	int rc = 0, i;
	struct dsi_display_ctrl *m_ctrl, *ctrl;

	pr_debug(" ++\n");

	m_ctrl = &display->ctrl[display->cmd_master_idx];

	if (display->tx_cmd_buf == NULL) {
		rc = dsi_customize_host_alloc_cmd_tx_buffer(display);
		if (rc) {
			pr_err("failed to allocate cmd tx buffer memory\n");
			goto done;
		}
	}

	rc = dsi_display_customize_cmd_engine_enable(display);
	if (rc) {
		pr_err("cmd engine enable failed\n");
		return -EPERM;
	}

	rc = dsi_display_validate_read_status(m_ctrl, display->panel,cmd0,rbuf, len);
	if (rc <= 0) {
		pr_err("[%s] read status failed on master,rc=%d\n",
		       display->name, rc);
		goto exit;
	}

	if (!display->panel->sync_broadcast_en)
		goto exit;

	for (i = 0; i < display->ctrl_count; i++) {
		ctrl = &display->ctrl[i];
		if (ctrl == m_ctrl)
			continue;

		rc = dsi_display_validate_read_status(ctrl, display->panel,cmd0,rbuf, len);
		if (rc <= 0) {
			pr_err("[%s] read status failed on slave,rc=%d\n",
			       display->name, rc);
			goto exit;
		}
	}
exit:


	dsi_display_customize_cmd_engine_disable(display);
done:
	return rc;
}

int dsi_display_panel_cmd_read(void *display,char cmd0,char *rbuf, int len)
{
	struct dsi_display *dsi_display = display;
	struct dsi_panel *panel;
	int rc = 0x1;

	if (dsi_display == NULL)
		return -EINVAL;

	panel = dsi_display->panel;

	if(!panel->reg_config.reg_enabled){
		pr_info("Panel property is not initialized\n");
		return rc;
	}


	mutex_lock(&dsi_display->display_lock);

	if (!panel->panel_initialized) {
		pr_info("Panel not initialized\n");
		mutex_unlock(&dsi_display->display_lock);
		return rc;
	}

	dsi_display_clk_ctrl(dsi_display->dsi_clk_handle,
		DSI_ALL_CLKS, DSI_CLK_ON);

	//Read DDID register value
	dsi_display_panel_reg_read(display,cmd0,rbuf,len);

	dsi_display_clk_ctrl(dsi_display->dsi_clk_handle,
		DSI_ALL_CLKS, DSI_CLK_OFF);
	mutex_unlock(&dsi_display->display_lock);

	return rc;
}

EXPORT_SYMBOL(dsi_display_panel_cmd_read);

static int panel_debug_base_open(struct inode *inode, struct file *file)
{
	/* non-seekable */
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	file->private_data = inode->i_private;
	return 0;
}

static int panel_debug_base_release(struct inode *inode, struct file *file)
{
	struct drm_connector *connector = file->private_data;
	struct sde_connector *c_conn;
	c_conn = to_sde_connector(connector);
	mutex_lock(&mdss_debug_lock);
	if (c_conn && c_conn->buffer) {
		kfree(c_conn->buffer);
		c_conn->buffer = NULL;
	}
	mutex_unlock(&mdss_debug_lock);
	return 0;
}

static ssize_t panel_debug_base_offset_read(struct file *file,
			char __user *buff, size_t count, loff_t *ppos)
{
	int len = 0;
	char buf[PANEL_TX_MAX_BUF] = {0x0};
	struct drm_connector *connector = file->private_data;
	struct sde_connector *c_conn;
	c_conn = to_sde_connector(connector);
	if (!c_conn)
		return -ENODEV;

	if (*ppos)
		return 0;	/* the end */

	mutex_lock(&mdss_debug_lock);
	len = snprintf(buf, sizeof(buf), "0x%02zx 0x%02zx\n", c_conn->reg_off, c_conn->reg_cnt);
	if (len < 0 || len >= sizeof(buf)) {
		mutex_unlock(&mdss_debug_lock);
		return 0;
	}

	if ((count < sizeof(buf)) || copy_to_user(buff, buf, len)) {
		mutex_unlock(&mdss_debug_lock);
		return -EFAULT;
	}

	pr_debug("offset=0x%02zx cnt=0x%02zx\n", c_conn->reg_off, c_conn->reg_cnt);
	*ppos += len;	/* increase offset */

	mutex_unlock(&mdss_debug_lock);
	return len;
}

static ssize_t panel_debug_base_offset_write(struct file *file,
		    const char __user *user_buf, size_t count, loff_t *ppos)
{
	u32 off = 0;
	u32 cnt = DEFAULT_BASE_REG_CNT;
	char buf[PANEL_TX_MAX_BUF] = {0x0};
	struct drm_connector *connector = file->private_data;
	struct sde_connector *c_conn;
	c_conn = to_sde_connector(connector);

	if (!c_conn)
		return -ENODEV;

	if (count >= sizeof(buf))
		return -EFAULT;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	buf[count] = 0;	/* end of string */

	if (sscanf(buf, "%x %x", &off, &cnt) != 2)
		return -EFAULT;

	if (off > c_conn->max_offset)
		return -EINVAL;


	mutex_lock(&mdss_debug_lock);
	c_conn->reg_off = off;
	c_conn->reg_cnt = cnt;
	mutex_unlock(&mdss_debug_lock);

	pr_debug("offset=%x cnt=%d\n", off, cnt);

	return count;
}


static const struct file_operations panel_off_fops = {
	.open = panel_debug_base_open,
	.release = panel_debug_base_release,
	.read = panel_debug_base_offset_read,
	.write = panel_debug_base_offset_write,
};


static ssize_t panel_debug_base_reg_read(struct file *file,
			char __user *user_buf, size_t count, loff_t *ppos)
{
	u32 i, len = 0, reg_buf_len = 0;
	char *panel_reg_buf, *rx_buf;
	struct drm_connector *connector = file->private_data;
	struct sde_connector *c_conn;
	struct dsi_display *display;
	int rc = -EFAULT;
	c_conn = to_sde_connector(connector);


	if (!c_conn)
		return -ENODEV;

	mutex_lock(&mdss_debug_lock);
	if (!c_conn->reg_cnt) {
		mutex_unlock(&mdss_debug_lock);
		return 0;
	}

	if (*ppos) {
		mutex_unlock(&mdss_debug_lock);
		return 0;	/* the end */
	}

	display = (struct dsi_display *) c_conn->display;

	/* '0x' + 2 digit + blank = 5 bytes for each number */
	reg_buf_len = (c_conn->reg_cnt * PANEL_REG_FORMAT_LEN)
		    + PANEL_REG_ADDR_LEN + 1;
	rx_buf = kzalloc(c_conn->reg_cnt, GFP_KERNEL);
	panel_reg_buf = kzalloc(reg_buf_len, GFP_KERNEL);

	if (!rx_buf || !panel_reg_buf) {
		pr_err("not enough memory to hold panel reg dump\n");
		rc = -ENOMEM;
		goto read_reg_fail;
	}

	panel_reg[0] = c_conn->reg_off;
	dsi_display_panel_cmd_read(display, panel_reg[0],rx_buf, c_conn->reg_cnt);

	len = scnprintf(panel_reg_buf, reg_buf_len, "0x%02zx: ", c_conn->reg_off);

	for (i = 0; (i < c_conn->reg_cnt); i++)
		len += scnprintf(panel_reg_buf + len, reg_buf_len - len,
				"0x%02x ", rx_buf[i]);

	if (len)
		panel_reg_buf[len - 1] = '\n';


	if ((count < reg_buf_len)
			|| (copy_to_user(user_buf, panel_reg_buf, len)))
		goto read_reg_fail;

	kfree(rx_buf);
	kfree(panel_reg_buf);

	*ppos += len;	/* increase offset */
	mutex_unlock(&mdss_debug_lock);
	return len;

read_reg_fail:
	kfree(rx_buf);
	kfree(panel_reg_buf);
	mutex_unlock(&mdss_debug_lock);
	return rc;
}



static const struct file_operations panel_reg_fops = {
	.open = panel_debug_base_open,
	.release = panel_debug_base_release,
	.read = panel_debug_base_reg_read,
};

int sde_connector_init_customize_debugfs(struct drm_connector *connector)
{
	struct sde_connector *sde_connector;
	struct msm_display_info info;

	if (!connector || !connector->debugfs_entry) {
		SDE_ERROR("invalid connector\n");
		return -EINVAL;
	}

	sde_connector = to_sde_connector(connector);

	sde_connector_get_info(connector, &info);

	if(info.is_primary){
		if (!debugfs_create_file("panel_off", 0644,
			connector->debugfs_entry,
			connector, &panel_off_fops)) {
			SDE_ERROR("failed to create connector panel_off\n");
			return -ENOMEM;
		}
		sde_connector->reg_off=0x0A;
		sde_connector->reg_cnt=0x01;
		sde_connector->max_offset=0xFF;

		if (!debugfs_create_file("panel_reg", 0644,
			connector->debugfs_entry,
			connector, &panel_reg_fops)) {
			SDE_ERROR("failed to create connector panel_reg\n");
			return -ENOMEM;
		}
	}
	return 0;
}

