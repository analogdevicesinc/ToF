// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the Analog Devices ADSD3500 chip.
 *
 * Copyright (C) 2022 Analog Devices, All Rights Reserved.
 *
 */

#include "adsd3500_regs.h"

#include <linux/bitfield.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pwm.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-of.h>
#include <media/v4l2-subdev.h>

#include <media/tegra-v4l2-camera.h>
#include <media/camera_common.h>

#include "adsd3500_mode_tbls.h"

#define ADSD3500_DEFAULT_MODE		ADSD3500_MODE_512x512_30FPS
#define ADSD3500_DEFAULT_DATAFMT	MEDIA_BUS_FMT_SBGGR12_1X12
#define ADSD3500_DEFAULT_WIDTH		512
#define ADSD3500_DEFAULT_HEIGHT		512

struct adsd3500_mode_info {
	uint32_t width;
	uint32_t height;
	uint32_t pixel_rate;
	uint32_t code;
	uint32_t link_freq_idx;
};

struct adsd3500_config_info {
	uint8_t nr_depth_bits;
	uint8_t nr_ab_bits;
	uint8_t nr_confidence_bits;
	uint8_t nr_mipi_lanes;
	bool use_vc;
};

struct adsd3500 {
	struct i2c_client *i2c_client;
	struct regmap *regmap;
	struct device *dev;
	struct v4l2_subdev *sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_rect crop;

	const struct adsd3500_mode_info *current_mode;
	struct adsd3500_config_info current_config;

	struct camera_common_data	*s_data;
	struct camera_common_pdata	*pdata;

	struct v4l2_ctrl_handler 	ctrl_handler;

	struct mutex lock;
	bool streaming;

	struct v4l2_ctrl *ctrls[];
};

#define V4L2_CID_ADSD3500_OPERATING_MODE  (V4L2_CID_USER_ADITOF_BASE + 0)
#define V4L2_CID_ADSD3500_CHIP_CONFIG (V4L2_CID_USER_ADITOF_BASE + 1)
#define V4L2_CID_ADSD3500_DEPTH_BITS (V4L2_CID_USER_ADITOF_BASE + 2)
#define V4L2_CID_ADSD3500_AB_BITS (V4L2_CID_USER_ADITOF_BASE + 3)
#define V4L2_CID_ADSD3500_CONFIDENCE_BITS (V4L2_CID_USER_ADITOF_BASE + 4)
#define V4L2_CID_ADSD3500_AB_AVG (V4L2_CID_USER_ADITOF_BASE + 5)
#define V4L2_CID_ADSD3500_DEPTH_EN (V4L2_CID_USER_ADITOF_BASE + 6)

static const struct reg_sequence adsd3500_powerup_setting[] = {
};

static const struct reg_sequence adsd3500_powerdown_setting[] = {
};

static const struct reg_sequence adsd3500_standby_setting[] = {
};

static const s64 link_freq_tbl[] = {
	732000000,
	732000000,
	732000000,
	732000000,
	732000000,
	732000000,
	732000000,
	732000000,
	732000000,
};

/* Elements of the structure must be ordered ascending by width & height */
static const struct adsd3500_mode_info adsd3500_mode_info_data[] = {
	{ //RAW12 12BPP
		.width = 512,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{ //RAW8 16BPP
		.width = 1024,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 1 /* an index in link_freq_tbl[] */
	},
	{ //RAW8 20BPP
		.width = 1280,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 2 /* an index in link_freq_tbl[] */
	},
	{ //RAW8 24BPP
		.width = 1536,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 3 /* an index in link_freq_tbl[] */
	},
	{ //RAW8 28BPP
		.width = 1792,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 4 /* an index in link_freq_tbl[] */
	},
	{ //RAW8 32BPP
		.width = 2048,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 5 /* an index in link_freq_tbl[] */
	},
	{ //RAW8 36BPP
		.width = 2304,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 6 /* an index in link_freq_tbl[] */
	},
	{ //RAW8 40BPP
		.width = 2560,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 7 /* an index in link_freq_tbl[] */
	},
	{ //RAW12 12BPP * 3 phase
		.width = 3072,
		.height = 1024,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.link_freq_idx = 8 /* an index in link_freq_tbl[] */
	}
};

static bool adsd3500_regmap_accessible_reg(struct device *dev, unsigned int reg)
{
	if (reg % 2)
		return 0;

	switch (reg) {
		case GET_CHIP_ID_CMD:
		case GET_IMAGER_MODE_CMD:
		case GET_IMAGER_AB_INVLD_TRSHLD:
		case GET_IMAGER_CONFIDENCE_TRSHLD:
		case GET_IMAGER_JBLF_STATE:
		case GET_IMAGER_JBLF_FILT_SIZE:
		case GET_STATUS_CMD:
			return 1;
		default:
			return 0;
	}
}

static const struct regmap_config adsd3500_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.cache_type = REGCACHE_NONE,
	.readable_reg = adsd3500_regmap_accessible_reg,
};

static int adsd3500_power_on(struct camera_common_data *s_data)
{
	struct adsd3500 *adsd3500 = (struct adsd3500 *)s_data->priv;
	struct v4l2_ctrl *operating_mode = adsd3500->ctrls[1];
	struct v4l2_ctrl *ab_avg = adsd3500->ctrls[2];
	struct v4l2_ctrl *depth_en = adsd3500->ctrls[3];
	struct adsd3500_config_info config = adsd3500->current_config;
	unsigned int write_cmd, write_val = 0;
	int ret;
	
	dev_dbg(adsd3500->dev, "Entered addicmos_power_on\n");
	
	write_cmd = SET_IMAGER_MODE_CMD | SET_IMAGER_MODE(operating_mode->val);

	write_val |= SET_IMAGER_MODE_DEPTH_EN(depth_en->val);
	write_val |= SET_IMAGER_MODE_DEPTH_BITS(config.nr_depth_bits ? 6 - config.nr_depth_bits: 0);
	
	write_val |= SET_IMAGER_MODE_AB_EN(config.nr_ab_bits ? 1 : 0);
	write_val |= SET_IMAGER_MODE_AB_BITS(config.nr_ab_bits ? 6 - config.nr_ab_bits : 0);

	write_val |= SET_IMAGER_MODE_CONF_BITS(config.nr_confidence_bits);

	write_val |= SET_IMAGER_MODE_VC_EN(!config.use_vc);
	write_val |= SET_IMAGER_MODE_AB_AVG_EN(ab_avg->val);
	write_val |= SET_IMAGER_MODE_MIPI_LANES_NR(config.nr_mipi_lanes);

	ret = regmap_write(adsd3500->regmap, write_cmd, write_val);
	if (ret) {
		dev_err(adsd3500->dev, "Could not set mode register\n");
		return ret;
	}
	
	return 0;
}

static int adsd3500_power_off(struct camera_common_data *s_data)
{
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int adsd3500_g_register(struct v4l2_subdev *sd,
			       struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;
	struct camera_common_data *s_data = to_camera_common_data(dev);
	struct adsd3500 *adsd3500 = (struct adsd3500 *)s_data->priv;
	unsigned int read_val;
	int ret;

	reg->size = 2;
	ret = regmap_read(adsd3500->regmap, reg->reg, &read_val);
	reg->val = read_val;

	return ret;
}

static int adsd3500_s_register(struct v4l2_subdev *sd,
			       const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;
	struct camera_common_data *s_data = to_camera_common_data(dev);
	struct adsd3500 *adsd3500 = (struct adsd3500 *)s_data->priv;

	return regmap_write(adsd3500->regmap, reg->reg, reg->val);
}
#endif

static int adsd3500_bpp_config(struct adsd3500 *priv,
				    struct v4l2_ctrl *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_ADSD3500_DEPTH_BITS:
		priv->current_config.nr_depth_bits = ctrl->val;
		break;
	case V4L2_CID_ADSD3500_AB_BITS:
		priv->current_config.nr_ab_bits = ctrl->val;
		break;
	case V4L2_CID_ADSD3500_CONFIDENCE_BITS:
		priv->current_config.nr_confidence_bits = ctrl->val;
		break;
	default:
		break;
	}

	return 0;
}

static int adsd3500_chip_config(struct adsd3500 *adsd3500,
				    struct v4l2_ctrl *ctrl)
{
	struct device *dev = adsd3500->dev;
	struct i2c_client *client = adsd3500->i2c_client;
	uint16_t pld_size;
	uint8_t r_w, *data;
	int ret;

	r_w = *ctrl->p_new.p_u8;
	pld_size = (uint16_t)(*(ctrl->p_new.p_u8 + 1) << 8 | *(ctrl->p_new.p_u8 + 2));
	data = ctrl->p_new.p_u8 + 3;

	dev_dbg(dev, "Entered adsd3500_chip_config. R/W: %d, PLD_SIZE: %d\n", r_w, pld_size);

	if ((pld_size > 4096) || (pld_size < 2))
		return -EINVAL;

	if (r_w) {
		ret = i2c_master_send(client, data, pld_size);
		if (ret < 0) {
			dev_warn(dev, "Write burst transfer failed\n");
			return -EIO;
		}
	} else {
		ret = i2c_master_recv(client, data, pld_size);
		if (ret < 0) {
			dev_warn(dev, "Read burst transfer failed\n");
			return -EIO;
		}
	}
	memset(ctrl->p_new.p_u8, 0xFF, 1);
	return 0;
}

static int adsd3500_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct adsd3500 *priv = container_of(ctrl->handler, struct adsd3500, ctrl_handler);
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;

	switch (ctrl->id) {
	default:
		dev_err(dev, "%s: unknown ctrl id.\n", __func__);
		return -EINVAL;
	}

	return err;
}

static int adsd3500_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct adsd3500 *adsd3500 = container_of(ctrl->handler,
						 struct adsd3500, ctrl_handler);
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_ADSD3500_OPERATING_MODE:
	case TEGRA_CAMERA_CID_SENSOR_MODE_ID:
	case V4L2_CID_ADSD3500_AB_AVG:
	case V4L2_CID_ADSD3500_DEPTH_EN:
	case V4L2_CID_PIXEL_RATE:
	case V4L2_CID_LINK_FREQ:
		break;
	case V4L2_CID_ADSD3500_CHIP_CONFIG:
		ret = adsd3500_chip_config(adsd3500, ctrl);
		break;
	case V4L2_CID_ADSD3500_DEPTH_BITS:
	case V4L2_CID_ADSD3500_AB_BITS:
	case V4L2_CID_ADSD3500_CONFIDENCE_BITS:
		ret = adsd3500_bpp_config(adsd3500, ctrl);
		break;
	default:
		dev_err(adsd3500->dev, "%s > Unhandled: %x  param=%x\n",
			__func__, ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops adsd3500_ctrl_ops = {
	.g_volatile_ctrl = adsd3500_g_volatile_ctrl,
	.s_ctrl = adsd3500_s_ctrl,
};

static const s64 nr_bits_qmenu[] = {
	0, 4, 8, 10, 12, 14, 16
};

static const struct v4l2_ctrl_config adsd3500_ctrls[] = {
	{
		.ops		= &adsd3500_ctrl_ops,
		.id		= TEGRA_CAMERA_CID_SENSOR_MODE_ID,
		.name		= "Sensor Mode",
		.type		= V4L2_CTRL_TYPE_INTEGER64,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
		.min		= 0,
		.max		= 0xFF,
		.def		= 0x0,
		.step 		= 1,
	},
	{
		/* Should always be second control in list*/
		.ops		= &adsd3500_ctrl_ops,
		.id		= V4L2_CID_ADSD3500_OPERATING_MODE,
		.name		= "Operating Mode",
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.def		= 7,
		.min		= 0,
		.max		= 10,
		.step		= 1,
	},
	{
		/* Should always be third control in list*/
		.ops		= &adsd3500_ctrl_ops,
		.id		= V4L2_CID_ADSD3500_AB_AVG,
		.name		= "AB Averaging",
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.def		= 1,
		.min		= 0,
		.max		= 1,
		.step		= 1,
	},
	{
		/* Should always be fourth control in list*/
		.ops		= &adsd3500_ctrl_ops,
		.id		= V4L2_CID_ADSD3500_DEPTH_EN,
		.name		= "Depth enable",
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.def		= 1,
		.min		= 0,
		.max		= 1,
		.step		= 1,
	},
	{
		.ops		= &adsd3500_ctrl_ops,
		.id		= V4L2_CID_ADSD3500_CHIP_CONFIG,
		.name		= "Chip Config",
		.type		= V4L2_CTRL_TYPE_U8,
		.def		= 0x00,
		.min		= 0x00,
		.max		= 0xFF,
		.step		= 1,
		.dims		= { 4099 }
	},
	{
		.ops		= &adsd3500_ctrl_ops,
		.id		= V4L2_CID_ADSD3500_DEPTH_BITS,
		.name		= "Phase / Depth Bits",
		.type		= V4L2_CTRL_TYPE_INTEGER_MENU,
		.def		= 2,
		.min		= 2,
		.max		= 6,
		.menu_skip_mask = 0x03,
		.qmenu_int	= nr_bits_qmenu,
	},
	{
		.ops		= &adsd3500_ctrl_ops,
		.id		= V4L2_CID_ADSD3500_AB_BITS,
		.name		= "AB Bits",
		.type		= V4L2_CTRL_TYPE_INTEGER_MENU,
		.def		= 0,
		.min		= 0,
		.max		= 6,
		.menu_skip_mask = 0x02,
		.qmenu_int	= nr_bits_qmenu,
	},
	{
		.ops		= &adsd3500_ctrl_ops,
		.id		= V4L2_CID_ADSD3500_CONFIDENCE_BITS,
		.name		= "Confidence Bits",
		.type		= V4L2_CTRL_TYPE_INTEGER_MENU,
		.def		= 0,
		.min		= 0,
		.max		= 2,
		.qmenu_int	= nr_bits_qmenu,
	}
};

static int adsd3500_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad != 0)
		return -EINVAL;

	switch (code->index) {
	case 0:
		code->code = MEDIA_BUS_FMT_SBGGR8_1X8;
		break;
	case 1:
		code->code = MEDIA_BUS_FMT_SBGGR12_1X12;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int adsd3500_enum_frame_size(struct v4l2_subdev *subdev,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	int i, j = 0;

	if (fse->index >= ARRAY_SIZE(adsd3500_mode_info_data))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(adsd3500_mode_info_data); i++)
	{
		if(adsd3500_mode_info_data[i].code == fse->code)
			j++;
		if (j > fse->index)
			break;
	}

	if (i < ARRAY_SIZE(adsd3500_mode_info_data)) {
		fse->min_width = adsd3500_mode_info_data[i].width;
		fse->max_width = adsd3500_mode_info_data[i].width;
		fse->min_height = adsd3500_mode_info_data[i].height;
		fse->max_height = adsd3500_mode_info_data[i].height;
	} else {
		return -EINVAL;
	}

	return 0;
}

static int adsd3500_get_format(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}

static int adsd3500_set_format(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_format *format)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;
	int ret;
	
	dev_dbg(dev, "set_fmt: %x %dx%d %d\n",
		format->format.code, format->format.width,
		format->format.height, format->which);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		ret = camera_common_try_fmt(sd, &format->format);
	else
		ret = camera_common_s_fmt(sd, &format->format);

	return ret;
}

static int adsd3500_start_streaming(struct adsd3500 *adsd3500)
{
	int ret;

	ret = regmap_write(adsd3500->regmap, STREAM_ON_CMD, STREAM_ON_VAL);
	if (ret < 0)
		dev_err(adsd3500->dev, "Write of STREAM-ON command failed.\n");

	return ret;
}

static int adsd3500_stop_streaming(struct adsd3500 *adsd3500)
{
	int ret;

	ret = regmap_write(adsd3500->regmap, STREAM_OFF_CMD, STREAM_OFF_VAL);
	if (ret < 0)
		dev_err(adsd3500->dev, "Write of STREAM-OFF command failed.\n");
	
	return ret;
}

static int adsd3500_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(subdev);
	struct device *dev = &client->dev;
	struct camera_common_data *s_data = to_camera_common_data(dev);
	struct adsd3500 *adsd3500 = (struct adsd3500 *)s_data->priv;
	int ret = 0;

	dev_dbg(dev, "s_stream: %d\n", enable);

	mutex_lock(&adsd3500->lock);
	if (adsd3500->streaming == enable) {
		mutex_unlock(&adsd3500->lock);
		return 0;
	}

	if (enable) {
		adsd3500_power_on(s_data);
		ret = adsd3500_start_streaming(adsd3500);
		if (ret)
			goto err_unlock;
	} else {
		adsd3500_stop_streaming(adsd3500);
	}

	adsd3500->streaming = enable;
	mutex_unlock(&adsd3500->lock);

	return ret;
	
err_unlock:
	mutex_unlock(&adsd3500->lock);

	return ret;
}

static int adsd3500_g_frame_interval(struct v4l2_subdev *subdev,
				     struct v4l2_subdev_frame_interval *fi)
{
	fi->interval.numerator = 1;
	fi->interval.denominator = 30;

	return 0;
}

static int adsd3500_s_frame_interval(struct v4l2_subdev *subdev,
				     struct v4l2_subdev_frame_interval *fi)
{
	return 0;
}

static int adsd3500_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	return 0;
}

static int adsd3500_ctrls_init(struct adsd3500 *priv)
{
	struct i2c_client *client = priv->i2c_client;
	struct v4l2_ctrl *ctrl;
	int num_ctrls;
	int err;
	int i;

	dev_dbg(&client->dev, "%s++\n", __func__);

	num_ctrls = ARRAY_SIZE(adsd3500_ctrls);
	v4l2_ctrl_handler_init(&priv->ctrl_handler, num_ctrls);

	for (i = 0; i < num_ctrls; i++) {
		ctrl = v4l2_ctrl_new_custom(&priv->ctrl_handler,
			&adsd3500_ctrls[i], NULL);
		if (ctrl == NULL) {
			dev_err(&client->dev, "Failed to init %s ctrl\n",
				adsd3500_ctrls[i].name);
			continue;
		}

		if (adsd3500_ctrls[i].type == V4L2_CTRL_TYPE_STRING &&
			adsd3500_ctrls[i].flags & V4L2_CTRL_FLAG_READ_ONLY) {
			ctrl->p_new.p_char = devm_kzalloc(&client->dev,
				adsd3500_ctrls[i].max + 1, GFP_KERNEL);
		}
		priv->ctrls[i] = ctrl;
	}

	priv->sd->ctrl_handler = &priv->ctrl_handler;
	if (priv->ctrl_handler.error) {
		dev_err(&client->dev, "Error %d adding controls\n",
			priv->ctrl_handler.error);
		err = priv->ctrl_handler.error;
		goto error;
	}
	
		//Initialize by default to 4 (RAW12, 12 bpp)
	v4l2_ctrl_s_ctrl(priv->ctrls[5], 4);

	return 0;

error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return err;
}

static int adsd3500_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops adsd3500_subdev_internal_ops = {
	.open = adsd3500_open,
};

const static struct of_device_id adsd3500_of_match[] = {
	{ .compatible = "adi,adsd3500" },
	{ /* sentinel */ }
};

static struct camera_common_pdata *adsd3500_parse_dt(struct i2c_client *client,
				struct camera_common_data *s_data)
{
	struct adsd3500 *adsd3500 = (struct adsd3500 *)s_data->priv;
	struct device_node *np = client->dev.of_node;
	struct camera_common_pdata *board_priv_pdata;
	int err;
	int gpio;
	const char *str;

	if (!np)
		return NULL;
	
	board_priv_pdata = devm_kzalloc(&client->dev,
			sizeof(*board_priv_pdata), GFP_KERNEL);

	adsd3500->current_config.use_vc = of_property_read_bool(np, "adi,use-vc");
	if (adsd3500->current_config.use_vc)
		dev_dbg(&client->dev, "Virtual Channel mode activated\n");

	err = of_property_read_string(np, "use_sensor_mode_id", &str);
	if (!err) {
		if (!strcmp(str, "true"))
			s_data->use_sensor_mode_id = true;
		else
			s_data->use_sensor_mode_id = false;
	}

	gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (gpio < 0) {
		dev_err(&client->dev, "Reset-gpios not found %d\n", gpio);
		gpio = 0;
	}
	board_priv_pdata->reset_gpio = (unsigned int)gpio;

	return board_priv_pdata;
}

MODULE_DEVICE_TABLE(of, adsd3500_of_match);

static struct camera_common_sensor_ops adsd3500_common_ops = {
	.power_off = adsd3500_power_off
};

static const struct v4l2_subdev_core_ops adsd3500_core_ops = {
	.s_power = camera_common_s_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = adsd3500_g_register,
	.s_register = adsd3500_s_register,
#endif
};

static const struct v4l2_subdev_video_ops adsd3500_video_ops = {
	.s_stream = adsd3500_s_stream,
	.g_frame_interval = adsd3500_g_frame_interval,
	.s_frame_interval = adsd3500_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops adsd3500_subdev_pad_ops = {
	.enum_mbus_code = adsd3500_enum_mbus_code,
	.enum_frame_size = adsd3500_enum_frame_size,
	.get_fmt = adsd3500_get_format,
	.set_fmt = adsd3500_set_format,
};

static const struct v4l2_subdev_ops adsd3500_subdev_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.core = &adsd3500_core_ops,
#endif
	.video = &adsd3500_video_ops,
	.pad = &adsd3500_subdev_pad_ops,
};

static const struct media_entity_operations adsd3500_subdev_entity_ops = {
	.link_setup = adsd3500_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

static int adsd3500_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_common_data *common_data;
	struct adsd3500 *priv;
	unsigned int read_val;
	int ret;

	dev_info(&client->dev, "probing adsd3500 v4l2 sensor\n");

	common_data = devm_kzalloc(&client->dev, sizeof(struct camera_common_data),
							   GFP_KERNEL);

	priv = devm_kzalloc(&client->dev,
			    sizeof(struct adsd3500) + sizeof(struct v4l2_ctrl *) *
			    ARRAY_SIZE(adsd3500_ctrls),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	common_data->priv = (void *)priv;

	priv->regmap = devm_regmap_init_i2c(client,
						 &adsd3500_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&client->dev, "Error initializing I2C regmap\n");
		return PTR_ERR(priv->regmap);
	}

	if (client->dev.of_node)
		priv->pdata = adsd3500_parse_dt(client, common_data);
	if (!priv->pdata) {
		dev_err(&client->dev, "unable to get platform data\n");
		return -EFAULT;
	}

	common_data->ops = &adsd3500_common_ops;
	common_data->ctrl_handler = &priv->ctrl_handler;
	common_data->dev = &client->dev;
	common_data->frmfmt = &adsd3500_frmfmt[0];
	common_data->colorfmt = camera_common_find_datafmt(
					  ADSD3500_DEFAULT_DATAFMT);
	common_data->numctrls = 0;
	common_data->numfmts = ARRAY_SIZE(adsd3500_frmfmt);
	common_data->def_mode = ADSD3500_DEFAULT_MODE;
	common_data->def_width = ADSD3500_DEFAULT_WIDTH;
	common_data->def_height = ADSD3500_DEFAULT_HEIGHT;
	common_data->fmt_width = common_data->def_width;
	common_data->fmt_height = common_data->def_height;

	priv->dev = &client->dev;
	priv->i2c_client = client;
	priv->s_data = common_data;
	priv->sd = &common_data->subdev;
	priv->sd->dev = &client->dev;
	priv->s_data->dev = &client->dev;
	mutex_init(&priv->lock);

	ret = camera_common_initialize(common_data, "adi_adsd3500");
	if (ret) {
		dev_err(&client->dev, "Failed to initialize adsd3500.\n");
		return ret;
	}

	priv->current_config.nr_mipi_lanes = common_data->numlanes;
	dev_dbg(&client->dev, "Lanes nr: %u\n", priv->current_config.nr_mipi_lanes);

	ret = regmap_read(priv->regmap, GET_CHIP_ID_CMD, &read_val);
	if (ret < 0) {
		dev_err(&client->dev, "Read of Chip ID register failed.\n");
		return -EIO;
	}
#if 0

	if (read_val != ADSD3500_CHIP_ID) {
		dev_err(&client->dev, "Chip ID: %.4X is wrong.\n", read_val);
		return -EFAULT;
	}
	dev_dbg(&client->dev, "Read Chip ID: %.4X\n", read_val);
#endif
	v4l2_i2c_subdev_init(priv->sd, client, &adsd3500_subdev_ops);
	
	ret = adsd3500_ctrls_init(priv);
	if (ret)
		return ret;

	priv->sd->internal_ops = &adsd3500_subdev_internal_ops;
	priv->sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->sd->entity.ops = &adsd3500_subdev_entity_ops;
	ret = tegra_media_entity_init(&priv->sd->entity, 1,
				&priv->pad, true, true);
	if (ret) {
		dev_err(&client->dev, "unable to init media entity\n");
		return ret;
	}

	ret = v4l2_async_register_subdev(priv->sd);
	if (ret) {
		dev_err(&client->dev, "could not register v4l2 device\n");
		return ret;
	}

	dev_info(&client->dev, "Detected ADSD3500 sensor\n");

	return 0;
}

static int adsd3500_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct adsd3500 *priv = (struct adsd3500 *)s_data->priv;

	v4l2_async_unregister_subdev(priv->sd);
	media_entity_cleanup(&priv->sd->entity);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	camera_common_cleanup(priv->s_data);
	mutex_destroy(&priv->lock);

	return 0;
}

static const struct i2c_device_id adsd3500_id[] = {
	{ "adsd3500", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, adsd3500_id);

static struct i2c_driver adsd3500_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(adsd3500_of_match),
		.name		= "adsd3500",
	},
	.probe			= adsd3500_probe,
	.remove			= adsd3500_remove,
	.id_table 		= adsd3500_id,
};

module_i2c_driver(adsd3500_i2c_driver);

MODULE_DESCRIPTION("Analog Devices ADSD3500 Driver");
MODULE_AUTHOR("Bogdan Togorean");
MODULE_LICENSE("GPL v2");
