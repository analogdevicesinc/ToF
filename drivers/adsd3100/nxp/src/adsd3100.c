// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the Analog Devices ADSD3100 ToF camera sensor.
 *
 * Copyright (C) 2019-2020 Analog Devices, All Rights Reserved.
 *
 */

#include <linux/gpio/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pwm.h>
#include <linux/firmware.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define FW_FILE_NAME	"adi/adsd3100-fw.bin"
#define ADI_MAGIC	"ADDICMOS"

static bool fw_load = false;
module_param(fw_load, bool, 0644);
MODULE_PARM_DESC(fw_load, "Boolean enabling/disabling FW loading by driver");

static bool calib_load = false;
module_param(calib_load, bool, 0644);
MODULE_PARM_DESC(calib_load, "Boolean enabling/disabling LXDAC loading by driver");

struct adsd3100_mode_info {
	u32 width;
	u32 height;
	u32 pixel_rate;
	u32 link_freq_idx;
};

struct adsd3100_mode_fw_block {
	const struct reg_sequence *mode_regs;
	ssize_t regs_count;
};

struct adsd3100_fw_header {
	unsigned char magic[8];
	__le32 modes_nr;
	__le32 data_size_bytes;
	__le16 data[];
} __packed;

struct adsd3100 {
	struct regmap *regmap;
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_rect crop;

	const struct adsd3100_mode_info *current_mode;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	/* custom controls */
	struct v4l2_ctrl *operating_mode;
	struct v4l2_ctrl *set_chip_config;

	struct mutex lock;
	bool streaming;

	struct gpio_desc *rst_gpio;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_spi, *pins_gpio;

	struct pwm_device *pwm_fsync;

	/* FW related data */
	int fw_regs_count;
	struct reg_sequence *fw_regs;
	u8 curr_operating_mode;

	const struct firmware *fw;
};

static inline struct adsd3100 *to_adsd3100(struct v4l2_subdev *sd)
{
	return container_of(sd, struct adsd3100, sd);
}

#define V4L2_CID_ADSD3100_OPERATING_MODE	(V4L2_CID_USER_ADSD_BASE + 0)
#define V4L2_CID_ADSD3100_CHIP_CONFIG		(V4L2_CID_USER_ADSD_BASE + 1)

static const struct reg_sequence adsd3100_powerup_setting[] = {
	{ 0x0000, 0x1d29 },
	{ 0x0002, 0x43d1 },
	{ 0x0000, 0x1d25 },
	{ 0x0002, 0x0000 },
	{ 0x0000, 0x1c99 },
	{ 0x0002, 0x0069 },
	{ 0x0002, 0x0060 },
	{ 0x0002, 0x0061 },
	{ 0x0002, 0x0307 },
	{ 0x0002, 0x0069 },
	{ 0x0002, 0x0105 },
	{ 0x0002, 0x0000 },
	{ 0x0002, 0x0069 },
	{ 0x0002, 0x0060 },
	{ 0x0002, 0x0061 },
	{ 0x0002, 0x0307 },
	{ 0x0002, 0x0000 },
	{ 0x0002, 0x0000 },
	{ 0x0002, 0x0000 },
	{ 0x0002, 0x0303 },
	{ 0x0002, 0x0069 },
	{ 0x0002, 0x0060 },
	{ 0x0002, 0x0061 },
	{ 0x0002, 0x0307 },
	{ 0x0002, 0x0060 },
	{ 0x0002, 0x0059 },
	{ 0x0002, 0x0059 },
	{ 0x0002, 0x0301 },
	{ 0x0002, 0x0000 },
	{ 0x0002, 0x006f },
	{ 0x0002, 0x0105 },
	{ 0x0002, 0x006e },
	{ 0x0002, 0x0067 },
	{ 0x0002, 0x0063 },
	{ 0x0002, 0x0301 },
	{ 0x0002, 0x0068 },
	{ 0x0002, 0x006e },
	{ 0x0002, 0x0205 },
	{ 0x0002, 0x646b },
	{ 0x0002, 0x0021 },
	{ 0x0172, 0x0a0a },
	{ 0x0132, 0x0047 },
	{ 0x0126, 0x0020 },
	{ 0x0132, 0x0088 },
	{ 0x0126, 0x0001 },
	{ 0x0132, 0x001d },
	{ 0x0126, 0x0008 },
	{ 0x0132, 0x008b },
	{ 0x0126, 0x0010 },
	{ 0x0132, 0x0075 },
	{ 0x0126, 0x0040 },
	{ 0x0246, 0xc1fa },
	{ 0x0102, 0x019e },
	{ 0x0104, 0x009a },
	{ 0x012e, 0x060a },
	{ 0x0130, 0x1f10 },
	{ 0x0520, 0x1a00 },
	{ 0x0522, 0x1627 },
	{ 0x0524, 0x1321 },
	{ 0x0526, 0x0a8a },
};

static const struct reg_sequence adsd3100_powerdown_setting[] = {
};

static const struct reg_sequence adsd3100_standby_setting[] = {
};

static const s64 link_freq_tbl[] = {
	732000000,
	732000000,
	732000000,
	732000000,
	732000000,
	732000000
};

/* Elements of the structure must be ordered ascending by width & height */
static const struct adsd3100_mode_info adsd3100_mode_info_data[] = {
	{
		.width = 3840,
		.height = 216,
		.pixel_rate = 488000000,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{
		.width = 4096,
		.height = 64,
		.pixel_rate = 488000000,
		.link_freq_idx = 1 /* an index in link_freq_tbl[] */
	},
	{
		.width = 4096,
		.height = 256,
		.pixel_rate = 488000000,
		.link_freq_idx = 2 /* an index in link_freq_tbl[] */
	},
	{
		.width = 4096,
		.height = 640,
		.pixel_rate = 488000000,
		.link_freq_idx = 3 /* an index in link_freq_tbl[] */
	},
	{
		.width = 4096,
		.height = 2304,
		.pixel_rate = 488000000,
		.link_freq_idx = 4 /* an index in link_freq_tbl[] */
	},
	{
		.width = 4096,
		.height = 2560,
		.pixel_rate = 488000000,
		.link_freq_idx = 5 /* an index in link_freq_tbl[] */
	}
};

static int adsd3100_regmap_write(void *context, const void *data, size_t count)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);

	return spi_write(spi, data, count);
}

static int adsd3100_regmap_read(void *context,
				const void *reg, size_t reg_size,
				void *val, size_t val_size)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	u16 *rx_buf;

	/* First response word should be ignored so allocate more & discard */
	rx_buf = (u16*)kmalloc(val_size + sizeof(u16), GFP_KERNEL);
	if (!rx_buf)
		return -ENOMEM;

	spi_write_then_read(spi, reg, reg_size, rx_buf, val_size + sizeof(u16));
	memcpy(val, rx_buf + 1, val_size);

	kfree(rx_buf);
	return 0;
}

static bool adsd3100_regmap_accessible_reg(struct device *dev, unsigned int reg)
{
	if (reg % 2)
		return 0;

	if ((reg >= 0x000) && (reg <= 0x294))
		return 1;
	else if ((reg >= 0x300) && (reg <= 0x83e))
		return 1;
	else if ((reg >= 0x900) && (reg <= 0xefe))
		return 1;
	else
		return 0;
}

static struct regmap_bus adsd3100_spi_bus_config = {
	.write = adsd3100_regmap_write,
	.read = adsd3100_regmap_read,
	.read_flag_mask = 0x00,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
	.val_format_endian_default = REGMAP_ENDIAN_BIG,
};

static const struct regmap_config adsd3100_spi_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.max_register = 0xefe,
	.read_flag_mask = 0x00,
	.write_flag_mask = 0x80,
	.cache_type = REGCACHE_NONE,
	.readable_reg = adsd3100_regmap_accessible_reg,
};

static int adsd3100_power_on(struct device *dev)
{
	struct spi_device *client = to_spi_device(dev);
	struct v4l2_subdev *sd = spi_get_drvdata(client);
	struct adsd3100 *adsd3100 = to_adsd3100(sd);
	unsigned int read_val;
	int i, ret;

	dev_dbg(adsd3100->dev, "Entered adsd3100_power_on\n");

	ret = regmap_write(adsd3100->regmap, 0xC, 0xC5);
	if (ret)
		dev_err(adsd3100->dev, "Could not set power up register\n");

	for (i = 0; i < 10; i ++) {
		msleep_interruptible(15);
		regmap_read(adsd3100->regmap, 0x256, &read_val);
		if (read_val == 0x2) {
			return 0;
		}
	}

	dev_err(adsd3100->dev, "Power on timed out.\n");
	dev_dbg(adsd3100->dev, "Status register 0x256 value: %x\n", read_val);
	regmap_read(adsd3100->regmap, 0x32, &read_val);
	dev_dbg(adsd3100->dev, "Status register 0x32 value: %x\n", read_val);

	return -ETIME;
}

static int adsd3100_power_off(struct device *dev)
{
	struct spi_device *client = to_spi_device(dev);
	struct v4l2_subdev *sd = spi_get_drvdata(client);
	struct adsd3100 *adsd3100 = to_adsd3100(sd);
	unsigned int read_val;
	int i, ret;

	if (!adsd3100->streaming)
		return 0;

	dev_dbg(adsd3100->dev, "Entered adsd3100_power_off\n");

	ret = regmap_write(adsd3100->regmap, 0xC, 0x2);
	if (ret)
		dev_err(adsd3100->dev, "Could not set power down register\n");

	for (i = 0; i < 5; i ++) {
		msleep_interruptible(100);
		regmap_read(adsd3100->regmap, 0xC, &read_val);
		if (read_val == 0x0) {
			return 0;
		}
	}

	pwm_disable(adsd3100->pwm_fsync);
	dev_err(adsd3100->dev, "Power off timed out.\n");
	regmap_read(adsd3100->regmap, 0x32, &read_val);
	dev_dbg(adsd3100->dev, "Status register 0x32 value: %x\n", read_val);

	return -ETIME;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int adsd3100_g_register(struct v4l2_subdev *sd,
			       struct v4l2_dbg_register *reg)
{
	struct adsd3100 *adsd3100 = to_adsd3100(sd);
	unsigned int read_val;
	int ret;

	reg->size = 2;
	ret = regmap_read(adsd3100->regmap, reg->reg, &read_val);
	reg->val = read_val;

	return ret;
}

static int adsd3100_s_register(struct v4l2_subdev *sd,
			       const struct v4l2_dbg_register *reg)
{
	struct adsd3100 *adsd3100 = to_adsd3100(sd);

	return regmap_write(adsd3100->regmap, reg->reg, reg->val);
}
#endif

static int adsd3100_s_power(struct v4l2_subdev *sd, int on)
{
	struct adsd3100 *adsd3100 = to_adsd3100(sd);

	dev_dbg(adsd3100->dev, "%s: %d\n", __func__, on);
	return 0;
}

static int adsd3100_chip_config(struct adsd3100 *adsd3100,
				    struct v4l2_ctrl *ctrl)
{
	uint16_t *val, *reg, val_nr, i;
	bool burst;
	int ret;

	burst = *(ctrl->p_new.p_u16);
	val_nr = *(ctrl->p_new.p_u16 + 1);
	val = ctrl->p_new.p_u16 + 2;
	reg = val + val_nr;

	dev_dbg(adsd3100->dev, "Entered adsd3100_chip_config. ADDR: %x VAL_NR %d \n", *reg, val_nr);

	if (*reg & 0x8000) { /* 0x8000 - write mask */
		if (burst)
			ret = regmap_bulk_write(adsd3100->regmap, *reg, val, val_nr);
		else
			for (i=0; i<val_nr; i++)
				ret = regmap_bulk_write(adsd3100->regmap, *(reg + i), val + i, 1);
		if (ret)
			dev_warn(adsd3100->dev,
				"could not write to register %x\n", *reg);
	} else {
		if (burst)
			ret = regmap_bulk_read(adsd3100->regmap, *reg, val, val_nr);
		else
			for (i=0; i<val_nr; i++)
				ret = regmap_bulk_read(adsd3100->regmap, *(reg + i), val + i, 1);
		if (ret)
			dev_warn(adsd3100->dev,
				"could not read from register %x\n", *reg);
	}

	return 0;
}

static int adsd3100_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct adsd3100 *adsd3100 = container_of(ctrl->handler,
						 struct adsd3100, ctrls);
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_ADSD3100_OPERATING_MODE:
		adsd3100->curr_operating_mode = ctrl->val;
		break;
	case V4L2_CID_ADSD3100_CHIP_CONFIG:
		ret = adsd3100_chip_config(adsd3100, ctrl);
		break;
	case V4L2_CID_PIXEL_RATE:
	case V4L2_CID_LINK_FREQ:
		break;
	default:
		dev_err(adsd3100->dev, "%s > Unhandled: %x  param=%x\n",
			__func__, ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops adsd3100_ctrl_ops = {
	.s_ctrl = adsd3100_s_ctrl,
};

static const struct v4l2_ctrl_config adsd3100_ctrl_chip_config = {
	.ops		= &adsd3100_ctrl_ops,
	.id		= V4L2_CID_ADSD3100_CHIP_CONFIG,
	.name		= "Chip Config",
	.type		= V4L2_CTRL_TYPE_U16,
	.def		= 0xFF,
	.min		= 0x00,
	.max		= 0xFFFF,
	.step		= 1,
	.dims		= { 65537 },
};

static const struct v4l2_ctrl_config adsd3100_ctrl_operating_mode = {
	.ops		= &adsd3100_ctrl_ops,
	.id		= V4L2_CID_ADSD3100_OPERATING_MODE,
	.name		= "Operating Mode",
	.type		= V4L2_CTRL_TYPE_INTEGER,
	.def		= 0,
	.min		= 0,
	.max		= 10,
	.step		= 1,
};

static int adsd3100_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SBGGR12_1X12;

	return 0;
}

static int adsd3100_enum_frame_size(struct v4l2_subdev *subdev,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->code != MEDIA_BUS_FMT_SBGGR12_1X12)
		return -EINVAL;

	if (fse->index >= ARRAY_SIZE(adsd3100_mode_info_data))
		return -EINVAL;

	fse->min_width = adsd3100_mode_info_data[fse->index].width;
	fse->max_width = adsd3100_mode_info_data[fse->index].width;
	fse->min_height = adsd3100_mode_info_data[fse->index].height;
	fse->max_height = adsd3100_mode_info_data[fse->index].height;

	return 0;
}

static struct v4l2_mbus_framefmt *
adsd3100_get_pad_format(struct adsd3100 *adsd3100,
			struct v4l2_subdev_pad_config *cfg, unsigned int pad,
			enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(&adsd3100->sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &adsd3100->fmt;
	default:
		return ERR_PTR(-EINVAL);
	}
}

static int adsd3100_get_format(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_format *format)
{
	struct adsd3100 *adsd3100 = to_adsd3100(sd);
	struct v4l2_mbus_framefmt *pad_format;

	pad_format = adsd3100_get_pad_format(adsd3100, cfg, format->pad,
					     format->which);
	if (IS_ERR(pad_format))
		return PTR_ERR(pad_format);

	format->format = *pad_format;

	return 0;
}

static struct v4l2_rect *
adsd3100_get_pad_crop(struct adsd3100 *adsd3100,
		      struct v4l2_subdev_pad_config *cfg,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&adsd3100->sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &adsd3100->crop;
	default:
		return ERR_PTR(-EINVAL);
	}
}

static int adsd3100_set_format(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_format *format)
{
	struct adsd3100 *adsd3100 = to_adsd3100(sd);
	struct v4l2_mbus_framefmt *framefmt;
	struct v4l2_rect *crop;
	const struct adsd3100_mode_info *new_mode;
	int ret;

	dev_dbg(adsd3100->dev, "set_fmt: %x %dx%d %d\n",
		format->format.code, format->format.width,
		format->format.height, format->which);

	mutex_lock(&adsd3100->lock);

	crop = adsd3100_get_pad_crop(adsd3100, cfg, format->pad,
				     format->which);
	if (IS_ERR(crop))
		return PTR_ERR(crop);

	new_mode = v4l2_find_nearest_size(adsd3100_mode_info_data,
					  ARRAY_SIZE(adsd3100_mode_info_data),
					  width, height, format->format.width,
					  format->format.height);
	crop->width = new_mode->width;
	crop->height = new_mode->height;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		ret = v4l2_ctrl_s_ctrl_int64(adsd3100->pixel_rate,
					     new_mode->pixel_rate);
		if (ret < 0)
			return ret;

		ret = v4l2_ctrl_s_ctrl(adsd3100->link_freq,
				       new_mode->link_freq_idx);
		if (ret < 0)
			return ret;

		adsd3100->current_mode = new_mode;
	}

	framefmt = adsd3100_get_pad_format(adsd3100, cfg, format->pad,
					   format->which);
	if (IS_ERR(framefmt))
		return PTR_ERR(framefmt);

	framefmt->width = crop->width;
	framefmt->height = crop->height;
	framefmt->code = MEDIA_BUS_FMT_SBGGR12_1X12;
	framefmt->field = V4L2_FIELD_NONE;
	framefmt->colorspace = V4L2_COLORSPACE_RAW;

	format->format = *framefmt;

	mutex_unlock(&adsd3100->lock);

	return 0;
}

static int adsd3100_entity_init_cfg(struct v4l2_subdev *subdev,
				    struct v4l2_subdev_pad_config *cfg)
{
	struct v4l2_subdev_format fmt = { 0 };

	fmt.which = cfg ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.format.width = adsd3100_mode_info_data[0].width;
	fmt.format.height = adsd3100_mode_info_data[0].height;

	adsd3100_set_format(subdev, cfg, &fmt);

	return 0;
}

static int adsd3100_get_selection(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_selection *sel)
{
	struct adsd3100 *adsd3100 = to_adsd3100(sd);
	struct v4l2_rect *crop;

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	crop = adsd3100_get_pad_crop(adsd3100, cfg, sel->pad, sel->which);
	if (IS_ERR(crop))
		return PTR_ERR(crop);

	sel->r = *crop;

	return 0;
}

static int adsd3100_start_streaming(struct adsd3100 *adsd3100)
{
	unsigned int read_val;
	int ret;

	regmap_read(adsd3100->regmap, 0x210, &read_val);
	if (read_val != adsd3100->curr_operating_mode) {
		ret = regmap_write(adsd3100->regmap, 0x200,
				   adsd3100->curr_operating_mode);
		if (ret)
			dev_err(adsd3100->dev, "Could not set mode register\n");
		/* Reduce line width only for mode 1 */
		if (adsd3100->curr_operating_mode == 1)
			ret = regmap_write(adsd3100->regmap, 0x27e,0x8006);
		else
			ret = regmap_write(adsd3100->regmap, 0x27e,0x0);
	}

	ret = pwm_enable(adsd3100->pwm_fsync);
	if (ret)
		dev_err(adsd3100->dev, "Could not enable FSYNC PWM\n");

	adsd3100->streaming = true;

	return ret;
}

static int adsd3100_stop_streaming(struct adsd3100 *adsd3100)
{
	pwm_disable(adsd3100->pwm_fsync);
	adsd3100->streaming = false;

	return 0;
}

static int adsd3100_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct adsd3100 *adsd3100 = to_adsd3100(subdev);
	int ret = 0;

	dev_dbg(adsd3100->dev, "s_stream: %d\n", enable);

	mutex_lock(&adsd3100->lock);
	if (adsd3100->streaming == enable) {
		mutex_unlock(&adsd3100->lock);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(adsd3100->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(adsd3100->dev);
			goto err_unlock;
		}

		ret = adsd3100_start_streaming(adsd3100);
		if (ret)
			goto err_rpm_put;
	} else {
		adsd3100_stop_streaming(adsd3100);
		pm_runtime_put(adsd3100->dev);
	}

	mutex_unlock(&adsd3100->lock);

	return ret;

err_rpm_put:
	pm_runtime_put(adsd3100->dev);
err_unlock:
	mutex_unlock(&adsd3100->lock);

	return ret;
}

static int adsd3100_g_frame_interval(struct v4l2_subdev *subdev,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct adsd3100 *adsd3100 = to_adsd3100(subdev);
	struct pwm_state state;

	/* Sync up PWM state. */
	pwm_init_state(adsd3100->pwm_fsync, &state);

	fi->interval.numerator = 1;
	fi->interval.denominator =
		(u32)(NSEC_PER_SEC / pwm_get_period(adsd3100->pwm_fsync));
	dev_dbg(adsd3100->dev, "%s frame rate = %u / %u\n", __func__,
		fi->interval.numerator, fi->interval.denominator);

	return 0;
}

static int adsd3100_s_frame_interval(struct v4l2_subdev *subdev,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct adsd3100 *adsd3100 = to_adsd3100(subdev);
	struct pwm_state state;
	int ret = 0;

	mutex_lock(&adsd3100->lock);

	/* Get PWM state. */
	pwm_init_state(adsd3100->pwm_fsync, &state);
	state.period = DIV_ROUND_UP(fi->interval.numerator * NSEC_PER_SEC,
				    fi->interval.denominator);
	pwm_set_relative_duty_cycle(&state, 50, 100);
	ret = pwm_apply_state(adsd3100->pwm_fsync, &state);

	dev_dbg(adsd3100->dev, "Set frame interval to %u / %u\n",
		fi->interval.numerator, fi->interval.denominator);

	mutex_unlock(&adsd3100->lock);

	return ret;
}

static int adsd3100_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct v4l2_subdev_core_ops adsd3100_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= adsd3100_g_register,
	.s_register	= adsd3100_s_register,
#endif
	.s_power = adsd3100_s_power,
};

static const struct dev_pm_ops adsd3100_pm_ops = {
	SET_RUNTIME_PM_OPS(adsd3100_power_off, adsd3100_power_on, NULL)
};

static const struct v4l2_subdev_video_ops adsd3100_video_ops = {
	.s_stream		= adsd3100_s_stream,
	.g_frame_interval	= adsd3100_g_frame_interval,
	.s_frame_interval	= adsd3100_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops adsd3100_subdev_pad_ops = {
	.init_cfg		= adsd3100_entity_init_cfg,
	.enum_mbus_code		= adsd3100_enum_mbus_code,
	.enum_frame_size	= adsd3100_enum_frame_size,
	.get_fmt		= adsd3100_get_format,
	.set_fmt		= adsd3100_set_format,
	.get_selection		= adsd3100_get_selection,
};

static const struct v4l2_subdev_ops adsd3100_subdev_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.core	= &adsd3100_core_ops,
#endif
	.video	= &adsd3100_video_ops,
	.pad	= &adsd3100_subdev_pad_ops,
};

static const struct media_entity_operations adsd3100_subdev_entity_ops = {
	.link_setup = adsd3100_link_setup,
};

static int adsd3100_spi_bus_init(struct v4l2_subdev *sd)
{
	struct adsd3100 *adsd3100 = to_adsd3100(sd);
	unsigned int read_val;
	int ret;

	/* Set SPI pins to GPIO mode */
	ret = pinctrl_select_state(adsd3100->pinctrl, adsd3100->pins_gpio);
	if (ret < 0)
		return ret;

	gpiod_set_value_cansleep(adsd3100->rst_gpio, 1);

	msleep(500);

	gpiod_set_value_cansleep(adsd3100->rst_gpio, 0);

	/* Set SPI pins to SPI mode */
	ret = pinctrl_select_state(adsd3100->pinctrl, adsd3100->pins_spi);
	if (ret < 0)
		return ret;

	ret = regmap_read(adsd3100->regmap, 0x112, &read_val);
	if (ret < 0) {
		dev_err(adsd3100->dev, "Read of Chip ID register failed.\n");
		return ret;
	}

	if (read_val != 0x5931) {
		dev_err(adsd3100->dev, "Chip ID: %.4X is wrong.\n", read_val);
		return -ENXIO;
	}
	dev_dbg(adsd3100->dev, "Read Chip ID: %.4X\n", read_val);

	return 0;
}

static int adsd3100_g_sensor_firmware(struct v4l2_subdev *sd)
{
	struct adsd3100 *adsd3100 = to_adsd3100(sd);
	const struct firmware *fw = adsd3100->fw;
	const struct adsd3100_fw_header *fw_head;
	unsigned int reg_nr, modes_nr, data_size_bytes, i, j;

	if (fw->size < sizeof(struct adsd3100_fw_header) ||
	    fw->size >= 0x4000000)
		goto err_size;

	fw_head = (struct adsd3100_fw_header *)fw->data;

	if (memcmp(fw_head->magic, ADI_MAGIC, ARRAY_SIZE(fw_head->magic))) {
		dev_err(adsd3100->dev, "FW loading failed: Invalid magic\n");
		return -EINVAL;
	}

	modes_nr = le32_to_cpu(fw_head->modes_nr);
	if (modes_nr == 0) {
		dev_err(adsd3100->dev, "FW should contain at least 1 mode.\n");
		return -EINVAL;
	}

	__v4l2_ctrl_modify_range(adsd3100->operating_mode,
				 adsd3100->operating_mode->minimum,
				 modes_nr - 1, 1, 0);

	data_size_bytes = le32_to_cpu(fw_head->data_size_bytes);
	reg_nr = data_size_bytes / sizeof(uint16_t) / 2;
	adsd3100->fw_regs_count = reg_nr;

	adsd3100->fw_regs = devm_kcalloc(adsd3100->dev, reg_nr,
			       sizeof(struct reg_sequence), GFP_KERNEL);
	if (!adsd3100->fw_regs)
		return -ENOMEM;

	for (i = 0, j = 0; i < reg_nr * 2; i += 2, j++) {
		adsd3100->fw_regs[j].reg = le16_to_cpu(fw_head->data[i]);
		adsd3100->fw_regs[j].def = le16_to_cpu(fw_head->data[i + 1]);
	}

	return 0;

err_size:
	dev_err(adsd3100->dev, "FW loading failed: Invalid size\n");
	return -EINVAL;
}

static int adsd3100_firmware_load(struct v4l2_subdev *sd)
{
	struct adsd3100 *adsd3100 = to_adsd3100(sd);
	int ret = 0;

	if (fw_load) {
		ret = request_firmware(&adsd3100->fw, FW_FILE_NAME, adsd3100->dev);
		if (ret < 0) {
			dev_err(adsd3100->dev, "FW request failed\n");
			goto release_firmware;
		}

		ret = adsd3100_g_sensor_firmware(sd);
		if (ret < 0) {
			dev_err(adsd3100->dev, "FW parsing failed\n");
			goto release_firmware;
		}

		/* Writes for Default firmware */
		regmap_multi_reg_write(adsd3100->regmap, adsd3100->fw_regs,
					     adsd3100->fw_regs_count);
	}

	if (calib_load) {
		/* Writes for Default calibration */
		regmap_multi_reg_write(adsd3100->regmap,
				       adsd3100_powerup_setting,
				       ARRAY_SIZE(adsd3100_powerup_setting));
	}

release_firmware:
	release_firmware(adsd3100->fw);
	return ret;
}

static int adsd3100_probe(struct spi_device *client)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct adsd3100 *adsd3100;
	struct pwm_state state;
	int ret;

	adsd3100 = devm_kzalloc(dev, sizeof(struct adsd3100), GFP_KERNEL);
	if (!adsd3100)
		return -ENOMEM;

	adsd3100->dev = dev;

	adsd3100->regmap = devm_regmap_init(dev, &adsd3100_spi_bus_config, dev,
					    &adsd3100_spi_regmap_config);
	if (IS_ERR(adsd3100->regmap)) {
		dev_err(dev, "Error initializing spi regmap\n");
		return PTR_ERR(adsd3100->regmap);
	}

	mutex_init(&adsd3100->lock);

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(endpoint, &bus_cfg);
	fwnode_handle_put(endpoint);
	if (ret < 0) {
		dev_err(dev, "parsing endpoint node failed\n");
		return ret;
	}

	adsd3100->rst_gpio = gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(adsd3100->rst_gpio)) {
		dev_err(dev, "Unable to get \"reset\" gpio\n");
		return PTR_ERR(adsd3100->rst_gpio);
	}

	adsd3100->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(adsd3100->pinctrl)) {
		dev_err(dev, "Unable to get \"pinctrl\" block\n");
		return PTR_ERR(adsd3100->pinctrl);
	}

	adsd3100->pins_spi = pinctrl_lookup_state(adsd3100->pinctrl, "spi");
	if (IS_ERR(adsd3100->pins_spi)) {
		dev_err(dev, "Unable to get \"spi-pinctrl\" block\n");
		return PTR_ERR(adsd3100->pins_spi);
	}

	adsd3100->pins_gpio = pinctrl_lookup_state(adsd3100->pinctrl, "gpio");
	if (IS_ERR(adsd3100->pins_gpio)) {
		dev_err(dev, "Unable to get \"gpio-pinctrl\" block\n");
		return PTR_ERR(adsd3100->pins_gpio);
	}

	ret = adsd3100_spi_bus_init(&adsd3100->sd);
	if (ret) {
		dev_err(dev, "%s: SPI bus initialization failed %d\n",
			__func__, ret);
		goto release_gpio;
	}

	adsd3100->pwm_fsync = devm_pwm_get(dev, NULL);
	if (IS_ERR(adsd3100->pwm_fsync)) {
		dev_err(dev, "Failed to get pwm device\n");
		return PTR_ERR(adsd3100->pwm_fsync);
	}

	/* Get PWM state. */
	pwm_init_state(adsd3100->pwm_fsync, &state);
	pwm_set_relative_duty_cycle(&state, 50, 100);
	ret = pwm_apply_state(adsd3100->pwm_fsync, &state);
	if (ret) {
		dev_err(dev, "%s: PWM init failed %d\n",
			__func__, ret);
		goto release_gpio;
	}

	v4l2_ctrl_handler_init(&adsd3100->ctrls, 3);

	adsd3100->pixel_rate = v4l2_ctrl_new_std(&adsd3100->ctrls,
						  &adsd3100_ctrl_ops,
						  V4L2_CID_PIXEL_RATE,
						  1, INT_MAX, 1, 1);
	adsd3100->link_freq = v4l2_ctrl_new_int_menu(&adsd3100->ctrls,
						     &adsd3100_ctrl_ops,
						     V4L2_CID_LINK_FREQ,
						     ARRAY_SIZE(
							     link_freq_tbl) - 1,
						     0, link_freq_tbl);
	if (adsd3100->link_freq)
		adsd3100->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	adsd3100->operating_mode = v4l2_ctrl_new_custom(&adsd3100->ctrls,
						&adsd3100_ctrl_operating_mode,
						NULL);

	adsd3100->set_chip_config = v4l2_ctrl_new_custom(&adsd3100->ctrls,
						&adsd3100_ctrl_chip_config,
						NULL);

	ret = adsd3100->ctrls.error;
	if (ret) {
		dev_err(dev, "%s: control initialization error %d\n",
			__func__, ret);
		goto free_ctrl;
	}
	adsd3100->sd.ctrl_handler = &adsd3100->ctrls;

	v4l2_spi_subdev_init(&adsd3100->sd, client, &adsd3100_subdev_ops);
	adsd3100->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	adsd3100->pad.flags = MEDIA_PAD_FL_SOURCE;
	adsd3100->sd.dev = &client->dev;
	adsd3100->sd.entity.ops = &adsd3100_subdev_entity_ops;
	adsd3100->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = media_entity_pads_init(&adsd3100->sd.entity, 1, &adsd3100->pad);
	if (ret < 0) {
		dev_err(dev, "could not register media entity\n");
		goto free_ctrl;
	}

	ret = adsd3100_firmware_load(&adsd3100->sd);
	if (ret < 0)
		return ret;

	adsd3100_entity_init_cfg(&adsd3100->sd, NULL);

	ret = v4l2_async_register_subdev(&adsd3100->sd);
	if (ret < 0) {
		dev_err(dev, "could not register v4l2 device\n");
		goto free_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

free_entity:
	media_entity_cleanup(&adsd3100->sd.entity);
free_ctrl:
	v4l2_ctrl_handler_free(&adsd3100->ctrls);
	mutex_destroy(&adsd3100->lock);
release_gpio:
	gpiod_put(adsd3100->rst_gpio);

	return ret;
}

static int adsd3100_remove(struct spi_device *client)
{
	struct v4l2_subdev *sd = spi_get_drvdata(client);
	struct adsd3100 *adsd3100 = to_adsd3100(sd);

	v4l2_async_unregister_subdev(&adsd3100->sd);
	media_entity_cleanup(&adsd3100->sd.entity);
	gpiod_put(adsd3100->rst_gpio);
	v4l2_ctrl_handler_free(&adsd3100->ctrls);
	mutex_destroy(&adsd3100->lock);

	pm_runtime_disable(adsd3100->dev);
	if (!pm_runtime_status_suspended(adsd3100->dev))
		adsd3100_power_off(adsd3100->dev);
	pm_runtime_set_suspended(adsd3100->dev);

	return 0;
}

static const struct of_device_id adsd3100_of_match[] = {
	{ .compatible = "adi,adsd3100" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, adsd3100_of_match);

static const struct spi_device_id adsd3100_ids[] = {
	{ "adsd3100", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(spi, adsd3100_ids);

static struct spi_driver adsd3100_spi_driver = {
	.driver			= {
		.of_match_table = adsd3100_of_match,
		.name		= "adsd3100",
		.pm		= &adsd3100_pm_ops,
	},
	.probe			= adsd3100_probe,
	.remove			= adsd3100_remove,
	.id_table		= adsd3100_ids,
};

module_spi_driver(adsd3100_spi_driver);

MODULE_DESCRIPTION("Analog Devices ADSD3100 Camera Driver");
MODULE_AUTHOR("Bogdan Togorean");
MODULE_LICENSE("GPL v2");
