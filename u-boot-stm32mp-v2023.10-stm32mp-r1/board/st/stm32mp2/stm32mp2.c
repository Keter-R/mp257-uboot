// SPDX-License-Identifier: GPL-2.0-or-later OR BSD-3-Clause
/*
 * Copyright (C) 2023, STMicroelectronics - All Rights Reserved
 */

#define LOG_CATEGORY LOGC_BOARD

#include <common.h>
#include <button.h>
#include <config.h>
#include <dm.h>
#include <efi_loader.h>
#include <env.h>
#include <env_internal.h>
#include <fdt_simplefb.h>
#include <fdt_support.h>
#include <g_dnl.h>
#include <i2c.h>
#include <led.h>
#include <log.h>
#include <misc.h>
#include <mmc.h>
#include <init.h>
#include <net.h>
#include <netdev.h>
#include <phy.h>
#include <regmap.h>
#include <syscon.h>
#include <asm/io.h>
#include <asm/global_data.h>
#include <asm/gpio.h>
#include <asm/arch/sys_proto.h>
#include <dm/device.h>
#include <dm/device-internal.h>
#include <dm/ofnode.h>
#include <dm/uclass.h>
#include <dt-bindings/gpio/gpio.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/iopoll.h>

#define SYSCFG_ETHCR_ETH_SEL_MII	0
#define SYSCFG_ETHCR_ETH_SEL_RGMII	BIT(4)
#define SYSCFG_ETHCR_ETH_SEL_RMII	BIT(6)
#define SYSCFG_ETHCR_ETH_CLK_SEL	BIT(1)
#define SYSCFG_ETHCR_ETH_REF_CLK_SEL	BIT(0)
/* CLOCK feed to PHY*/
#define ETH_CK_F_25M	25000000
#define ETH_CK_F_50M	50000000
#define ETH_CK_F_125M	125000000

#define ILITEK_REG_ID		0x40
#define ILITEK_ID_LEN		7
#define ADV7511_REG_CHIP_REVISION	0x00
#define ADV7511_CHIP_REVISION_LEN	256

#if CONFIG_IS_ENABLED(EFI_HAVE_CAPSULE_SUPPORT)
struct efi_fw_image fw_images[1];

struct efi_capsule_update_info update_info = {
	.num_images = ARRAY_SIZE(fw_images),
	.images = fw_images,
};

u8 num_image_type_guids = ARRAY_SIZE(fw_images);
#endif /* EFI_HAVE_CAPSULE_SUPPORT */

/*
 * Get a global data pointer
 */
DECLARE_GLOBAL_DATA_PTR;

int checkboard(void)
{
	int ret;
	u32 otp;
	struct udevice *dev;
	const char *fdt_compat;
	int fdt_compat_len;

	fdt_compat = ofnode_get_property(ofnode_root(), "compatible", &fdt_compat_len);

	log_info("Board: stm32mp2 (%s)\n", fdt_compat && fdt_compat_len ? fdt_compat : "");

	/* display the STMicroelectronics board identification */
	if (CONFIG_IS_ENABLED(CMD_STBOARD)) {
		ret = uclass_get_device_by_driver(UCLASS_MISC,
						  DM_DRIVER_GET(stm32mp_bsec),
						  &dev);
		if (!ret)
			ret = misc_read(dev, STM32_BSEC_SHADOW(BSEC_OTP_BOARD),
					&otp, sizeof(otp));
		if (ret > 0 && otp)
			log_info("Board: MB%04x Var%d.%d Rev.%c-%02d\n",
				 otp >> 16,
				 (otp >> 12) & 0xF,
				 (otp >> 4) & 0xF,
				 ((otp >> 8) & 0xF) - 1 + 'A',
				 otp & 0xF);
	}

	return 0;
}

#ifdef CONFIG_USB_GADGET_DOWNLOAD
#define STM32MP1_G_DNL_DFU_PRODUCT_NUM 0xdf11
#define STM32MP1_G_DNL_FASTBOOT_PRODUCT_NUM 0x0afb

int g_dnl_bind_fixup(struct usb_device_descriptor *dev, const char *name)
{
	if (IS_ENABLED(CONFIG_DFU_OVER_USB) &&
	    !strcmp(name, "usb_dnl_dfu"))
		put_unaligned(STM32MP1_G_DNL_DFU_PRODUCT_NUM, &dev->idProduct);
	else if (IS_ENABLED(CONFIG_FASTBOOT) &&
		 !strcmp(name, "usb_dnl_fastboot"))
		put_unaligned(STM32MP1_G_DNL_FASTBOOT_PRODUCT_NUM,
			      &dev->idProduct);
	else
		put_unaligned(CONFIG_USB_GADGET_PRODUCT_NUM, &dev->idProduct);

	return 0;
}
#endif /* CONFIG_USB_GADGET_DOWNLOAD */

/* touchscreen driver: only used for pincontrol configuration */
static const struct udevice_id touchscreen_ids[] = {
	{ .compatible = "ilitek,ili251x", },
	{ }
};

U_BOOT_DRIVER(touchscreen) = {
	.name		= "touchscreen",
	.id		= UCLASS_I2C_GENERIC,
	.of_match	= touchscreen_ids,
};

static int i2c_read(ofnode node, u16 reg, u8 *buf, int len, uint wlen)
{
	ofnode bus_node;
	struct udevice *dev;
	struct udevice *bus;
	struct i2c_msg msgs[2];
	u32 chip_addr;
	__be16 wbuf;
	int ret;

	/* parent should be an I2C bus */
	bus_node = ofnode_get_parent(node);
	ret = uclass_get_device_by_ofnode(UCLASS_I2C, bus_node, &bus);
	if (ret) {
		log_debug("can't find I2C bus for node %s\n", ofnode_get_name(bus_node));
		return ret;
	}

	ret = ofnode_read_u32(node, "reg", &chip_addr);
	if (ret) {
		log_debug("can't read I2C address in %s\n", ofnode_get_name(node));
		return ret;
	}

	ret = dm_i2c_probe(bus, chip_addr, 0, &dev);
	if (ret)
		return false;

	if (wlen == 2)
		wbuf = cpu_to_be16(reg);
	else
		wbuf = reg;

	msgs[0].flags = 0;
	msgs[0].addr  = chip_addr;
	msgs[0].len   = wlen;
	msgs[0].buf   = (u8 *)&wbuf;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = chip_addr;
	msgs[1].len   = len;
	msgs[1].buf   = buf;

	ret = dm_i2c_xfer(dev, msgs, 2);

	return ret;
}

static bool reset_gpio(ofnode node)
{
	struct gpio_desc reset_gpio;

	gpio_request_by_name_nodev(node, "reset-gpios", 0, &reset_gpio, GPIOD_IS_OUT);

	if (!dm_gpio_is_valid(&reset_gpio))
		return false;

	dm_gpio_set_value(&reset_gpio, true);
	mdelay(1);
	dm_gpio_set_value(&reset_gpio, false);
	mdelay(10);

	dm_gpio_free(NULL, &reset_gpio);

	return true;
}

/* HELPER: search detected driver */
struct detect_info_t {
	bool (*detect)(void);
	char *compatible;
};

static const char *detect_device(const struct detect_info_t *info, u8 size)
{
	u8 i;

	for (i = 0; i < size; i++) {
		if (info[i].detect())
			return info[i].compatible;
	}

	return NULL;
}

bool detect_stm32mp25x_etml0700zxxdha(void)
{
	ofnode node;
	char id[ILITEK_ID_LEN];
	int ret;

	node = ofnode_by_compatible(ofnode_null(), "ilitek,ili251x");
	if (!ofnode_valid(node))
		return false;

	if (!reset_gpio(node))
		return false;

	mdelay(200);

	ret = i2c_read(node, ILITEK_REG_ID, id, sizeof(id), 1);
	if (ret)
		return false;

	/* FW panel ID is starting at the 4th byte */
	if (!strncmp(&id[4], "WSV", sizeof(id) - 4))
		return true;

	return false;
}

bool detect_stm32mp25x_adv753x(void)
{
	ofnode node;
	char id[ADV7511_CHIP_REVISION_LEN];

	node = ofnode_by_compatible(ofnode_null(),  "adi,adv7533");
	if (!ofnode_valid(node)) {
		node = ofnode_by_compatible(ofnode_null(),  "adi,adv7535");
		if (!ofnode_valid(node))
			return false;
	}

	if (!reset_gpio(node))
		return false;

	mdelay(10);

	i2c_read(node, ADV7511_REG_CHIP_REVISION, id, sizeof(id), 1);

	if (id[0] == 0x14)
		return true;

	return false;
}

static const struct detect_info_t stm32mp25x_panels[] = {
	{
		.detect = detect_stm32mp25x_etml0700zxxdha,
		.compatible = "edt,etml0700z9ndha",
	},
};

static const struct detect_info_t stm32mp25x_bridges[] = {
	{
		.detect = detect_stm32mp25x_adv753x,
		.compatible = "adi,adv7533",
	},
	{
		.detect = detect_stm32mp25x_adv753x,
		.compatible = "adi,adv7535",
	},
};

static void board_stm32mp25x_eval_init(void)
{
	const char *compatible;
	struct udevice *dev;
	ofnode node;

	/* auto detection of connected panels */
	compatible = detect_device(stm32mp25x_panels, ARRAY_SIZE(stm32mp25x_panels));

	if (!compatible) {
		/* remove the panel in environment */
		env_set("panel", "");

		/* no panel detected then unbind lvds to avoid a bad clock tree */
		node = ofnode_by_compatible(ofnode_null(), "st,stm32mp25-lvds");
		if (!ofnode_valid(node))
			return;

		device_find_global_by_ofnode(node, &dev);
		device_remove(dev, DM_REMOVE_NORMAL);
		device_unbind(dev);
	} else {
		/* save the detected compatible in environment */
		env_set("panel", compatible);
	}

	/* auto detection of connected hdmi bridge */
	compatible = detect_device(stm32mp25x_bridges, ARRAY_SIZE(stm32mp25x_bridges));

	if (!compatible)
		/* remove the hdmi bridge in environment */
		env_set("hdmi", "");
	else
		/* save the detected compatible in environment */
		env_set("hdmi", compatible);
}

static void board_stm32mp2xx_disco_init(void)
{
	const char *compatible;
	struct udevice *dev;
	ofnode node;

	/* auto detection of connected panels */
	compatible = detect_device(stm32mp25x_panels, ARRAY_SIZE(stm32mp25x_panels));

	if (!compatible) {
		/* remove the panel in environment */
		env_set("panel", "");

		/* no panel detected then unbind lvds to avoid a bad clock tree */
		node = ofnode_by_compatible(ofnode_null(), "st,stm32mp25-lvds");
		if (!ofnode_valid(node))
			return;

		device_find_global_by_ofnode(node, &dev);
		device_remove(dev, DM_REMOVE_NORMAL);
		device_unbind(dev);
	} else {
		/* save the detected compatible in environment */
		env_set("panel", compatible);
	}
}

static int get_led(struct udevice **dev, char *led_string)
{
	const char *led_name;
	int ret;

	led_name = ofnode_conf_read_str(led_string);
	if (!led_name) {
		log_debug("could not find %s config string\n", led_string);
		return -ENOENT;
	}
	ret = led_get_by_label(led_name, dev);
	if (ret) {
		log_debug("get=%d\n", ret);
		return ret;
	}

	return 0;
}

static int setup_led(enum led_state_t cmd)
{
	struct udevice *dev;
	int ret;

	if (!CONFIG_IS_ENABLED(LED))
		return 0;

	ret = get_led(&dev, "u-boot,boot-led");
	if (ret)
		return ret;

	ret = led_set_state(dev, cmd);
	return ret;
}

static void check_user_button(void)
{
	struct udevice *button1 = NULL, *button2 = NULL;
	enum forced_boot_mode boot_mode = BOOT_NORMAL;

	if (!IS_ENABLED(CONFIG_BUTTON))
		return;

	if (!IS_ENABLED(CONFIG_FASTBOOT) && !IS_ENABLED(CONFIG_CMD_STM32PROG))
		return;

	if (IS_ENABLED(CONFIG_CMD_STM32PROG))
		button_get_by_label("User-1", &button1);

	if (IS_ENABLED(CONFIG_FASTBOOT))
		button_get_by_label("User-2", &button2);

	if (!button1 && !button2)
		return;

	if (button1 && button_get_state(button1) == BUTTON_ON) {
		log_notice("STM32Programmer key pressed, ");
		boot_mode = BOOT_STM32PROG;
	}

	if (button2 && button_get_state(button2) == BUTTON_ON) {
		log_notice("Fastboot key pressed, ");
		boot_mode = BOOT_FASTBOOT;
	}

	if (boot_mode != BOOT_NORMAL) {
		log_notice("entering download mode...\n");
		clrsetbits_le32(TAMP_BOOT_CONTEXT, TAMP_BOOT_FORCED_MASK,
				boot_mode);
	}
}

static bool board_is_stm32mp257_eval(void)
{
	if (CONFIG_IS_ENABLED(TARGET_ST_STM32MP25X) &&
	    (of_machine_is_compatible("st,stm32mp257f-ev1")))
		return true;

	return false;
}

static bool board_is_stm32mp235_disco(void)
{
	if (CONFIG_IS_ENABLED(TARGET_ST_STM32MP23X) &&
	    (of_machine_is_compatible("st,stm32mp235f-dk")))
		return true;

	return false;
}

static bool board_is_stm32mp257_disco(void)
{
	if (CONFIG_IS_ENABLED(TARGET_ST_STM32MP25X) &&
	    (of_machine_is_compatible("st,stm32mp257f-dk")))
		return true;

	return false;
}

/* board dependent setup after realloc */
int board_init(void)
{
	setup_led(LEDST_ON);

	check_user_button();

#if CONFIG_IS_ENABLED(EFI_HAVE_CAPSULE_SUPPORT)
	efi_guid_t image_type_guid = STM32MP_FIP_IMAGE_GUID;

	guidcpy(&fw_images[0].image_type_id, &image_type_guid);
	fw_images[0].fw_name = u"STM32MP-FIP";
	fw_images[0].image_index = 1;
#endif

	return 0;
}

/* eth init function : weak called in eqos driver */
int board_interface_eth_init(struct udevice *dev,
			     phy_interface_t interface_type, ulong rate)
{
	struct regmap *regmap;
	uint regmap_mask, regmap_offset;
	int ret;
	u32 value;
	bool ext_phyclk, eth_clk_sel_reg;

	/* Ethernet PHY have no cristal or need to be clock by RCC */
	ext_phyclk = dev_read_bool(dev, "st,ext-phyclk");

	/* Gigabit Ethernet 125MHz clock selection. */
	eth_clk_sel_reg = dev_read_bool(dev, "st,eth-clk-sel");

	regmap = syscon_regmap_lookup_by_phandle(dev, "st,syscon");

	if (!IS_ERR(regmap)) {
		u32 fmp[3];

		ret = dev_read_u32_array(dev, "st,syscon", fmp, 3);
		if (ret) {
			pr_err("%s: Need to specify Offset and Mask of syscon register\n", __func__);
			return ret;
		}
		else {
			regmap_mask = fmp[2];
			regmap_offset = fmp[1];
		}
	} else {
		return -ENODEV;
	}
	switch (interface_type) {
	case PHY_INTERFACE_MODE_MII:
		value = SYSCFG_ETHCR_ETH_SEL_MII;
		debug("%s: PHY_INTERFACE_MODE_MII\n", __func__);
		break;
	case PHY_INTERFACE_MODE_RMII:
		if (rate == ETH_CK_F_50M && ext_phyclk)
			value = SYSCFG_ETHCR_ETH_SEL_RMII |
				SYSCFG_ETHCR_ETH_REF_CLK_SEL;
		else
			value = SYSCFG_ETHCR_ETH_SEL_RMII;
		debug("%s: PHY_INTERFACE_MODE_RMII\n", __func__);
		break;
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		if (rate == ETH_CK_F_125M && (eth_clk_sel_reg ||ext_phyclk))
			value = SYSCFG_ETHCR_ETH_SEL_RGMII |
				SYSCFG_ETHCR_ETH_CLK_SEL;
		else
			value = SYSCFG_ETHCR_ETH_SEL_RGMII;
		debug("%s: PHY_INTERFACE_MODE_RGMII\n", __func__);
		break;
	default:
		debug("%s: Do not manage %d interface\n",
		      __func__, interface_type);
		/* Do not manage others interfaces */
		return -EINVAL;
	}

	ret = regmap_update_bits(regmap, regmap_offset, regmap_mask, value);

	return ret;
}

enum env_location env_get_location(enum env_operation op, int prio)
{
	u32 bootmode = get_bootmode();

	if (prio)
		return ENVL_UNKNOWN;

	switch (bootmode & TAMP_BOOT_DEVICE_MASK) {
	case BOOT_FLASH_SD:
	case BOOT_FLASH_EMMC:
		if (CONFIG_IS_ENABLED(ENV_IS_IN_MMC))
			return ENVL_MMC;
		else
			return ENVL_NOWHERE;

	case BOOT_FLASH_NAND:
	case BOOT_FLASH_SPINAND:
		if (CONFIG_IS_ENABLED(ENV_IS_IN_UBI))
			return ENVL_UBI;
		else
			return ENVL_NOWHERE;

	case BOOT_FLASH_NOR:
		if (CONFIG_IS_ENABLED(ENV_IS_IN_SPI_FLASH))
			return ENVL_SPI_FLASH;
		else
			return ENVL_NOWHERE;

	case BOOT_FLASH_HYPERFLASH:
		if (CONFIG_IS_ENABLED(ENV_IS_IN_FLASH))
			return ENVL_FLASH;
		else
			return ENVL_NOWHERE;

	default:
		return ENVL_NOWHERE;
	}
}

int mmc_get_boot(void)
{
	struct udevice *dev;
	u32 boot_mode = get_bootmode();
	unsigned int instance = (boot_mode & TAMP_BOOT_INSTANCE_MASK) - 1;
	char cmd[20];
	const u32 sdmmc_addr[] = {
		STM32_SDMMC1_BASE,
		STM32_SDMMC2_BASE,
		STM32_SDMMC3_BASE
	};

	if (instance > ARRAY_SIZE(sdmmc_addr))
		return 0;

	/* search associated sdmmc node in devicetree */
	snprintf(cmd, sizeof(cmd), "mmc@%x", sdmmc_addr[instance]);
	if (uclass_get_device_by_name(UCLASS_MMC, cmd, &dev)) {
		log_err("mmc%d = %s not found in device tree!\n", instance, cmd);
		return 0;
	}

	return dev_seq(dev);
};

int mmc_get_env_dev(void)
{
	const int mmc_env_dev = CONFIG_IS_ENABLED(ENV_IS_IN_MMC, (CONFIG_SYS_MMC_ENV_DEV), (-1));

	if (mmc_env_dev >= 0)
		return mmc_env_dev;

	/* use boot instance to select the correct mmc device identifier */
	return mmc_get_boot();
}

/******************** ALIENTEK ADD START ********************/
#ifdef ALIENTEK_MIPI_RGB_LCD
#include <adc.h>
#define GOODIX_REG_ID		0x8140
#define GOODIX_ID_LEN		4
#define FT5426_REG_ID   	0xA3
#define FT5426_ID_LEN   	1
#define CST340_REG_ID		0x8142
#define CST340_ID_LEN		2

static const struct udevice_id atk_touchscreen_ids[] = {
	{ .compatible = "alientek,mipi-ts", },
	{ .compatible = "alientek,rgb-ts", },
	{ }
};

U_BOOT_DRIVER(atk_touchscreen) = {
	.name		= "atk_touchscreen",
	.id		= UCLASS_I2C_GENERIC,
	.of_match	= atk_touchscreen_ids,
};

static int atk_i2c_read(ofnode node, u32 chip_addr,
	u16 reg, u8 *buf, int len, uint wlen)
{
	ofnode bus_node;
	struct udevice *dev;
	struct udevice *bus;
	struct i2c_msg msgs[2];
	__be16 wbuf;
	int ret;

	/* parent should be an I2C bus */
	bus_node = ofnode_get_parent(node);
	ret = uclass_get_device_by_ofnode(UCLASS_I2C, bus_node, &bus);
	if (ret) {
		printf("cannot find I2C bus for node %s\n", ofnode_get_name(bus_node));
		return ret;
	}

	ret = dm_i2c_probe(bus, chip_addr, 0, &dev);
	if (ret)
		return false;

	if (wlen == 2)
		wbuf = cpu_to_be16(reg);
	else
		wbuf = reg;

	msgs[0].flags = 0;
	msgs[0].addr  = chip_addr;
	msgs[0].len   = wlen;
	msgs[0].buf   = (u8 *)&wbuf;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = chip_addr;
	msgs[1].len   = len;
	msgs[1].buf   = buf;

	ret = dm_i2c_xfer(dev, msgs, 2);

	return ret;
}

int goodix_reset_irq_gpio(ofnode node)
{
	struct gpio_desc reset_gpio;
	struct gpio_desc irq_gpio;

	gpio_request_by_name_nodev(node, "reset-gpios", 0, &reset_gpio, GPIOD_IS_OUT);
	gpio_request_by_name_nodev(node, "irq-gpios", 0, &irq_gpio, GPIOD_IS_OUT);

	if(!dm_gpio_is_valid(&reset_gpio)) {
		printf("%s: cannot get goodix reset-gpios\n", __func__);
		return -1;
	}

	if(!dm_gpio_is_valid(&irq_gpio)) {
		printf("%s: cannot get goodix irq-gpios\n", __func__);
		return -1;
	}

	dm_gpio_set_value(&reset_gpio, false);	//set reset gpio low
	mdelay(40);

	dm_gpio_set_value(&irq_gpio, true);		//set irq gpio high
	mdelay(1);

	dm_gpio_set_value(&reset_gpio, true);	//set reset gpio high
	mdelay(40);

	dm_gpio_free(NULL, &reset_gpio);
	dm_gpio_free(NULL, &irq_gpio);
	return 0;
}

int detect_alientek_mipi_lcd_touchscreen(void)
{
	int i, ret;
	ofnode node;
	unsigned char goodix_id[GOODIX_ID_LEN] = {0};
	unsigned int goodix_addr = 0;
	char *goodix_id_list[] = {
		"911", "1151", "9271", "9147", "928", "1158",
    };

	node = ofnode_by_compatible(ofnode_null(), "alientek,mipi-ts");
	if(!ofnode_valid(node)) {
		printf("cannot find alientek,mipi-ts compatible\n");
		return -1;
	}

	ret = ofnode_read_u32(node, "reg", &goodix_addr);
	if (ret) {
		printf("cannot read goodix I2C address in %s\n", ofnode_get_name(node));
		return ret;
	}

	ret = goodix_reset_irq_gpio(node);
	if(ret) {
		printf("error mipi lcd goodix_reset_irq_gpio\n");
		return -1;
	}

	ret = atk_i2c_read(node, goodix_addr, GOODIX_REG_ID, goodix_id, sizeof(goodix_id), 2);
	if (!ret) {
		for (i = 0; i < ARRAY_SIZE(goodix_id_list); i++) {
			if (!strncmp(goodix_id, goodix_id_list[i], sizeof(goodix_id))) {
				//printf("ts: Goodix (ID %s)\n", goodix_id);
				return 0;
			}
		}
	}

	return -1;
}

int detect_alientek_rgb_lcd_touchscreen(void)
{
	int i, ret;
	ofnode node;
	unsigned char goodix_id[GOODIX_ID_LEN] = {0};
	unsigned char ft5426_id[FT5426_ID_LEN] = {0};
	unsigned char cst340_id[CST340_ID_LEN] = {0};
	unsigned int goodix_addr = 0, ft_cst_addr = 0;
	char *goodix_id_list[] = {
		"911", "1151", "9271", "9147", "928", "1158",
    };

	node = ofnode_by_compatible(ofnode_null(), "alientek,rgb-ts");
	if(!ofnode_valid(node)) {
		printf("cannot find alientek,rgb-ts compatible\n");
		return -1;
	}

	ret = ofnode_read_u32(node, "reg", &goodix_addr);
	if (ret) {
		printf("cannot read goodix I2C address in %s\n", ofnode_get_name(node));
		return ret;
	}

	ofnode_read_u32(node, "ft5426,cst340,reg", &ft_cst_addr);

	ret = goodix_reset_irq_gpio(node);
	if(ret) {
		printf("error rgb lcd goodix_reset_irq_gpio\n");
		return -1;
	}

	ret = atk_i2c_read(node, goodix_addr, GOODIX_REG_ID, goodix_id, sizeof(goodix_id), 2);
	if (!ret) {
		for (i = 0; i < ARRAY_SIZE(goodix_id_list); i++) {
			if (!strncmp(goodix_id, goodix_id_list[i], sizeof(goodix_id))) {
				//printf("ts: Goodix (ID %s)\n", goodix_id);
				return 0;
			}
		}
	}

	if (ft_cst_addr != 0) {
		//FT5426
		ret = atk_i2c_read(node, ft_cst_addr, FT5426_REG_ID, ft5426_id, sizeof(ft5426_id), 1);
		if (!ret) {
			if (ft5426_id[0] == 0x54) {
				//printf("ts: FocalTech FT5426 (ID 0x%X)\n", ft5426_id[0]);
				return 0;
			}
		}
		//CST340
		ret = atk_i2c_read(node, ft_cst_addr, CST340_REG_ID, cst340_id, sizeof(cst340_id), 2);
		if (!ret) { 
			if (cst340_id[0] == 0xa5) {
				//printf("ts: Hynitron CST340 (ID 0x%x 0x%x)\n", cst340_id[0], cst340_id[1]);
				return 0;
			}
		}
	}

	return -1;
}

int alientek_set_rgb_lcd(void)
{
    ofnode node;
    int ret, i;
    struct gpio_desc priv_rgb[3];
    unsigned int read_id = 0;
	unsigned int rgb_lcd_id = 0;
	int ts_valid = -1;

    node = ofnode_path("/rgb_lcd_id_pinctrl");
    if(!ofnode_valid(node)) {
        printf("%s: cannot find /rgb_lcd_id_pinctrl node\n", __func__);
        return -1;
    }

    ret = gpio_request_by_name_nodev(node, "gpior", 0,&priv_rgb[0], GPIOD_IS_IN);
    if(ret) {
        printf("%s: cannot get GPIO: ret=%d\n", __func__, ret);
        return -1;
    }

    ret = gpio_request_by_name_nodev(node, "gpiog", 0,&priv_rgb[1], GPIOD_IS_IN);
    if(ret) {
        printf("%s: cannot get GPIO: ret=%d\n", __func__, ret);
        return -1;
    }

    ret = gpio_request_by_name_nodev(node, "gpiob", 0,&priv_rgb[2], GPIOD_IS_IN);
    if(ret) {
        printf("%s: cannot get GPIO: ret=%d\n", __func__, ret);
        return -1;
    }

	ts_valid = detect_alientek_rgb_lcd_touchscreen();
	if(!ts_valid) {
		for(i = 0; i < 3; i++) {
			ret = dm_gpio_get_value(&priv_rgb[i]);
			if(!ret)
				read_id |= (0x1 << i);
		}
		//printf("alientek rgb_lcd_id: %d\n", read_id);
		switch(read_id) {
			case 1:	//atk_rgb_lcd_7_800x480
				rgb_lcd_id = 1;
				env_set("rgb_lcd_id", "1");	
				break;
			case 2:	//atk_rgb_lcd_7_1024x600
				rgb_lcd_id = 2;
				env_set("rgb_lcd_id", "2");	
				break;
			case 4:	//atk_rgb_lcd_4.3_800x480
				rgb_lcd_id = 4;
				env_set("rgb_lcd_id", "4");	
				break;			
			case 5:	//atk_rgb_lcd_10.1_1280x800
				rgb_lcd_id = 5;
				env_set("rgb_lcd_id", "5");	
				break;
			default :
				rgb_lcd_id = 0;
				env_set("rgb_lcd_id", "0");	
				break;
		}
	} else {
		rgb_lcd_id = 0;
		env_set("rgb_lcd_id", "0");	
		printf("Invalid detect_alientek_rgb_lcd_touchscreen\n");
	}

    gpio_free_list_nodev(&priv_rgb[0], 3);
    return rgb_lcd_id;
}

int adc_mipi_dsi_lcd_measurement(ofnode node, unsigned int *adc_value)
{
	struct ofnode_phandle_args adc_args;
	struct udevice *adc;
	unsigned int raw;
	int ret, uV, mV;
	unsigned int times, adc_average_value, adc_sum = 0;

	if(ofnode_parse_phandle_with_args(node, "st,adc_mipi_dsi_lcd_id",
						"#io-channel-cells", 0, 0,
						&adc_args)) {
		printf("cannot find /config/st,adc_mipi_dsi_lcd_id\n");
		return -1;
	}

	ret = uclass_get_device_by_ofnode(UCLASS_ADC, adc_args.node, &adc);
	if(ret) {
		printf("cannot get adc device(%d)\n", ret);
		return -1;
	}

	for(times = 0; times < 5; times++) {
		ret = adc_channel_single_shot(adc->name, adc_args.args[0], &raw);
		if(ret) {
			printf("single shot failed for %s[%d]!\n",
				adc->name, adc_args.args[0]);
			return -1;
		}
		if(!adc_raw_to_uV(adc, raw, &uV)) {
			mV = uV / 1000;
		} else {
			printf("cannot get uV value for %s[%d]\n",
				adc->name, adc_args.args[0]);
			return -1;
		}
		adc_sum +=  mV;
	}

	adc_average_value = adc_sum / times;
	*adc_value = adc_average_value;
	//printf("adc_mipi_dsi_lcd = %d mV\n", *adc_value);
	return 0;
}

int alientek_set_mipi_lcd(void)
{
	int ret;
	int dsi_lcd_id = 1; //atk_no_mipi
	int ts_valid = -1;
	unsigned int adc_value = 0;
	ofnode node;

	node = ofnode_path("/config_adc_mipi_dsi_lcd_id");
	if(!ofnode_valid(node)) {
		log_debug("cannot find /config_adc_mipi_dsi_lcd_id node\n");
		return -ENOENT;
	}

	ts_valid = detect_alientek_mipi_lcd_touchscreen();
	if(!ts_valid) {
		//------------------
		dsi_lcd_id = 2;
		env_set("dsi_lcd_id", "2");
		break;
		//------------------
		ret = adc_mipi_dsi_lcd_measurement(node, &adc_value);
		if(ret) {
			return -1;
		}
		if(adc_value > 500 && adc_value < 700) { //atk_mipi_dsi_5x5_720x1280
			dsi_lcd_id = 2;
			env_set("dsi_lcd_id", "2");
		} else if(adc_value > 800 && adc_value < 1000) { //atk_mipi_dsi_5x5_1080x1920
			dsi_lcd_id = 3;
			env_set("dsi_lcd_id", "3");
		} else if(adc_value > 1150 && adc_value < 1350) { //atk_mipi_dsi_10x1_800x1280
			dsi_lcd_id = 4;
			env_set("dsi_lcd_id", "4");
		} else { //atk_no_mipi
			dsi_lcd_id = 1;
			env_set("dsi_lcd_id", "1");
		}
		//printf("alientek dsi_lcd_id: %d\n", dsi_lcd_id);
	} else {
		dsi_lcd_id = 1;
		env_set("dsi_lcd_id", "1");
		printf("Invalid detect_alientek_mipi_lcd_touchscreen\n");
	}

	return dsi_lcd_id;
}

#endif
/******************** ALIENTEK ADD END ********************/

int board_late_init(void)
{
	const void *fdt_compat;
	int fdt_compat_len;
	char dtb_name[256];
	int buf_len;

	if (board_is_stm32mp257_eval())
		board_stm32mp25x_eval_init();

	if (board_is_stm32mp257_disco() | board_is_stm32mp235_disco())
		board_stm32mp2xx_disco_init();

	if (IS_ENABLED(CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG)) {
		fdt_compat = fdt_getprop(gd->fdt_blob, 0, "compatible",
					 &fdt_compat_len);
		if (fdt_compat && fdt_compat_len) {
			if (strncmp(fdt_compat, "st,", 3) != 0) {
				env_set("board_name", fdt_compat);
			} else {
				env_set("board_name", fdt_compat + 3);

				buf_len = sizeof(dtb_name);
				strlcpy(dtb_name, fdt_compat + 3, buf_len);
				buf_len -= strlen(fdt_compat + 3);
				strlcat(dtb_name, ".dtb", buf_len);
				env_set("fdtfile", dtb_name);
			}
		}
	}

#ifdef ALIENTEK_MIPI_RGB_LCD
	int ret;
	ret = alientek_set_mipi_lcd();
	if(ret < 0 || ret == 1) { //no mipi lcd, detect rgb lcd
		alientek_set_rgb_lcd();
	}
#endif

	return 0;
}

static int fixup_stm32mp257_eval_panel(void *blob)
{
	char const *panel = env_get("panel");
	char const *hdmi = env_get("hdmi");
	bool detect_etml0700z9ndha = false;
	bool detect_adv753x = false;
	int nodeoff = 0, ret;
	enum fdt_status status;

	if (panel)
		detect_etml0700z9ndha = !strcmp(panel, "edt,etml0700z9ndha");

	if (hdmi)
		/* string compare the hdmi compatible limit to 10 chars (adi,adv753) */
		detect_adv753x = !strncmp(hdmi, "adi,adv753x", 10);

	/* update LVDS panel "edt,etml0700z9ndha" */
	status = detect_etml0700z9ndha ? FDT_STATUS_OKAY : FDT_STATUS_DISABLED;
	nodeoff = fdt_set_status_by_compatible(blob, "edt,etml0700z9ndha", status);
	if (nodeoff < 0)
		return nodeoff;
	nodeoff = fdt_set_status_by_compatible(blob, "ilitek,ili251x", status);
	if (nodeoff < 0)
		return nodeoff;
	nodeoff = fdt_set_status_by_pathf(blob, status, "/panel-lvds-backlight");
	if (nodeoff < 0)
		return nodeoff;
	nodeoff = fdt_set_status_by_compatible(blob, "st,stm32mp25-lvds", status);
	if (nodeoff < 0)
		return nodeoff;

	/* update HDMI bridge "adi,adv753x" */
	status = detect_adv753x ? FDT_STATUS_OKAY : FDT_STATUS_DISABLED;
	nodeoff = fdt_set_status_by_compatible(blob, "adi,adv7533", status);
	if (nodeoff < 0)
		nodeoff = fdt_set_status_by_compatible(blob, "adi,adv7535", status);
	/* Do not force disable status for sound card. Keep default status instead */
	if (status == FDT_STATUS_OKAY) {
		if (nodeoff < 0)
			return nodeoff;
		nodeoff = fdt_node_offset_by_compat_reg(blob, "st,stm32mp25-i2s", 0x400b0000);
		if (nodeoff < 0)
			return nodeoff;
		ret = fdt_set_node_status(blob, nodeoff, status);
		if (ret < 0)
			return ret;
		nodeoff = fdt_set_status_by_pathf(blob, status, "/sound");
		if (nodeoff < 0)
			return nodeoff;
		nodeoff = fdt_status_okay_by_compatible(blob, "st,stm32mp25-dsi");
		if (nodeoff < 0)
			return nodeoff;
	}

	if (!detect_adv753x && !detect_etml0700z9ndha) {
		nodeoff = fdt_status_disabled_by_compatible(blob, "st,stm32mp25-ltdc");
		if (nodeoff < 0)
			return nodeoff;
	}

	return 0;
}

static int fixup_stm32mp2xx_disco_panel(void *blob)
{
	char const *panel = env_get("panel");
	bool detect_etml0700z9ndha = false;
	int nodeoff = 0;
	enum fdt_status status;

	if (panel)
		detect_etml0700z9ndha = !strcmp(panel, "edt,etml0700z9ndha");

	/* update LVDS panel "edt,etml0700z9ndha" */
	status = detect_etml0700z9ndha ? FDT_STATUS_OKAY : FDT_STATUS_DISABLED;
	nodeoff = fdt_set_status_by_compatible(blob, "edt,etml0700z9ndha", status);
	if (nodeoff < 0)
		return nodeoff;
	nodeoff = fdt_set_status_by_compatible(blob, "ilitek,ili251x", status);
	if (nodeoff < 0)
		return nodeoff;
	nodeoff = fdt_set_status_by_pathf(blob, status, "/panel-lvds-backlight");
	if (nodeoff < 0)
		return nodeoff;
	nodeoff = fdt_set_status_by_compatible(blob, "st,stm32mp25-lvds", status);
	if (nodeoff < 0)
		return nodeoff;

	return 0;
}

int ft_board_setup(void *blob, struct bd_info *bd)
{
	int ret;

	fdt_copy_fixed_partitions(blob);

	if (CONFIG_IS_ENABLED(FDT_SIMPLEFB))
		fdt_simplefb_enable_and_mem_rsv(blob);

	if (board_is_stm32mp257_eval()) {
		ret = fixup_stm32mp257_eval_panel(blob);
		if (ret)
			log_err("Error during panel fixup ! (%d)\n", ret);
	}

	if (board_is_stm32mp257_disco() | board_is_stm32mp235_disco()) {
		ret = fixup_stm32mp2xx_disco_panel(blob);
		if (ret)
			log_err("Error during panel fixup ! (%d)\n", ret);
	}

	return 0;
}

void board_quiesce_devices(void)
{
	setup_led(LEDST_OFF);
}

#if defined(CONFIG_USB_DWC3) && defined(CONFIG_CMD_STM32PROG_USB)
#include <dfu.h>
/*
 * TEMP: force USB BUS reset forced to false, because it is not supported
 *       in DWC3 USB driver
 * avoid USB bus reset support in DFU stack is required to reenumeration in
 * stm32prog command after flashlayout load or after "dfu-util -e -R"
 */
bool dfu_usb_get_reset(void)
{
	return false;
}
#endif

#if defined(CONFIG_STM32_HYPERBUS)
/* weak function called from common/board_r.c */
int is_flash_available(void)
{
	struct udevice *dev;
	int ret;

	ret = uclass_get_device_by_driver(UCLASS_MTD,
					  DM_DRIVER_GET(stm32_hyperbus),
					  &dev);
	if (ret)
		return 0;

	return 1;
}
#endif

/* weak function called from env/sf.c */
void *env_sf_get_env_addr(void)
{
	return NULL;
}

#if defined(CONFIG_OF_BOARD_FIXUP)

#if defined(CONFIG_STM32MP21X)
#define SPINAND_NOR_PATH "/soc@0/spi@40430000/flash@0"
#else
#define SPINAND_NOR_PATH "/soc@0/ommanager@40500000/spi@40430000/flash@0"
#endif

int fdt_update_fwu_properties(void *blob, int nodeoff,
			      const char *compat_str,
			      const char *storage_path)
{
	int ret;
	int storage_off;

	ret = fdt_increase_size(blob, 100);
	if (ret) {
		printf("fdt_increase_size: err=%s\n", fdt_strerror(ret));
		return ret;
	}

	ret = fdt_setprop_string(blob, nodeoff, "compatible", compat_str);
	if (ret) {
		log_err("Can't set compatible property\n");
		return ret;
	}

	storage_off = fdt_path_offset(blob, storage_path);
	if (storage_off < 0) {
		log_err("Can't find %s path\n", storage_path);
		return nodeoff;
	}

	ret = fdt_setprop_string(blob, nodeoff, "fwu-mdata-store", storage_path);

	if (ret < 0)
		log_err("Can't set fwu-mdata-store property\n");

	return ret;
}

int fdt_update_fwu_mdata(void *blob)
{
	int nodeoff, ret = 0;
	u32 bootmode;

	nodeoff = fdt_path_offset(blob, "/fwu-mdata");
	if (nodeoff < 0) {
		log_info("no /fwu-mdata node ?\n");

		return 0;
	}

	bootmode = get_bootmode() & TAMP_BOOT_DEVICE_MASK;

	switch (bootmode) {
	case BOOT_FLASH_SD:
		/* sdmmc1 : nothing to do, already the default device tree configuration */
		break;
	case BOOT_FLASH_EMMC:
		/* sdmmc2 */
		ret = fdt_update_fwu_properties(blob, nodeoff, "u-boot,fwu-mdata-gpt",
						"/soc@0/bus@42080000/mmc@48230000");
		break;

	case BOOT_FLASH_SPINAND:
	case BOOT_FLASH_NOR:
		/* flash0 */
		ret = fdt_update_fwu_properties(blob, nodeoff, "u-boot,fwu-mdata-mtd",
						SPINAND_NOR_PATH);
		break;
	default:
		/* TF-A firmware update not supported for other boot device */
		ret = fdt_del_node(blob, nodeoff);
	}

	return ret;
}

int board_fix_fdt(void *blob)
{
	int ret = 0;

	if (CONFIG_IS_ENABLED(FWU_MDATA))
		ret = fdt_update_fwu_mdata(blob);

	return ret;
}
#endif /* CONFIG_OF_BOARD_FIXUP */
