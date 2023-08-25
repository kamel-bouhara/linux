// SPDX-License-Identifier: GPL-2.0-only
/*
 * TouchNetix aXiom Touchscreen Driver
 *
 * Copyright (C) 2020-2023 TouchNetix Ltd.
 *
 * Author(s): Bart Prescott <bartp@baasheep.co.uk>
 *            Pedro Torruella <pedro.torruella@touchnetix.com>
 *            Mark Satterthwaite <mark.satterthwaite@touchnetix.com>
 *            Hannah Rossiter <hannah.rossiter@touchnetix.com>
 *
 */

#include <linux/crc16.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/string.h>

/**
 * u31 has 2 pages for usage table entries.
 * (2 * AXIOM_COMMS_PAGE_SIZE) / AXIOM_U31_BYTES_PER_USAGE = 85
 */
#define AXIOM_COMMS_MAX_USAGE_PAGES	3
#define AXIOM_PROX_LEVEL		-128
#define AXIOM_COMMS_PAGE_SIZE		256
#define AXIOM_REBASELINE_CMD		0x03
#define AXIOM_U46_AUX_CHANNELS		4U
#define AXIOM_U31_BYTES_PER_USAGE	6U
#define AXIOM_U41_MAX_TARGETS		10U
#define AXIOM_U31_MAX_USAGES		85U
#define AXIOM_DEVINFO_USAGE_ID		0x31
#define AXIOM_REPORT_USAGE_ID		0x34
#define AXIOM_USAGE_2DCTS_REPORT_ID	0x41U
#define AXIOM_USAGE_2AUX_REPORT_ID	0x46U
#define AXIOM_USAGE_2HB_REPORT_ID	0x01U
#define AXIOM_U31_PAGE0_LENGTH		0x0C
#define AXIOM_COMMS_OVERFLOW_MSK	0x80
#define AXIOM_COMMS_REPORT_LEN_MSK	0x7F
#define AXIOM_U46_AUX_MASK		0xFFFU

struct axiom_devinfo {
	char bootloader_fw_ver_major;
	char bootloader_fw_ver_minor;
	char bootloader_mode;
	u16 device_id;
	char fw_major;
	char fw_minor;
	u16 fw_info_extra;
	u16 jedec_id;
	char num_usages;
	char silicon_revision;
};

/**
 * Describes parameters of a specific usage, essenstially a single element of
 * the "Usage Table"
 */
struct usage_entry {
	char id;
	char is_report;
	char start_page;
	char num_pages;
};

/**
 * Holds state of a "Target", A.K.A. as a "touch", but called a target as it
 * can be a detected "target" prior to touch, eg, hovering.
 */
enum axiom_target_state {
	TARGET_STATE_NOT_PRESENT = 0,
	TARGET_STATE_PROX = 1,
	TARGET_STATE_HOVER = 2,
	TARGET_STATE_TOUCHING = 3,
	TARGET_STATE_MIN = TARGET_STATE_NOT_PRESENT,
	TARGET_STATE_MAX = TARGET_STATE_TOUCHING,
};

struct u41_target {
	enum axiom_target_state state;
	u16 x;
	u16 y;
	s8 z;
	bool insert;
	bool touch;
};

struct axiom_target_report {
	u8 index;
	u8 present;
	u16 x;
	u16 y;
	s8 z;
};

struct axiom_cmd_header {
	u16 target_address;
	u16 length:15;
	u16 read:1;
	char write_data[];
};

struct axiom_data {
	struct axiom_devinfo devinfo;
	struct device *pdev;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *irq_gpio;
	struct i2c_client *client;
	struct input_dev *input_dev;
	char max_report_len;
	u32 report_overflow_counter;
	u32 report_counter;
	char rx_buf[AXIOM_COMMS_MAX_USAGE_PAGES * AXIOM_COMMS_PAGE_SIZE];
	struct u41_target targets[AXIOM_U41_MAX_TARGETS];
	struct usage_entry usage_table[AXIOM_U31_MAX_USAGES];
	bool usage_table_populated;
};

/**
 * aXiom devices are typically configured to report
 * touches at a rate of 100Hz (10ms). For systems
 * that require polling for reports, 100ms seems like
 * an acceptable polling rate.
 * When reports are polled, it will be expected to
 * occasionally observe the overflow bit being set
 * in the reports. This indicates that reports are not
 * being read fast enough.
 */
#define POLL_INTERVAL_DEFAULT_MS 100

/**
 * Translate usage/page/offset triplet into physical address.
 *
 * @usage: groups of registers
 * @page: page to which the usage belongs to offset
 * @offset of the usage
 */
static u16
usage_to_target_address(struct axiom_data *ts, char usage, char page,
			char offset)
{
	struct axiom_devinfo *device_info;
	struct usage_entry *usage_table;
	u16 target_address;
	u32 i;

	device_info = &ts->devinfo;
	usage_table = ts->usage_table;

	/* At the moment the convention is that u31 is always at physical address 0x0 */
	if (!ts->usage_table_populated && (usage == AXIOM_DEVINFO_USAGE_ID)) {
		target_address = ((page << 8) + offset);
	} else if (ts->usage_table_populated) {
		for (i = 0; i < device_info->num_usages; i++) {
			if (usage_table[i].id == usage) {
				if (page < usage_table[i].num_pages) {
					target_address =
					    ((usage_table[i].start_page + page) << 8) + offset;
				} else {
					target_address = 0xFFFF;
					dev_err(ts->pdev,
						"Invalid usage table! usage: %u, page: %u, offset: %u\n",
						usage, page, offset);
				}
				break;
			}
		}
	} else {
		target_address = 0xFFFF;
		dev_err(ts->pdev, "Unpopulated usage table for usage: %u\n",
			usage);
	}

	dev_dbg(ts->pdev, "target_address is 0x%04x for usage: %u page %u\n",
		target_address, usage, page);

	return target_address;
}

static int
axiom_i2c_read(struct i2c_client *client, u8 usage, u8 page, u8 *buf, u16 len)
{
	struct axiom_cmd_header cmd_header;
	struct axiom_data *ts = i2c_get_clientdata(client);
	struct i2c_msg msg[2];
	int ret;

	/* Build the header */
	cmd_header.target_address = usage_to_target_address(ts, usage, page, 0);
	cmd_header.length = len;
	cmd_header.read = 1;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = sizeof(cmd_header);
	msg[0].buf = (u8 *)&cmd_header;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = (char *)buf;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret != ARRAY_SIZE(msg)) {
		dev_err(&client->dev,
			"Failed reading usage %#x page %#x, error=%d\n", usage,
			page, ret);
		return -EIO;
	}

	return 0;
}

static int
axiom_i2c_write(struct i2c_client *client, u8 usage, u8 page, u8 *buf, u16 len)
{
	struct axiom_cmd_header cmd_header;
	struct axiom_data *ts = i2c_get_clientdata(client);
	struct i2c_msg msg[2];
	int ret;

	cmd_header.target_address = usage_to_target_address(ts, usage, page, 0);
	cmd_header.length = len;
	cmd_header.read = 0;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = sizeof(cmd_header);
	msg[0].buf = (u8 *)&cmd_header;

	msg[1].addr = client->addr;
	msg[1].flags = 0;
	msg[1].len = len;
	msg[1].buf = (char *)buf;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret != 2) {
		dev_err(&client->dev,
			"Failed to write usage %#x page %#x, error=%d\n", usage,
			page, ret);
		return ret;
	}

	return 0;
}

/**
 * Decodes and populates the local Usage Table.
 * Given a buffer of data read from page 1 onwards of u31 from an aXiom
 * device.
 */
static char axiom_populate_usage_table(struct axiom_data *ts, char *rx_data)
{
	u32 usage_id = 0;
	char max_report_len = 0;
	struct axiom_devinfo *device_info;
	struct usage_entry *usage_table;

	device_info = &ts->devinfo;
	usage_table = ts->usage_table;

	for (usage_id = 0; usage_id < device_info->num_usages; usage_id++) {
		u16 offset = (usage_id * AXIOM_U31_BYTES_PER_USAGE);
		char id = rx_data[offset + 0];
		char start_page = rx_data[offset + 1];
		char num_pages = rx_data[offset + 2];
		char max_offset = ((rx_data[offset + 3] & 0x7F) + 1) * 2;

		/* Store the entry into the usage table */
		usage_table[usage_id].id = id;
		usage_table[usage_id].is_report = ((num_pages == 0) ? 1 : 0);
		usage_table[usage_id].start_page = start_page;
		usage_table[usage_id].num_pages = num_pages;

		dev_dbg(ts->pdev, "Usage %2u Info: %*ph\n", usage_id,
			AXIOM_U31_BYTES_PER_USAGE, &rx_data[offset]);

		/* Identify the max report length the module will receive */
		if ((usage_table[usage_id].is_report)
		    && (max_offset > max_report_len))
			max_report_len = max_offset;
	}
	ts->usage_table_populated = true;

	return max_report_len;
}

/**
 * Retrieve, store and print the axiom device information
 */
static int axiom_discover(struct axiom_data *ts)
{
	struct axiom_devinfo *devinfo = &ts->devinfo;
	struct device *pdev = ts->pdev;
	char *rx_data = &ts->rx_buf[0];
	int ret;

	/* First the first page of u31 to get the device information and */
	/* the number of usages */
	ret =
	    axiom_i2c_read(ts->client, AXIOM_DEVINFO_USAGE_ID, 0, rx_data,
			   AXIOM_U31_PAGE0_LENGTH);
	if (ret)
		return ret;

	devinfo->bootloader_mode = ((rx_data[1] & 0x80) != 0) ? 1 : 0;
	devinfo->device_id = ((rx_data[1] & 0x7F) << 8) | rx_data[0];
	devinfo->fw_minor = rx_data[2];
	devinfo->fw_major = rx_data[3];
	devinfo->fw_info_extra = (rx_data[4]) | (rx_data[5] << 8);
	devinfo->bootloader_fw_ver_minor = rx_data[6];
	devinfo->bootloader_fw_ver_major = rx_data[7];
	devinfo->jedec_id = (rx_data[8]) | (rx_data[9] << 8);
	devinfo->num_usages = rx_data[10];
	devinfo->silicon_revision = rx_data[11];

	dev_dbg(pdev, "Data Decode:\n");
	dev_dbg(pdev, "  Bootloader Mode: %u\n", ts->devinfo.bootloader_mode);
	dev_dbg(pdev, "  Device ID      : %04x\n", ts->devinfo.device_id);
	dev_dbg(pdev, "  Firmware Rev   : %02x.%02x\n", ts->devinfo.fw_major,
		ts->devinfo.fw_minor);
	dev_dbg(pdev, "  Bootloader Rev : %02x.%02x\n",
		ts->devinfo.bootloader_fw_ver_major,
		ts->devinfo.bootloader_fw_ver_minor);
	dev_dbg(pdev, "  FW Extra Info  : %04x\n", ts->devinfo.fw_info_extra);
	dev_dbg(pdev, "  Silicon        : %02x\n", ts->devinfo.jedec_id);
	dev_dbg(pdev, "  Num Usages     : %04x\n", ts->devinfo.num_usages);

	/* Read the second page of u31 to get the usage table */
	ret = axiom_i2c_read(ts->client, AXIOM_DEVINFO_USAGE_ID, 1, rx_data,
			     (AXIOM_U31_BYTES_PER_USAGE * ts->devinfo.num_usages));
	if (ret)
		return ret;

	ts->max_report_len = axiom_populate_usage_table(ts, rx_data);
	dev_dbg(pdev, "Max Report Length: %u\n", ts->max_report_len);

	return 0;
}

/*
 * Support function to axiom_process_u41_report.
 * It generates input-subsystem events for every target.
 * After calling this function the caller shall issue
 * a Sync to the input sub-system.
 */
static bool
axiom_process_u41_report_target(struct axiom_data *ts,
				struct axiom_target_report *target)
{
	struct input_dev *input_dev = ts->input_dev;
	enum axiom_target_state current_state;
	struct u41_target *target_prev_state;
	struct device *pdev = ts->pdev;
	bool update = false;
	int slot;

	/* Verify the target index */
	if (target->index >= AXIOM_U41_MAX_TARGETS) {
		dev_dbg(pdev, "Invalid target index! %u\n", target->index);
		return false;
	}

	target_prev_state = &ts->targets[target->index];

	current_state =
	    ((target->present == 0) ? TARGET_STATE_NOT_PRESENT : (target->z >=
								  0) ?
	     TARGET_STATE_TOUCHING : (target->z > AXIOM_PROX_LEVEL)
	     && (target->z < 0) ? TARGET_STATE_HOVER : (target->z ==
							AXIOM_PROX_LEVEL) ?
	     TARGET_STATE_PROX : TARGET_STATE_NOT_PRESENT);
	if ((target_prev_state->state == current_state)
	    && (target_prev_state->x == target->x)
	    && (target_prev_state->y == target->y)
	    && (target_prev_state->z == target->z)) {
		return false;
	}

	slot = target->index;

	dev_dbg(pdev, "U41 Target T%u, slot:%u present:%u, x:%u, y:%u, z:%d\n",
		target->index, slot, target->present,
		target->x, target->y, target->z);

	switch (current_state) {
	default:
	case TARGET_STATE_NOT_PRESENT:
	case TARGET_STATE_PROX:
		{
			if (target_prev_state->insert) {
				update = true;
				target_prev_state->insert = false;
				input_mt_slot(input_dev, slot);

				if (slot == 0)
					input_report_key(input_dev, BTN_LEFT,
							 0);

				input_mt_report_slot_inactive(input_dev);
				/* make sure the previous coordinates are all off */
				/* screen when the finger comes back */
				target->x = target->y = 65535;
				target->z = AXIOM_PROX_LEVEL;
			}
			break;
		}
	case TARGET_STATE_HOVER:
	case TARGET_STATE_TOUCHING:
		{
			target_prev_state->insert = true;
			update = true;
			input_mt_slot(input_dev, slot);
			input_report_abs(input_dev, ABS_MT_TRACKING_ID, slot);
			input_report_abs(input_dev, ABS_MT_POSITION_X,
					 target->x);
			input_report_abs(input_dev, ABS_X, target->x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y,
					 target->y);
			input_report_abs(input_dev, ABS_Y, target->y);

			if (current_state == TARGET_STATE_TOUCHING) {
				input_report_abs(input_dev, ABS_MT_DISTANCE, 0);
				input_report_abs(input_dev, ABS_DISTANCE, 0);
				input_report_abs(input_dev, ABS_MT_PRESSURE,
						 target->z);
				input_report_abs(input_dev, ABS_PRESSURE,
						 target->z);
			} else {
				input_report_abs(input_dev, ABS_MT_DISTANCE,
						 -target->z);
				input_report_abs(input_dev, ABS_DISTANCE,
						 -target->z);
				input_report_abs(input_dev, ABS_MT_PRESSURE, 0);
				input_report_abs(input_dev, ABS_PRESSURE, 0);
			}

			if (slot == 0)
				input_report_key(input_dev, BTN_LEFT,
						 (current_state ==
						  TARGET_STATE_TOUCHING));

			break;
		}
	}

	target_prev_state->state = current_state;
	target_prev_state->x = target->x;
	target_prev_state->y = target->y;
	target_prev_state->z = target->z;

	if (update)
		input_mt_sync_frame(input_dev);

	return update;
}

/**
 * Take a raw buffer with u41 report data and decode it.
 * Also generate input events if needed.
 * @rx_buf: ptr to a byte array [0]: Usage number [1]: Status LSB [2]: Status MSB
 */
static void axiom_process_u41_report(struct axiom_data *ts, char *rx_buf)
{
	struct axiom_target_report target;
	struct input_dev *input_dev = ts->input_dev;
	bool update_done = false;
	u16 target_status;
	u32 i;

	if (rx_buf[0] != 0x41) {
		dev_err(ts->pdev,
			"Data in buffer does not have expected u41 format.\n");
		return;
	}

	target_status = ((rx_buf[1]) | (rx_buf[2] << 8));

	for (i = 0; i < AXIOM_U41_MAX_TARGETS; i++) {
		target.index = i;
		target.present = ((target_status & (1 << i)) != 0) ? 1 : 0;
		target.x = (rx_buf[(i * 4) + 3]) | (rx_buf[(i * 4) + 4] << 8);
		target.y = (rx_buf[(i * 4) + 5]) | (rx_buf[(i * 4) + 6] << 8);
		target.z = (s8) (rx_buf[i + 43]);
		update_done |= axiom_process_u41_report_target(ts, &target);
	}

	if (update_done)
		input_sync(input_dev);
}

static void axiom_process_u46_report(struct axiom_data *ts, char *rx_buf)
{
	struct input_dev *input_dev = ts->input_dev;
	u16 aux_value;
	u32 event_value;
	u32 i = 0;

	for (i = 0; i < AXIOM_U46_AUX_CHANNELS; i++) {
		aux_value =
		    ((rx_buf[(i * 2) + 2] << 8) | (rx_buf[(i * 2) + 1])) &
		    AXIOM_U46_AUX_MASK;
		event_value = (i << 16) | (aux_value);
		input_event(input_dev, EV_MSC, MSC_RAW, event_value);
	}

	input_mt_sync(input_dev);
	input_sync(input_dev);
}

/**
 * Support function to axiom_process_report.
 * It validates the crc and multiplexes the axiom reports to the appropriate
 * report handler
 */
void axiom_process_report(struct axiom_data *ts, char *report_data)
{
	struct device *pdev = ts->pdev;
	char usage = report_data[1];
	u16 crc_report;
	u16 crc_calc;
	char len;

	if ((report_data[0] & AXIOM_COMMS_OVERFLOW_MSK) != 0)
		ts->report_overflow_counter++;

	len = (report_data[0] & AXIOM_COMMS_REPORT_LEN_MSK) << 1;
	if (len == 0) {
		dev_err(pdev, "Zero length report discarded.\n");
		return;
	}

	dev_dbg(pdev, "Payload Data %*ph\n", len, report_data);

	/* Validate the report CRC */
	crc_report = (report_data[len - 1] << 8) | (report_data[len - 2]);
	/* Length is in 16 bit words and remove the size of the CRC16 itself */
	crc_calc = crc16(0, report_data, (len - 2));

	if (crc_calc != crc_report) {
		dev_err(pdev,
			"CRC mismatch! Expected: %#x, Calculated CRC: %#x.\n",
			crc_report, crc_calc);
		return;
	}

	switch (usage) {
	case AXIOM_USAGE_2DCTS_REPORT_ID:
		axiom_process_u41_report(ts, &report_data[1]);
		break;

	case AXIOM_USAGE_2AUX_REPORT_ID:
		/* This is an aux report (force) */
		axiom_process_u46_report(ts, &report_data[1]);
		break;

	case AXIOM_USAGE_2HB_REPORT_ID:
		/* This is a heartbeat report */
		break;

	default:
		break;
	}

	ts->report_counter++;
}

static void axiom_i2c_poll(struct input_dev *input_dev)
{
	struct axiom_data *ts = input_get_drvdata(input_dev);
	char *rx_data = ts->rx_buf;

	axiom_i2c_read(ts->client, AXIOM_REPORT_USAGE_ID, 0, rx_data,
		       ts->max_report_len);
	axiom_process_report(ts, rx_data);
}

static void axiom_reset(struct gpio_desc *reset_gpio)
{
        gpiod_set_value_cansleep(reset_gpio, 1);
        usleep_range(1000, 2000);
        gpiod_set_value_cansleep(reset_gpio, 0);
        msleep(100);
}

/**
 * Rebaseline the touchscreen, effectively zero-ing it
 */
void axiom_rebaseline(struct axiom_data *ts)
{
	struct device *pdev = ts->pdev;
	char buffer[8] = { 0 };
	int ret;

	memset(buffer, 0, sizeof(buffer));

	buffer[0] = AXIOM_REBASELINE_CMD;

	ret = axiom_i2c_write(ts->client, 0x02, 0, buffer, sizeof(buffer));
	if (ret)
		dev_err(pdev, "Rebaseline failed\n");

	dev_info(pdev, "Capture Baseline Requested\n");
}
EXPORT_SYMBOL_GPL(axiom_rebaseline);

static irqreturn_t axiom_irq(int irq, void *handle)
{
	struct axiom_data *ts = handle;
	u8 *rx_data = &ts->rx_buf[0];

	axiom_i2c_read(ts->client, AXIOM_REPORT_USAGE_ID, 0, rx_data, ts->max_report_len);
	axiom_process_report(ts, rx_data);

	return IRQ_HANDLED;
}

static int axiom_i2c_probe(struct i2c_client *client)
{
	struct axiom_data *ts;
	struct device *pdev = &client->dev;
	struct input_dev *input_dev;
	int error;
	int target;

	ts = devm_kzalloc(pdev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->irq_gpio = devm_gpiod_get_optional(pdev, "irq", GPIOD_IN);
	if (IS_ERR(ts->irq_gpio))
		return dev_err_probe(pdev, PTR_ERR(ts->irq_gpio), "failed to get irq GPIO");

	ts->reset_gpio = devm_gpiod_get_optional(pdev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(pdev, PTR_ERR(ts->reset_gpio), "failed to get reset GPIO\n");

	axiom_reset(ts->reset_gpio);

	if (ts->irq_gpio) {
		error =
		    devm_request_threaded_irq(pdev, client->irq, NULL,
					      axiom_irq,
					      IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					      "axiom_irq", ts);
		if (error < 0)
			return dev_err_probe(pdev, error, "Failed to request threaded IRQ\n");
	}

	ts->client = client;
	ts->pdev = pdev;
	ts->usage_table_populated = false;

	i2c_set_clientdata(client, ts);

	axiom_discover(ts);
	axiom_rebaseline(ts);

	input_dev = devm_input_allocate_device(ts->pdev);
	if (!input_dev)
		return -ENOMEM;

	input_dev->name = "TouchNetix aXiom Touchscreen";
	input_dev->phys = "input/axiom_ts";

	/* Single Touch */
	input_set_abs_params(input_dev, ABS_X, 0, 65535, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, 65535, 0, 0);

	/* Multi Touch */
	/* Min, Max, Fuzz (expected noise in px, try 4?) and Flat */
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, 65535, 0, 0);
	/* Min, Max, Fuzz (expected noise in px, try 4?) and Flat */
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, 65535, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_DISTANCE, 0, 127, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 127, 0, 0);

	/* Registers the axiom device as a touch screen instead of as a mouse pointer */
	input_mt_init_slots(input_dev, AXIOM_U41_MAX_TARGETS, INPUT_MT_DIRECT);

	input_set_capability(input_dev, EV_KEY, BTN_LEFT);

	/* Enables the raw data for up to 4 force channels to be sent to the */
	/* input subsystem */
	set_bit(EV_REL, input_dev->evbit);
	set_bit(EV_MSC, input_dev->evbit);
	/* Declare that we support "RAW" Miscellaneous events */
	set_bit(MSC_RAW, input_dev->mscbit);

	if (!ts->irq_gpio) {
		error = input_setup_polling(input_dev, axiom_i2c_poll);
		if (error) {
			dev_err(ts->pdev, "Unable to set up polling: %d\n", 
				error);
			return error;
		}
		input_set_poll_interval(input_dev, POLL_INTERVAL_DEFAULT_MS);
	}

	ts->input_dev = input_dev;
	input_set_drvdata(ts->input_dev, ts);

	/* Ensure that all reports are initialised to not be present. */
	for (target = 0; target < AXIOM_U41_MAX_TARGETS; target++)
		ts->targets[target].state = TARGET_STATE_NOT_PRESENT;

	error = input_register_device(input_dev);

	if (error != 0)
		return dev_err_probe(ts->pdev, error, 
					"Could not register with Input Sub-system.\n");

	return 0;
}

static void axiom_i2c_remove(struct i2c_client *client)
{
	struct axiom_data *ts = i2c_get_clientdata(client);

	if (ts->usage_table)
		ts->usage_table_populated = false;

	if (ts->input_dev)
		input_unregister_device(ts->input_dev);
}

static const struct i2c_device_id axiom_i2c_id_table[] = {
	{"axiom"},
	{},
};

MODULE_DEVICE_TABLE(i2c, axiom_i2c_id_table);

static const struct of_device_id axiom_i2c_dt_ids[] = {
	{
	 .compatible = "touchnetix,axiom",
	 },
	{}
};

MODULE_DEVICE_TABLE(of, axiom_i2c_dt_ids);

static struct i2c_driver axiom_i2c_driver = {
	.driver = {
		   .name = "axiom_i2c",
		   .of_match_table = axiom_i2c_dt_ids,
		   },
	.id_table = axiom_i2c_id_table,
	.probe = axiom_i2c_probe,
	.remove = axiom_i2c_remove,
};

module_i2c_driver(axiom_i2c_driver);

MODULE_AUTHOR("Kamel Bouhara <kamel.bouhara@bootlin.com>");
MODULE_DESCRIPTION("aXiom touchscreen I2C bus driver");
MODULE_LICENSE("GPL");
