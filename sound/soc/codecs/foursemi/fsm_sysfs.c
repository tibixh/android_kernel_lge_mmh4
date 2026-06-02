/**
 * Copyright (c) 2018 WinTech Technologies Co., Ltd. 2018-2019. All rights reserved.
 *Description: Core Defination For Foursemi Device .
 *Author: Fourier Semiconductor Inc.
 * Create: 2019-03-17 File created.
 */

#include "fsm_public.h"
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/slab.h>

#define MAX_LEN 800
static int g_f0_test_status = 0;

static ssize_t fsm_start_f0_test(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t len)
{
    fsm_config_t *cfg = fsm_get_config();
    int ret = -EINVAL;
    int argv;

    sscanf(buf, "%d", &argv);
    pr_info("argv: %d", argv);

    switch (argv) {
        case 0:
            if (g_f0_test_status) {
                ret = fsm_i2c_cmd(FSM_CMD_INIT, 1);
                g_f0_test_status = 0;
            }
            break;
        case 1:
            if (g_f0_test_status == 0) {
                ret = fsm_i2c_cmd(FSM_CMD_F0_STEP1, 0);
                g_f0_test_status = 1;
            }
            break;
        default:
            if (g_f0_test_status) {
                pr_info("freq: %d\n", argv);
                cfg->test_freq = argv;
                ret = fsm_i2c_cmd(FSM_CMD_F0_STEP2, 0);
            }
            break;
    }
    return ret;
}

static ssize_t fsm_zmdata_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    int dev_count = fsm_check_device();
    char str_buffer[MAX_LEN];
    int len;

    len = snprintf(str_buffer, 10, "count:%d ", dev_count);
    len += fsm_get_zmdata_str(&str_buffer[len], (MAX_LEN - len));
    str_buffer[len] = '\0';
    //pr_info("buf: %s\n", str_buffer);
    return scnprintf(buf, PAGE_SIZE, "%s\n", str_buffer);
}

void fsm_hal_clear_calib_data(fsm_dev_t *fsm_dev)
{
    fsm_dev->calib_data.count = 0;
    fsm_dev->calib_data.calib_count = 0;
    fsm_dev->calib_data.calib_re25 = 0;
    fsm_dev->calib_data.minval = 0;
    fsm_dev->calib_data.preval = 0;
}

static ssize_t fsm_show_ic_temp(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return 0;
}

static ssize_t fsm_show_calib(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int ret;
    char str_buffer[MAX_LEN];
    int len;

    pr_info("%s force calibration enter", __func__);

    ret = fsm_calibrate(1);

    len = snprintf(str_buffer, MAX_LEN, "count:%d ", fsm_check_device());
    len += fsm_get_r25_str(&str_buffer[len], (MAX_LEN - len));
    str_buffer[len] = '\0';

    pr_info("buf: %s\n", str_buffer);
    return scnprintf(buf, PAGE_SIZE, "%s\n", str_buffer);
}

static ssize_t fsm_dsp_bypass_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t len)
{
    /*int ret;
    int bypass;

    if (!sscanf(buf, "%d", &bypass)) {
        return -EINVAL;
    }

    ret = fsm_i2c_cmd(FSM_CMD_SET_BYPASS, bypass);

    if(ret == 0) {
        pr_info("%s %s sys DSP Success, ret: %d\n", __func__, ((bypass == 1) ? "Bypass":"Unbypass"), ret);
    } else {
        pr_info("%s %s sys DSP Failed, ret: %d\n", __func__, ((bypass == 1) ? "Bypass":"Unbypass"), ret);
    }

    return ret;*/
    return 0;
}

static ssize_t fsm_reg_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
   /* struct i2c_client *client = to_i2c_client(dev);
    fsm_dev_t *fsm = i2c_get_clientdata(client);
    struct snd_soc_codec *codec;
    int reg=0, nVal=0;
    u16 val=0;

    if (fsm->codec) {
        DEBUGPRINT("%s: enter force right 0x35", __func__);
        DEBUGPRINT("%s: count=%d buf=%s stereo mode=%d", __func__, (int)count, buf, stereo_mode);
        if (count >= 3 && count <= 8) {
            if(sscanf(buf, "%x %x", &reg, &nVal) == 2) {
                DEBUGPRINT("%s reg=0x%x val=0x%x", __func__, reg, nVal);
                val = (u16)nVal;
                if(stereo_mode) {
                    codec = fsm_codecs[fsm_DEV_INDEX_R];
                } else {
                    codec = fsm_codecs[fsm_DEV_INDEX_L];
                }
                // write single reg
                snd_soc_write(codec, reg, val);
            } else {
                PRINT_ERROR("%s parsing failed", __func__ );
            }
        }
    }

    return count;*/
    return 0;
}

static DEVICE_ATTR(f0_test_fsm, 0664,
           fsm_zmdata_show, fsm_start_f0_test);

static DEVICE_ATTR(ic_temp_fsm, 0664,
           fsm_show_ic_temp, NULL);

//S_IRUGO|S_IWUGO,
static DEVICE_ATTR(dsp_bypass_fsm, 0664,
           NULL, fsm_dsp_bypass_store);

static DEVICE_ATTR(force_calib_fsm, 0664,
           fsm_show_calib, NULL);

static DEVICE_ATTR(reg_write_fsm, S_IWUSR,
           NULL, fsm_reg_store);


static struct attribute *fsm_attributes[] = {
    &dev_attr_f0_test_fsm.attr,
    &dev_attr_ic_temp_fsm.attr,
    &dev_attr_dsp_bypass_fsm.attr,
    &dev_attr_force_calib_fsm.attr,
    &dev_attr_reg_write_fsm.attr,
    NULL
};

static const struct attribute_group fsm_attr_group = {
    .attrs = fsm_attributes,
};

int fsm_sysfs_init(struct i2c_client *client)
{
    int ret;

    ret = sysfs_create_group(&client->dev.kobj, &fsm_attr_group);
    pr_info("%s exit, ret = %d\n", __func__, ret);
    return ret;
}

void fsm_sysfs_deinit(struct i2c_client *client)
{
    sysfs_remove_group(&client->dev.kobj, &fsm_attr_group);
    pr_info("%s exit\n", __func__);
}


