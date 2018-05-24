#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include "bme680.h"

#define I2C_RETRY_DELAY 3

struct bme680_device
{
    struct i2c_client *client;
    struct bme680_dev gas_sensor;
};

static struct bme680_device bme680;

static int bme680_i2c_read(struct i2c_client *client, u8 addr, u8 *data, int len)
{
    int result;
    int tries = 0;
    struct i2c_msg msgs[] = {
        {
            .addr = client->addr,
            .flags = client->flags,
            .len = 1,
            .buf = &addr,
        },
        {
            .addr = client->addr,
            .flags = client->flags | I2C_M_RD,
            .len = len,
            .buf = data,
        },
    };

    do
    {
        result = i2c_transfer(client->adapter, msgs, 2);
        pr_debug("%s reg = 0x%x data = 0x%x\n", __func__, addr, *data);
        if (result != 2)
            msleep_interruptible(I2C_RETRY_DELAY);
    } while ((result != 2) && (++tries < 3));

    return result == 2? 0:result ;
}

static int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
    rslt = bme680_i2c_read(bme680.client, reg_addr, reg_data, len);
    return rslt;
}

static int bme680_i2c_write(struct i2c_client *client, u8 reg, u8 *data)
{
    int result;
    int tries = 0;
    u8 buf[] = {reg, *data};

    struct i2c_msg msgs[] = {
        {
            .addr = client->addr,
            .flags = client->flags & I2C_M_TEN,
            .len = 2,
            .buf = buf,
        },
    };

    do
    {
        result = i2c_transfer(client->adapter, msgs, 1);
        pr_debug("%s reg = 0x%x data = 0x%x\n", __func__, reg, *data);
        if (result != 1)
            msleep_interruptible(I2C_RETRY_DELAY);
    } while ((result != 1) && (++tries < 3));

    return result == 1? 0:result ;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int i;
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
    rslt = bme680_i2c_write(bme680.client, reg_addr, reg_data);
    if (len > 1) {
        for(i = 0; i< len / 2; i++){
            rslt = bme680_i2c_write(bme680.client, reg_data[2*i+1], &reg_data[2*i+2]);
        }
    }
    return rslt;
}

void user_delay_ms(uint32_t period)
{
    /*
     * Return control or wait,
     * for a period amount of milliseconds
     */
    msleep(period);
}

int bme680_prepare(void)
{
    uint8_t set_required_settings;
    uint16_t meas_period;
    int8_t rslt = BME680_OK;
    pr_debug("enter %s()\n", __func__);

    /* Set the temperature, pressure and humidity settings */
    bme680.gas_sensor.tph_sett.os_hum = BME680_OS_2X;
    bme680.gas_sensor.tph_sett.os_pres = BME680_OS_4X;
    bme680.gas_sensor.tph_sett.os_temp = BME680_OS_8X;
    bme680.gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;

    /* Set the remaining gas sensor settings and link the heating profile */
    bme680.gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
    /* Create a ramp heat waveform in 3 steps */
    bme680.gas_sensor.gas_sett.heatr_temp = 320; /* degree Celsius */
    bme680.gas_sensor.gas_sett.heatr_dur = 150;  /* milliseconds */

    /* Select the power mode */
    /* Must be set before writing the sensor configuration */
    bme680.gas_sensor.power_mode = BME680_FORCED_MODE;

    /* Set the required sensor settings needed */
    set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL | BME680_GAS_SENSOR_SEL;

    /* Set the desired sensor configuration */
    rslt = bme680_set_sensor_settings(set_required_settings, &bme680.gas_sensor);

    /* Set the power mode */
    rslt = bme680_set_sensor_mode(&bme680.gas_sensor);

    /* Get the total measurement duration so as to sleep or wait till the
	 * measurement is complete */

    bme680_get_profile_dur(&meas_period, &bme680.gas_sensor);
    user_delay_ms(meas_period); /* Delay till the measurement is ready */

    return 0;
}

static ssize_t bme680_temp_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
    struct bme680_field_data data;
    int8_t rslt = BME680_OK;
    bme680_prepare();
    rslt = bme680_get_sensor_data(&data, &bme680.gas_sensor);
    if (rslt == 0)
        return sprintf(buf, "%d\n", data.temperature);
    else
        pr_err("%s() err %d\n", __func__, data.temperature);
    return 0;
}

static ssize_t bme680_pressure_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
    struct bme680_field_data data;
    int8_t rslt = BME680_OK;
    bme680_prepare();
    rslt = bme680_get_sensor_data(&data, &bme680.gas_sensor);
    if (rslt == 0)
        return sprintf(buf, "%d\n", data.pressure);
    else
        pr_err("%s() err %d\n", __func__, data.pressure);
    return 0;
}

static ssize_t bme680_humidity_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
    struct bme680_field_data data;
    int8_t rslt = BME680_OK;
    bme680_prepare();
    rslt = bme680_get_sensor_data(&data, &bme680.gas_sensor);
    if (rslt == 0)
        return sprintf(buf, "%d\n", data.humidity);
    else
        pr_err("%s() err %d\n", __func__, data.humidity);
    return 0;
}

static ssize_t bme680_gas_resistance_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
    struct bme680_field_data data;
    int8_t rslt = BME680_OK;
    bme680_prepare();
    rslt = bme680_get_sensor_data(&data, &bme680.gas_sensor);
    if (rslt == 0)
        return sprintf(buf, "%d\n", data.gas_resistance);
    else
        pr_err("%s() err %d\n", __func__, data.gas_resistance);
    return 0;
}

static DEVICE_ATTR(temp, 0440, bme680_temp_show, NULL);
static DEVICE_ATTR(pressure, 0440, bme680_pressure_show, NULL);
static DEVICE_ATTR(humidity, 0440, bme680_humidity_show, NULL);
static DEVICE_ATTR(gas_resistance, 0440, bme680_gas_resistance_show, NULL);

struct attribute *bme680_attributes[] = {
    &dev_attr_temp.attr,
    &dev_attr_pressure.attr,
    &dev_attr_humidity.attr,
    &dev_attr_gas_resistance.attr,
    NULL};

struct attribute_group bme680_attribute_group = {
    .attrs = bme680_attributes};

static int bme680_i2c_probe(struct i2c_client *client,
                            const struct i2c_device_id *id)
{
    //	struct device_node *node = (&client->dev)->of_node;
    int ret = 0;
    //int value = 0;
    // struct bme680_device *bme680;
    int8_t rslt = BME680_OK;

    pr_info("enter %s\n", __func__);
    if (!i2c_check_functionality(client->adapter,
                                 I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA))
    {
        dev_err(&client->dev, "client is not i2c capable\n");
        return -ENXIO;
    }

    bme680.client = client;
    i2c_set_clientdata(client, &bme680);

    bme680.gas_sensor.dev_id = BME680_I2C_ADDR_PRIMARY;
    bme680.gas_sensor.intf = BME680_I2C_INTF;
    bme680.gas_sensor.delay_ms = user_delay_ms;
    bme680.gas_sensor.read = user_i2c_read;
    bme680.gas_sensor.write = user_i2c_write;

    /* amb_temp can be set to 25 prior to configuring the gas sensor 
     * or by performing a few temperature readings without operating the gas sensor.
     */
    bme680.gas_sensor.amb_temp = 25;

    rslt = bme680_init(&bme680.gas_sensor);
    pr_info("%s() init result = %d\n", __func__, rslt);

    ret = sysfs_create_group(&client->dev.kobj, &bme680_attribute_group);
    printk("%s probe successfully!\n", __func__);
    return 0;
}

static int bme680_i2c_remove(struct i2c_client *client)
{
    // struct bme680_device *bme680 = i2c_get_clientdata(client);

    sysfs_remove_group(&client->dev.kobj, &bme680_attribute_group);
    // kfree(bme680);
    return 0;
}

static const struct of_device_id bme680_of_match[] = {
    {
        .compatible = "bosch,bme680",
    },
    {},
};
MODULE_DEVICE_TABLE(of, bme680_of_match);

static const struct i2c_device_id bme680_i2c_id[] = {
    {"bme680", 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, bme680_i2c_id);

static struct i2c_driver bme680_i2c_driver = {
    .probe = bme680_i2c_probe,
    .remove = bme680_i2c_remove,
    .id_table = bme680_i2c_id,
    .driver = {
        .name = "bme680",
        .of_match_table = bme680_of_match,
    },
};
module_i2c_driver(bme680_i2c_driver);

MODULE_LICENSE("GPL v2");
