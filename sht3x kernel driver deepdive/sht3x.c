#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/sysfs.h>
#include <linux/poll.h>
#include <linux/pm.h>

#include "sht3x_ioctl.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("NguyenNhan");
MODULE_DESCRIPTION("Enhanced SHT3x I2C driver for temperature and humidity sensing");
MODULE_VERSION("0.3");

#define PRINTK(x, y, ...) printk(x "SHT3X: %s " y, __FUNCTION__, ## __VA_ARGS__)

#define I2C_BUS_AVAILABLE   (1)              // Default I2C bus number
#define SLAVE_DEVICE_NAME   ("SHT3X")        // Device and driver name
#define SLAVE_ADDR          (0x44)           // SHT3x default I2C address
#define SLAVE_ADDR_ALT      (0x45)           // SHT3x alternate I2C address

#define COMMAND_LEN         (2)               // Length of I2C command
#define STATUS_LEN          (3)               // Length of status register data
#define MEASUREMENT_LEN     (6)               // Length of measurement data (2B temp + 1B CRC + 2B hum + 1B CRC)
#define MAX_READ_RETRIES    (3)               // Maximum retries for failed reads

// SHT3x command set
static const unsigned char cmd_break[COMMAND_LEN] = {0x30, 0x93};
static const unsigned char cmd_status_read[COMMAND_LEN] = {0xf3, 0x2d};
static const unsigned char cmd_status_clear[COMMAND_LEN] = {0x30, 0x41};
static const unsigned char cmd_heater_enable[COMMAND_LEN] = {0x30, 0x6d};
static const unsigned char cmd_heater_disable[COMMAND_LEN] = {0x30, 0x66};
static const unsigned char cmd_singleshot_low[COMMAND_LEN] = {0x2c, 0x10};
static const unsigned char cmd_singleshot_med[COMMAND_LEN] = {0x2c, 0x0D};
static const unsigned char cmd_singleshot_hi[COMMAND_LEN] = {0x2c, 0x06};
static const unsigned char cmd_periodic_0p5_low[COMMAND_LEN] = {0x20, 0x2f};
static const unsigned char cmd_periodic_0p5_med[COMMAND_LEN] = {0x20, 0x24};
static const unsigned char cmd_periodic_0p5_hi[COMMAND_LEN] = {0x20, 0x32};
static const unsigned char cmd_periodic_1_low[COMMAND_LEN] = {0x21, 0x2d};
static const unsigned char cmd_periodic_1_med[COMMAND_LEN] = {0x21, 0x26};
static const unsigned char cmd_periodic_1_hi[COMMAND_LEN] = {0x21, 0x30};
static const unsigned char cmd_periodic_2_low[COMMAND_LEN] = {0x22, 0x2b};
static const unsigned char cmd_periodic_2_med[COMMAND_LEN] = {0x22, 0x20};
static const unsigned char cmd_periodic_2_hi[COMMAND_LEN] = {0x22, 0x36};
static const unsigned char cmd_periodic_4_low[COMMAND_LEN] = {0x23, 0x29};
static const unsigned char cmd_periodic_4_med[COMMAND_LEN] = {0x23, 0x22};
static const unsigned char cmd_periodic_4_hi[COMMAND_LEN] = {0x23, 0x34};
static const unsigned char cmd_periodic_10_low[COMMAND_LEN] = {0x27, 0x2a};
static const unsigned char cmd_periodic_10_med[COMMAND_LEN] = {0x27, 0x21};
static const unsigned char cmd_periodic_10_hi[COMMAND_LEN] = {0x27, 0x37};

// Array of measurement commands
static const unsigned char *measurement_commands[] = {
    cmd_singleshot_low, cmd_singleshot_med, cmd_singleshot_hi,
    cmd_periodic_0p5_low, cmd_periodic_0p5_med, cmd_periodic_0p5_hi,
    cmd_periodic_1_low, cmd_periodic_1_med, cmd_periodic_1_hi,
    cmd_periodic_2_low, cmd_periodic_2_med, cmd_periodic_2_hi,
    cmd_periodic_4_low, cmd_periodic_4_med, cmd_periodic_4_hi,
    cmd_periodic_10_low, cmd_periodic_10_med, cmd_periodic_10_hi
};

// Driver state variables
static int measurement_mode = SHT3X_SINGLE_SHOT_LOW;
static int slave_addr = SLAVE_ADDR;
static int crc_check_enabled = SHT3X_CRC_CHECK_ENABLE;
static unsigned char measurement[MEASUREMENT_LEN];
static int measurement_count = 0;
static int measurement_status = 0;
DEFINE_MUTEX(measurement_mutex);
DECLARE_WAIT_QUEUE_HEAD(measurement_wait);

// I2C client and adapter
static struct i2c_adapter *sht_i2c_adapter = NULL;
static struct i2c_client *sht_i2c_client = NULL;
static struct i2c_board_info sht_i2c_board_info = {I2C_BOARD_INFO(SLAVE_DEVICE_NAME, SLAVE_ADDR)};

// I2C device ID table
static const struct i2c_device_id i2c_id[] = {
    { SLAVE_DEVICE_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, i2c_id);

// Power management operations
static int i2c_suspend(struct device *dev);
static int i2c_resume(struct device *dev);
static const struct dev_pm_ops sht3x_pm_ops = {
    .suspend = i2c_suspend,
    .resume = i2c_resume,
};

// I2C driver structure
static struct i2c_driver sht_i2c_driver = {
    .driver = {
        .name = SLAVE_DEVICE_NAME,
        .owner = THIS_MODULE,
        .pm = &sht3x_pm_ops,
    },
    .probe = i2c_probe,
    .remove = i2c_remove,
    .id_table = i2c_id,
};

// Character device structures
static dev_t dev = 0;
static struct class *dev_class;
static struct cdev dev_cdev;
static char deventry[] = "sht3x  ";
static atomic_t dev_use_count = ATOMIC_INIT(0);

// File operations
static int fops_open(struct inode *inode, struct file *file);
static int fops_release(struct inode *inode, struct file *file);
static ssize_t fops_read(struct file *filp, char __user *buf, size_t len, loff_t *off);
static ssize_t fops_write(struct file *filp, const char __user *buf, size_t len, loff_t *off);
static long fops_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static unsigned int fops_poll(struct file *filp, struct poll_table_struct *wait);

static struct file_operations fops = {
    .owner          = THIS_MODULE,
    .read           = fops_read,
    .write          = fops_write,
    .open           = fops_open,
    .unlocked_ioctl = fops_ioctl,
    .poll           = fops_poll,
    .release        = fops_release,
};

// Timer and workqueue for periodic measurements
static struct timer_list periodic_timer;
static void periodic_timer_callback(struct timer_list *data);
static void worker_read_measurement(struct work_struct *work);
DECLARE_WORK(worker, worker_read_measurement);
static struct workqueue_struct *workqueue = NULL;

// Sysfs attributes
static struct device_attribute dev_attr_temperature;
static struct device_attribute dev_attr_humidity;
static struct device_attribute dev_attr_status;

// Module parameters
module_param(measurement_mode, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(measurement_mode, "Measurement mode (0-17)");
module_param(slave_addr, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(slave_addr, "I2C slave address (0x44 or 0x45)");

// Function prototypes
static int periodic_time_ms(void);
static int break_and_cleanup(void);
static u8 calculate_crc8(const u8 *data, int len);
static int validate_measurement_crc(const unsigned char *data);
static float convert_temperature(const unsigned char *data);
static float convert_humidity(const unsigned char *data);

/**
 * calculate_crc8 - Compute CRC8 checksum for SHT3x data
 * @data: Input data buffer
 * @len: Length of data to process
 * Returns: Calculated CRC8 value
 */
static u8 calculate_crc8(const u8 *data, int len) {
    u8 crc = 0xFF;
    int i, j;
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/**
 * validate_measurement_crc - Validate CRC of measurement data
 * @data: 6-byte measurement data (temp: 2B + CRC, hum: 2B + CRC)
 * Returns: 0 on success, -1 on CRC failure
 */
static int validate_measurement_crc(const unsigned char *data) {
    u8 crc_temp = calculate_crc8(data, 2);
    u8 crc_hum = calculate_crc8(data + 3, 2);
    return (crc_temp == data[2] && crc_hum == data[5]) ? 0 : -1;
}

/**
 * convert_temperature - Convert raw temperature to Celsius
 * @data: 2-byte temperature data
 * Returns: Temperature in Celsius
 */
static float convert_temperature(const unsigned char *data) {
    u16 raw_temp = (data[0] << 8) | data[1];
    return -45.0 + 175.0 * ((float)raw_temp / 65535.0);
}

/**
 * convert_humidity - Convert raw humidity to percentage
 * @data: 2-byte humidity data
 * Returns: Relative humidity in percentage
 */
static float convert_humidity(const unsigned char *data) {
    u16 raw_hum = (data[3] << 8) | data[4];
    return 100.0 * ((float)raw_hum / 65535.0);
}

/**
 * temperature_show - Sysfs interface to read temperature
 */
static ssize_t temperature_show(struct device *dev, struct device_attribute *attr, char *buf) {
    mutex_lock(&measurement_mutex);
    if (measurement_status != 0) {
        mutex_unlock(&measurement_mutex);
        return -EIO;
    }
    float temp = convert_temperature(measurement);
    mutex_unlock(&measurement_mutex);
    return scnprintf(buf, PAGE_SIZE, "%.2f\n", temp);
}

/**
 * humidity_show - Sysfs interface to read humidity
 */
static ssize_t humidity_show(struct device *dev, struct device_attribute *attr, char *buf) {
    mutex_lock(&measurement_mutex);
    if (measurement_status != 0) {
        mutex_unlock(&measurement_mutex);
        return -EIO;
    }
    float hum = convert_humidity(measurement);
    mutex_unlock(&measurement_mutex);
    return scnprintf(buf, PAGE_SIZE, "%.2f\n", hum);
}

/**
 * status_show - Sysfs interface to read status register
 */
static ssize_t status_show(struct device *dev, struct device_attribute *attr, char *buf) {
    unsigned char status[STATUS_LEN];
    int i2cret = i2c_master_send(sht_i2c_client, cmd_status_read, COMMAND_LEN);
    if (i2cret != COMMAND_LEN) {
        PRINTK(KERN_ERR, "Status read command failed: %d\n", i2cret);
        return -EIO;
    }
    i2cret = i2c_master_recv(sht_i2c_client, status, STATUS_LEN);
    if (i2cret != STATUS_LEN) {
        PRINTK(KERN_ERR, "Status read failed: %d\n", i2cret);
        return -EIO;
    }
    return scnprintf(buf, PAGE_SIZE, "0x%04x\n", ((status[0] << 8) | status[1]));
}

static struct device_attribute dev_attr_temperature = {
    .attr = {.name = "temperature", .mode = 0444},
    .show = temperature_show,
};

static struct device_attribute dev_attr_humidity = {
    .attr = {.name = "humidity", .mode = 0444},
    .show = humidity_show,
};

static struct device_attribute dev_attr_status = {
    .attr = {.name = "status", .mode = 0444},
    .show = status_show,
};

/**
 * sht3x_driver_init - Initialize the SHT3x driver
 * Returns: 0 on success, negative error code on failure
 */
static int __init sht3x_driver_init(void) {
    int ret;

    // Validate slave address
    if (slave_addr != SLAVE_ADDR && slave_addr != SLAVE_ADDR_ALT) {
        PRINTK(KERN_ERR, "Invalid slave address: 0x%02X\n", slave_addr);
        return -EINVAL;
    }

    // Create workqueue for periodic measurements
    workqueue = create_singlethread_workqueue(SLAVE_DEVICE_NAME);
    if (!workqueue) {
        PRINTK(KERN_ERR, "Failed to create workqueue\n");
        return -ENOMEM;
    }

    // Initialize I2C adapter
    sht_i2c_board_info.addr = slave_addr;
    sht_i2c_adapter = i2c_get_adapter(I2C_BUS_AVAILABLE);
    if (!sht_i2c_adapter) {
        PRINTK(KERN_ERR, "Failed to get I2C adapter for bus %d\n", I2C_BUS_AVAILABLE);
        ret = -ENODEV;
        goto err_workqueue;
    }

    // Register I2C client
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,8,0)
    sht_i2c_client = i2c_new_client_device(sht_i2c_adapter, &sht_i2c_board_info);
    if (IS_ERR(sht_i2c_client)) {
        ret = PTR_ERR(sht_i2c_client);
#else
    sht_i2c_client = i2c_new_device(sht_i2c_adapter, &sht_i2c_board_info);
    if (!sht_i2c_client) {
        ret = -ENODEV;
#endif
        PRINTK(KERN_ERR, "Failed to create I2C client: %d\n", ret);
        goto err_adapter;
    }

    // Register I2C driver
    ret = i2c_add_driver(&sht_i2c_driver);
    if (ret) {
        PRINTK(KERN_ERR, "Failed to add I2C driver: %d\n", ret);
        goto err_client;
    }
    i2c_put_adapter(sht_i2c_adapter);

    // Allocate character device
    ret = alloc_chrdev_region(&dev, 0, 1, "sht3x");
    if (ret < 0) {
        PRINTK(KERN_ERR, "Failed to allocate major number: %d\n", ret);
        goto err_i2c_driver;
    }

    // Initialize and add character device
    cdev_init(&dev_cdev, &fops);
    ret = cdev_add(&dev_cdev, dev, 1);
    if (ret < 0) {
        PRINTK(KERN_ERR, "Failed to add character device: %d\n", ret);
        goto err_chrdev;
    }

    // Create device class
    dev_class = class_create(THIS_MODULE, "sht3x_class");
    if (IS_ERR(dev_class)) {
        ret = PTR_ERR(dev_class);
        PRINTK(KERN_ERR, "Failed to create device class: %d\n", ret);
        goto err_cdev;
    }

    // Create device file
    snprintf(&deventry[5], 3, "%02X", slave_addr);
    if (!device_create(dev_class, NULL, dev, NULL, deventry)) {
        PRINTK(KERN_ERR, "Failed to create device file\n");
        ret = -ENODEV;
        goto err_class;
    }

    // Create sysfs files
    ret = device_create_file(device_create(dev_class, NULL, dev, NULL, deventry), &dev_attr_temperature);
    ret |= device_create_file(device_create(dev_class, NULL, dev, NULL, deventry), &dev_attr_humidity);
    ret |= device_create_file(device_create(dev_class, NULL, dev, NULL, deventry), &dev_attr_status);
    if (ret) {
        PRINTK(KERN_ERR, "Failed to create sysfs files\n");
        goto err_device;
    }

    // Initialize timer
    timer_setup(&periodic_timer, periodic_timer_callback, 0);
    PRINTK(KERN_INFO, "Driver initialized for slave at 0x%02X\n", slave_addr);
    return 0;

err_device:
    device_destroy(dev_class, dev);
err_class:
    class_destroy(dev_class);
err_cdev:
    cdev_del(&dev_cdev);
err_chrdev:
    unregister_chrdev_region(dev, 1);
err_i2c_driver:
    i2c_del_driver(&sht_i2c_driver);
err_client:
    i2c_unregister_device(sht_i2c_client);
err_adapter:
    i2c_put_adapter(sht_i2c_adapter);
err_workqueue:
    destroy_workqueue(workqueue);
    return ret;
}

/**
 * sht3x_driver_exit - Cleanup and exit the driver
 */
static void __exit sht3x_driver_exit(void) {
    del_timer_sync(&periodic_timer);
    device_remove_file(device_create(dev_class, NULL, dev, NULL, deventry), &dev_attr_temperature);
    device_remove_file(device_create(dev_class, NULL, dev, NULL, deventry), &dev_attr_humidity);
    device_remove_file(device_create(dev_class, NULL, dev, NULL, deventry), &dev_attr_status);
    device_destroy(dev_class, dev);
    class_destroy(dev_class);
    cdev_del(&dev_cdev);
    unregister_chrdev_region(dev, 1);
    i2c_unregister_device(sht_i2c_client);
    i2c_del_driver(&sht_i2c_driver);
    destroy_workqueue(workqueue);
    PRINTK(KERN_INFO, "Driver unloaded\n");
}

/**
 * i2c_probe - Probe I2C device
 */
static int i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    i2c_master_send(client, cmd_break, COMMAND_LEN);
    i2c_master_send(client, cmd_status_clear, COMMAND_LEN);
    PRINTK(KERN_INFO, "I2C device probed\n");
    return 0;
}

/**
 * i2c_remove - Remove I2C device
 */
static int i2c_remove(struct i2c_client *client) {
    i2c_master_send(client, cmd_break, COMMAND_LEN);
    i2c_master_send(client, cmd_status_clear, COMMAND_LEN);
    PRINTK(KERN_INFO, "I2C device removed\n");
    return 0;
}

/**
 * i2c_suspend - Suspend the device
 */
static int i2c_suspend(struct device *dev) {
    if (measurement_mode != SHT3X_SINGLE_SHOT_LOW) {
        break_and_cleanup();
    }
    PRINTK(KERN_INFO, "Device suspended\n");
    return 0;
}

/**
 * i2c_resume - Resume the device
 */
static int i2c_resume(struct device *dev) {
    i2c_master_send(sht_i2c_client, cmd_break, COMMAND_LEN);
    i2c_master_send(sht_i2c_client, cmd_status_clear, COMMAND_LEN);
    if (measurement_mode >= SHT3X_PERIODIC_0P5_LOW) {
        i2c_master_send(sht_i2c_client, measurement_commands[measurement_mode], COMMAND_LEN);
        mod_timer(&periodic_timer, jiffies + msecs_to_jiffies(periodic_time_ms()));
    }
    PRINTK(KERN_INFO, "Device resumed\n");
    return 0;
}

/**
 * fops_open - Open the device file
 */
static int fops_open(struct inode *inode, struct file *file) {
    PRINTK(KERN_INFO, "Opening device: %p\n", file);
    if (atomic_inc_return(&dev_use_count) == 1) {
        i2c_master_send(sht_i2c_client, cmd_break, COMMAND_LEN);
        i2c_master_send(sht_i2c_client, cmd_status_clear, COMMAND_LEN);
    }
    file->private_data = (void *)(long)measurement_count;
    return 0;
}

/**
 * fops_release - Release the device file
 */
static int fops_release(struct inode *inode, struct file *file) {
    PRINTK(KERN_INFO, "Closing device: %p\n", file);
    if (atomic_dec_and_test(&dev_use_count)) {
        if (measurement_mode != SHT3X_SINGLE_SHOT_LOW) {
            break_and_cleanup();
        }
    }
    return 0;
}

/**
 * fops_ioctl - Handle IOCTL commands
 */
static long fops_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    int i2cret;
    unsigned char status[STATUS_LEN];

    PRINTK(KERN_INFO, "IOCTL cmd=0x%X arg=%ld\n", cmd, arg);

    switch (cmd) {
    case SHT3X_BREAK:
        if (break_and_cleanup()) {
            return -EIO;
        }
        break;

    case SHT3X_HEATER_CONTROL:
        if (arg == SHT3X_HEATER_DISABLE) {
            i2cret = i2c_master_send(sht_i2c_client, cmd_heater_disable, COMMAND_LEN);
        } else if (arg == SHT3X_HEATER_ENABLE) {
            i2cret = i2c_master_send(sht_i2c_client, cmd_heater_enable, COMMAND_LEN);
        } else {
            PRINTK(KERN_ERR, "Invalid heater control argument: %ld\n", arg);
            return -EINVAL;
        }
        if (i2cret != COMMAND_LEN) {
            PRINTK(KERN_ERR, "Heater command failed: %d\n", i2cret);
            return -EIO;
        }
        break;

    case SHT3X_STATUS:
        if (arg == SHT3X_STATUS_READ) {
            i2cret = i2c_master_send(sht_i2c_client, cmd_status_read, COMMAND_LEN);
            if (i2cret != COMMAND_LEN) {
                PRINTK(KERN_ERR, "Status read command failed: %d\n", i2cret);
                return -EIO;
            }
            i2cret = i2c_master_recv(sht_i2c_client, status, STATUS_LEN);
            if (i2cret != STATUS_LEN) {
                PRINTK(KERN_ERR, "Status read failed: %d\n", i2cret);
                return -EIO;
            }
            return ((long)status[0] << 16) | ((long)status[1] << 8) | (long)status[2];
        } else if (arg == SHT3X_STATUS_CLEAR) {
            i2cret = i2c_master_send(sht_i2c_client, cmd_status_clear, COMMAND_LEN);
            if (i2cret != COMMAND_LEN) {
                PRINTK(KERN_ERR, "Status clear command failed: %d\n", i2cret);
                return -EIO;
            }
        } else {
            PRINTK(KERN_ERR, "Invalid status argument: %ld\n", arg);
            return -EINVAL;
        }
        break;

    case SHT3X_MEASUREMENT_MODE:
        if (arg >= ARRAY_SIZE(measurement_commands)) {
            PRINTK(KERN_ERR, "Invalid measurement mode: %ld\n", arg);
            return -EINVAL;
        }
        if (measurement_mode == arg) {
            return 0;
        }
        if (measurement_mode != SHT3X_SINGLE_SHOT_LOW) {
            PRINTK(KERN_ERR, "Break required before changing measurement mode\n");
            return -EINVAL;
        }
        measurement_mode = arg;
        PRINTK(KERN_INFO, "Measurement mode set to %d\n", measurement_mode);
        if (arg >= SHT3X_PERIODIC_0P5_LOW) {
            i2cret = i2c_master_send(sht_i2c_client, measurement_commands[arg], COMMAND_LEN);
            if (i2cret != COMMAND_LEN) {
                PRINTK(KERN_ERR, "Measurement command failed: %d\n", i2cret);
                return -EIO;
            }
            mod_timer(&periodic_timer, jiffies + msecs_to_jiffies(periodic_time_ms()));
        }
        break;

    case SHT3X_CRC_CHECK:
        if (arg == SHT3X_CRC_CHECK_DISABLE) {
            crc_check_enabled = SHT3X_CRC_CHECK_DISABLE;
            PRINTK(KERN_INFO, "CRC check disabled\n");
        } else if (arg == SHT3X_CRC_CHECK_ENABLE) {
            crc_check_enabled = SHT3X_CRC_CHECK_ENABLE;
            PRINTK(KERN_INFO, "CRC check enabled\n");
        } else {
            PRINTK(KERN_ERR, "Invalid CRC check argument: %ld\n", arg);
            return -EINVAL;
        }
        break;

    default:
        PRINTK(KERN_ERR, "Invalid IOCTL command: 0x%X\n", cmd);
        return -EINVAL;
    }
    return 0;
}

/**
 * fops_read - Read measurement data
 */
static ssize_t fops_read(struct file *filp, char __user *buf, size_t len, loff_t *off) {
    unsigned char single_measurement[MEASUREMENT_LEN];
    int _count;

    if (len > MEASUREMENT_LEN) len = MEASUREMENT_LEN;

    if (measurement_mode <= SHT3X_SINGLE_SHOT_HIGH) {
        int ret = i2c_master_send(sht_i2c_client, measurement_commands[measurement_mode], COMMAND_LEN);
        if (ret != COMMAND_LEN) {
            PRINTK(KERN_ERR, "Single-shot send failed: %d\n", ret);
            return -EIO;
        }
        msleep(15); // Wait for measurement (max 15ms for high repeatability)
        ret = i2c_master_recv(sht_i2c_client, single_measurement, MEASUREMENT_LEN);
        if (ret != MEASUREMENT_LEN) {
            PRINTK(KERN_ERR, "Single-shot read failed: %d\n", ret);
            return -EIO;
        }
        if (crc_check_enabled && validate_measurement_crc(single_measurement)) {
            PRINTK(KERN_ERR, "Single-shot CRC check failed\n");
            return -EIO;
        }
        if (copy_to_user(buf, single_measurement, len)) {
            return -EFAULT;
        }
        return len;
    }

    _count = (int)(long)filp->private_data;
    mutex_lock(&measurement_mutex);
    while (_count == measurement_count && measurement_status == 0) {
        mutex_unlock(&measurement_mutex);
        if (filp->f_flags & O_NONBLOCK) {
            return -EAGAIN;
        }
        if (wait_event_interruptible(measurement_wait, (_count != measurement_count || measurement_status != 0))) {
            return -ERESTARTSYS;
        }
        mutex_lock(&measurement_mutex);
    }
    if (measurement_status != 0) {
        mutex_unlock(&measurement_mutex);
        return -EIO;
    }
    if (crc_check_enabled && validate_measurement_crc(measurement)) {
        PRINTK(KERN_ERR, "Periodic CRC check failed\n");
        mutex_unlock(&measurement_mutex);
        return -EIO;
    }
    if (copy_to_user(buf, measurement, len)) {
        mutex_unlock(&measurement_mutex);
        return -EFAULT;
    }
    _count = measurement_count;
    mutex_unlock(&measurement_mutex);
    filp->private_data = (void *)(long)_count;
    return len;
}

/**
 * fops_write - Write custom I2C command
 */
static ssize_t fops_write(struct file *filp, const char __user *buf, size_t len, loff_t *off) {
    unsigned char cmd[COMMAND_LEN];
    if (len != COMMAND_LEN) {
        PRINTK(KERN_ERR, "Invalid write length: %zu, expected %d\n", len, COMMAND_LEN);
        return -EINVAL;
    }
    if (copy_from_user(cmd, buf, COMMAND_LEN)) {
        return -EFAULT;
    }
    int ret = i2c_master_send(sht_i2c_client, cmd, COMMAND_LEN);
    if (ret != COMMAND_LEN) {
        PRINTK(KERN_ERR, "Write command failed: %d\n", ret);
        return -EIO;
    }
    return len;
}

/**
 * fops_poll - Poll for new measurement data
 */
static unsigned int fops_poll(struct file *filp, struct poll_table_struct *wait) {
    unsigned int mask = 0;
    int _count = (int)(long)filp->private_data;

    mutex_lock(&measurement_mutex);
    poll_wait(filp, &measurement_wait, wait);
    if (_count != measurement_count || measurement_status != 0) {
        mask |= POLLIN | POLLRDNORM;
    }
    mutex_unlock(&measurement_mutex);
    return mask;
}

/**
 * periodic_timer_callback - Timer callback for periodic measurements
 */
static void periodic_timer_callback(struct timer_list *data) {
    if (!queue_work(workqueue, &worker)) {
        PRINTK(KERN_WARNING, "Work queue busy\n");
    }
}

/**
 * worker_read_measurement - Work function to read periodic measurements
 */
static void worker_read_measurement(struct work_struct *work) {
    static int read_failed = 0;
    unsigned char sensor_data[MEASUREMENT_LEN];
    int i2cret;

    i2cret = i2c_master_recv(sht_i2c_client, sensor_data, MEASUREMENT_LEN);
    mutex_lock(&measurement_mutex);
    if (i2cret != MEASUREMENT_LEN) {
        read_failed++;
        PRINTK(KERN_WARNING, "Periodic read failed (%d): %d\n", read_failed, i2cret);
        if (read_failed >= MAX_READ_RETRIES) {
            PRINTK(KERN_ERR, "Too many read failures, issuing break\n");
            break_and_cleanup();
            read_failed = 0;
        } else {
            mod_timer(&periodic_timer, jiffies + msecs_to_jiffies(100));
        }
        measurement_status = 1;
    } else {
        read_failed = 0;
        if (!crc_check_enabled || validate_measurement_crc(sensor_data) == 0) {
            measurement_count++;
            memcpy(measurement, sensor_data, MEASUREMENT_LEN);
            measurement_status = 0;
        } else {
            PRINTK(KERN_ERR, "Periodic CRC check failed\n");
            measurement_status = 1;
        }
        mod_timer(&periodic_timer, jiffies + msecs_to_jiffies(periodic_time_ms()));
    }
    mutex_unlock(&measurement_mutex);
    wake_up_interruptible(&measurement_wait);
}

/**
 * break_and_cleanup - Stop periodic measurements and reset state
 */
static int break_and_cleanup(void) {
    int i2cret;
    del_timer_sync(&periodic_timer);
    cancel_work_sync(&worker);
    mutex_lock(&measurement_mutex);
    measurement_status = 0;
    measurement_mode = SHT3X_SINGLE_SHOT_LOW;
    mutex_unlock(&measurement_mutex);
    i2cret = i2c_master_send(sht_i2c_client, cmd_break, COMMAND_LEN);
    if (i2cret != COMMAND_LEN) {
        PRINTK(KERN_ERR, "Break command failed: %d\n", i2cret);
        return -EIO;
    }
    return 0;
}

/**
 * periodic_time_ms - Get timer interval for periodic measurements
 */
static int periodic_time_ms(void) {
    switch (measurement_mode) {
    case SHT3X_PERIODIC_0P5_LOW:
    case SHT3X_PERIODIC_0P5_MED:
    case SHT3X_PERIODIC_0P5_HIGH:
        return 2000;
    case SHT3X_PERIODIC_1_LOW:
    case SHT3X_PERIODIC_1_MED:
    case SHT3X_PERIODIC_1_HIGH:
        return 1000;
    case SHT3X_PERIODIC_2_LOW:
    case SHT3X_PERIODIC_2_MED:
    case SHT3X_PERIODIC_2_HIGH:
        return 500;
    case SHT3X_PERIODIC_4_LOW:
    case SHT3X_PERIODIC_4_MED:
    case SHT3X_PERIODIC_4_HIGH:
        return 250;
    case SHT3X_PERIODIC_10_LOW:
    case SHT3X_PERIODIC_10_MED:
    case SHT3X_PERIODIC_10_HIGH:
        return 100;
    default:
        PRINTK(KERN_ERR, "Invalid measurement mode: %d\n", measurement_mode);
        return 1000;
    }
}

module_init(sht3x_driver_init);
module_exit(sht3x_driver_exit);