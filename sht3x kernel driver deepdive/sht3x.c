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
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <linux/mutex.h>

#include "sht3x_ioctl.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("NguyenNhan");
MODULE_DESCRIPTION("Enhanced SHT3x I2C driver for temperature and humidity sensing with improvements");
MODULE_VERSION("0.4");

#define pr_fmt(fmt) "SHT3X: " fmt

#define I2C_BUS_AVAILABLE   (1)              // Default I2C bus number
#define SLAVE_DEVICE_NAME   ("SHT3X")        // Device and driver name
#define SLAVE_ADDR          (0x44)           // SHT3x default I2C address
#define SLAVE_ADDR_ALT      (0x45)           // SHT3x alternate I2C address

#define COMMAND_LEN         (2)               // Length of I2C command
#define STATUS_LEN          (3)               // Length of status register data
#define MEASUREMENT_LEN     (6)               // Length of measurement data (2B temp + 1B CRC + 2B hum + 1B CRC)
#define MAX_READ_RETRIES    (3)               // Maximum retries for failed reads
#define RING_BUFFER_SIZE    (10)              // Size for mmap ring buffer

// SHT3x command set (unchanged)
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
static int custom_interval_ms = 0;  // 0 for default
static unsigned char measurement[MEASUREMENT_LEN];
static int measurement_count = 0;
static int measurement_status = 0;
static DEFINE_MUTEX(measurement_mutex);
static DEFINE_MUTEX(init_mutex);  // For init in open
static DECLARE_WAIT_QUEUE_HEAD(measurement_wait);

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
    SET_SYSTEM_SLEEP_PM_OPS(i2c_suspend, i2c_resume)
};

// I2C driver structure
static struct i2c_driver sht_i2c_driver = {
    .driver = {
        .name = SLAVE_DEVICE_NAME,
        .owner = THIS_MODULE,
        .pm = &sht3x_pm_ops,
    },
    .probe_new = i2c_probe,  // Dynamic probe
    .disconnect = i2c_remove,
    .id_table = i2c_id,
};

// Character device structures
static dev_t dev = 0;
static struct class *dev_class;
static struct cdev dev_cdev;
static char deventry[] = "sht3x  ";
static atomic_t dev_use_count = ATOMIC_INIT(0);
static struct device *sht_device = NULL;  // Global device pointer

// File operations
static int fops_open(struct inode *inode, struct file *file);
static int fops_release(struct inode *inode, struct file *file);
static ssize_t fops_read(struct file *filp, char __user *buf, size_t len, loff_t *off);
static ssize_t fops_write(struct file *filp, const char __user *buf, size_t len, loff_t *off);
static long fops_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static unsigned int fops_poll(struct file *filp, struct poll_table_struct *wait);
static int fops_mmap(struct file *filp, struct vm_area_struct *vma);

static struct file_operations fops = {
    .owner          = THIS_MODULE,
    .read           = fops_read,
    .write          = fops_write,
    .open           = fops_open,
    .unlocked_ioctl = fops_ioctl,
    .poll           = fops_poll,
    .release        = fops_release,
    .mmap           = fops_mmap,
};

// Timer and workqueue for periodic measurements
static struct timer_list periodic_timer;
static void periodic_timer_callback(struct timer_list *data);
static void worker_read_measurement(struct work_struct *work);
DECLARE_WORK(worker, worker_read_measurement);
static struct workqueue_struct *workqueue = NULL;

// Mmap ring buffer for file management
static unsigned char *ring_buffer = NULL;
static int ring_head = 0;

// Sysfs attributes
static struct device_attribute dev_attr_temperature;
static struct device_attribute dev_attr_humidity;
static struct device_attribute dev_attr_status;
static struct device_attribute dev_attr_heater;  // New writable for heater
static struct device_attribute dev_attr_mode;    // New writable for mode

// Procfs
static struct proc_dir_entry *proc_entry = NULL;
static int proc_show(struct seq_file *m, void *v);

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
static int sht_i2c_send(const unsigned char *cmd, int len, int retries);
static int sht_i2c_recv(unsigned char *buf, int len, int retries);

// Sysfs show functions (unchanged except pr_)
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

static ssize_t status_show(struct device *dev, struct device_attribute *attr, char *buf) {
    unsigned char status[STATUS_LEN];
    if (sht_i2c_send(cmd_status_read, COMMAND_LEN, MAX_READ_RETRIES) != 0) {
        pr_err("Status read command failed\n");
        return -EIO;
    }
    if (sht_i2c_recv(status, STATUS_LEN, MAX_READ_RETRIES) != 0) {
        pr_err("Status read failed\n");
        return -EIO;
    }
    // Validate CRC for status if enabled (simple, assume 2 bytes data + CRC)
    u8 crc_calc = calculate_crc8(status, 2);
    if (crc_check_enabled && crc_calc != status[2]) {
        pr_err("Status CRC failed\n");
        return -EIO;
    }
    return scnprintf(buf, PAGE_SIZE, "0x%04x\n", ((status[0] << 8) | status[1]));
}

// New: Heater sysfs (writable)
static ssize_t heater_show(struct device *dev, struct device_attribute *attr, char *buf) {
    // Assume heater state from last command, or read status bit (bit 6 in status for heater)
    unsigned char status[STATUS_LEN];
    sht_i2c_send(cmd_status_read, COMMAND_LEN, MAX_READ_RETRIES);
    sht_i2c_recv(status, STATUS_LEN, MAX_READ_RETRIES);
    int heater_on = (status[0] & 0x40) ? 1 : 0;  // Bit 6
    return scnprintf(buf, PAGE_SIZE, "%d\n", heater_on);
}

static ssize_t heater_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    int val;
    if (kstrtoint(buf, 10, &val) || (val != 0 && val != 1)) return -EINVAL;
    const unsigned char *cmd = val ? cmd_heater_enable : cmd_heater_disable;
    if (sht_i2c_send(cmd, COMMAND_LEN, MAX_READ_RETRIES) != 0) return -EIO;
    return count;
}

// New: Mode sysfs (writable)
static ssize_t mode_show(struct device *dev, struct device_attribute *attr, char *buf) {
    return scnprintf(buf, PAGE_SIZE, "%d\n", measurement_mode);
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    int val;
    if (kstrtoint(buf, 10, &val) < 0 || val < 0 || val >= ARRAY_SIZE(measurement_commands)) return -EINVAL;
    if (measurement_mode != SHT3X_SINGLE_SHOT_LOW) {
        if (break_and_cleanup()) return -EIO;
    }
    measurement_mode = val;
    if (val >= SHT3X_PERIODIC_0P5_LOW) {
        sht_i2c_send(measurement_commands[val], COMMAND_LEN, MAX_READ_RETRIES);
        mod_timer(&periodic_timer, jiffies + msecs_to_jiffies(periodic_time_ms()));
    }
    return count;
}

static DEVICE_ATTR_RO(temperature);
static DEVICE_ATTR_RO(humidity);
static DEVICE_ATTR_RO(status);
static DEVICE_ATTR_RW(heater);
static DEVICE_ATTR_RW(mode);

// CRC calculation (unchanged)
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

static int validate_measurement_crc(const unsigned char *data) {
    u8 crc_temp = calculate_crc8(data, 2);
    u8 crc_hum = calculate_crc8(data + 3, 2);
    return (crc_temp == data[2] && crc_hum == data[5]) ? 0 : -1;
}

static float convert_temperature(const unsigned char *data) {
    u16 raw_temp = (data[0] << 8) | data[1];
    return -45.0 + 175.0 * ((float)raw_temp / 65535.0);
}

static float convert_humidity(const unsigned char *data) {
    u16 raw_hum = (data[3] << 8) | data[4];
    return 100.0 * ((float)raw_hum / 65535.0);
}

// I2C wrapper with retries
static int sht_i2c_send(const unsigned char *cmd, int len, int retries) {
    int ret, i;
    for (i = 0; i < retries; i++) {
        ret = i2c_master_send(sht_i2c_client, cmd, len);
        if (ret == len) return 0;
        msleep(10);  // Backoff
    }
    return -EIO;
}

static int sht_i2c_recv(unsigned char *buf, int len, int retries) {
    int ret, i;
    for (i = 0; i < retries; i++) {
        ret = i2c_master_recv(sht_i2c_client, buf, len);
        if (ret == len) return 0;
        msleep(10);
    }
    return -EIO;
}

// Procfs show
static int proc_show(struct seq_file *m, void *v) {
    struct task_struct *task;
    mutex_lock(&measurement_mutex);
    seq_printf(m, "Mode: %d, Count: %d, Status: %d, Temp: %.2f, Hum: %.2f\n",
               measurement_mode, measurement_count, measurement_status,
               convert_temperature(measurement), convert_humidity(measurement));
    mutex_unlock(&measurement_mutex);
    // List open processes (simple, via atomic count)
    task = current;  // Example, for full list need more
    seq_printf(m, "Current process: %s (PID %d)\n", task->comm, task->pid);
    return 0;
}

static int proc_open(struct inode *inode, struct file *file) {
    return single_open(file, proc_show, NULL);
}

static const struct proc_ops proc_fops = {
    .proc_open = proc_open,
    .proc_read = seq_read,
    .proc_lseek = seq_lseek,
    .proc_release = single_release,
};

/**
 * sht3x_driver_init - Initialize the SHT3x driver
 */
static int __init sht3x_driver_init(void) {
    int ret;

    // Validate slave address
    if (slave_addr != SLAVE_ADDR && slave_addr != SLAVE_ADDR_ALT) {
        pr_err("Invalid slave address: 0x%02X\n", slave_addr);
        return -EINVAL;
    }

    // Create workqueue (modern)
    workqueue = alloc_workqueue(SLAVE_DEVICE_NAME, WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
    if (!workqueue) {
        pr_err("Failed to create workqueue\n");
        return -ENOMEM;
    }

    // Allocate ring buffer for mmap
    ring_buffer = vmalloc(MEASUREMENT_LEN * RING_BUFFER_SIZE);
    if (!ring_buffer) {
        pr_err("Failed to allocate ring buffer\n");
        ret = -ENOMEM;
        goto err_workqueue;
    }

    // Initialize I2C adapter
    sht_i2c_board_info.addr = slave_addr;
    sht_i2c_adapter = i2c_get_adapter(I2C_BUS_AVAILABLE);
    if (!sht_i2c_adapter) {
        pr_err("Failed to get I2C adapter for bus %d\n", I2C_BUS_AVAILABLE);
        ret = -ENODEV;
        goto err_ring;
    }

    // Register I2C client (use new API)
    sht_i2c_client = i2c_new_client_device(sht_i2c_adapter, &sht_i2c_board_info);
    if (IS_ERR(sht_i2c_client)) {
        ret = PTR_ERR(sht_i2c_client);
        pr_err("Failed to create I2C client: %d\n", ret);
        goto err_adapter;
    }

    // Register I2C driver
    ret = i2c_add_driver(&sht_i2c_driver);
    if (ret) {
        pr_err("Failed to add I2C driver: %d\n", ret);
        goto err_client;
    }
    i2c_put_adapter(sht_i2c_adapter);

    // Allocate character device
    ret = alloc_chrdev_region(&dev, 0, 1, "sht3x");
    if (ret < 0) {
        pr_err("Failed to allocate major number: %d\n", ret);
        goto err_i2c_driver;
    }

    // Initialize and add character device
    cdev_init(&dev_cdev, &fops);
    ret = cdev_add(&dev_cdev, dev, 1);
    if (ret < 0) {
        pr_err("Failed to add character device: %d\n", ret);
        goto err_chrdev;
    }

    // Create device class
    dev_class = class_create(THIS_MODULE, "sht3x_class");
    if (IS_ERR(dev_class)) {
        ret = PTR_ERR(dev_class);
        pr_err("Failed to create device class: %d\n", ret);
        goto err_cdev;
    }

    // Create device file
    snprintf(&deventry[5], 3, "%02X", slave_addr);
    sht_device = device_create(dev_class, NULL, dev, NULL, deventry);
    if (IS_ERR(sht_device)) {
        ret = PTR_ERR(sht_device);
        pr_err("Failed to create device file: %d\n", ret);
        goto err_class;
    }

    // Create sysfs files
    ret = device_create_file(sht_device, &dev_attr_temperature);
    ret |= device_create_file(sht_device, &dev_attr_humidity);
    ret |= device_create_file(sht_device, &dev_attr_status);
    ret |= device_create_file(sht_device, &dev_attr_heater);
    ret |= device_create_file(sht_device, &dev_attr_mode);
    if (ret) {
        pr_err("Failed to create sysfs files\n");
        goto err_device;
    }

    // Create procfs
    proc_entry = proc_create("sht3x", 0444, NULL, &proc_fops);
    if (!proc_entry) {
        pr_warn("Failed to create procfs entry\n");
    }

    // Initialize timer
    timer_setup(&periodic_timer, periodic_timer_callback, 0);
    pr_info("Driver initialized for slave at 0x%02X\n", slave_addr);
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
err_ring:
    vfree(ring_buffer);
err_workqueue:
    destroy_workqueue(workqueue);
    return ret;
}

/**
 * sht3x_driver_exit - Cleanup and exit the driver
 */
static void __exit sht3x_driver_exit(void) {
    del_timer_sync(&periodic_timer);
    if (proc_entry) proc_remove(proc_entry);
    device_remove_file(sht_device, &dev_attr_temperature);
    device_remove_file(sht_device, &dev_attr_humidity);
    device_remove_file(sht_device, &dev_attr_status);
    device_remove_file(sht_device, &dev_attr_heater);
    device_remove_file(sht_device, &dev_attr_mode);
    device_destroy(dev_class, dev);
    class_destroy(dev_class);
    cdev_del(&dev_cdev);
    unregister_chrdev_region(dev, 1);
    i2c_unregister_device(sht_i2c_client);
    i2c_del_driver(&sht_i2c_driver);
    destroy_workqueue(workqueue);
    vfree(ring_buffer);
    pr_info("Driver unloaded\n");
}

/**
 * i2c_probe - Probe I2C device (dynamic)
 */
static int i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    sht_i2c_send(cmd_break, COMMAND_LEN, 1);
    sht_i2c_send(cmd_status_clear, COMMAND_LEN, 1);
    pr_info("I2C device probed at addr 0x%02x\n", client->addr);
    return 0;
}

/**
 * i2c_remove - Remove I2C device
 */
static int i2c_remove(struct i2c_client *client) {
    sht_i2c_send(cmd_break, COMMAND_LEN, 1);
    sht_i2c_send(cmd_status_clear, COMMAND_LEN, 1);
    pr_info("I2C device removed\n");
    return 0;
}

/**
 * i2c_suspend - Suspend the device
 */
static int i2c_suspend(struct device *dev) {
    if (measurement_mode != SHT3X_SINGLE_SHOT_LOW) {
        break_and_cleanup();
    }
    pr_info("Device suspended\n");
    return 0;
}

/**
 * i2c_resume - Resume the device
 */
static int i2c_resume(struct device *dev) {
    sht_i2c_send(cmd_break, COMMAND_LEN, 1);
    sht_i2c_send(cmd_status_clear, COMMAND_LEN, 1);
    if (measurement_mode >= SHT3X_PERIODIC_0P5_LOW) {
        sht_i2c_send(measurement_commands[measurement_mode], COMMAND_LEN, 1);
        int interval = custom_interval_ms ? custom_interval_ms : periodic_time_ms();
        mod_timer(&periodic_timer, jiffies + msecs_to_jiffies(interval));
    }
    pr_info("Device resumed\n");
    return 0;
}

/**
 * fops_open - Open the device file (improved process management)
 */
static int fops_open(struct inode *inode, struct file *file) {
    struct task_struct *task = current;
    pr_info("Process %s (PID %d) opening device\n", task->comm, task->pid);
    if (atomic_inc_return(&dev_use_count) == 1) {
        mutex_lock(&init_mutex);
        sht_i2c_send(cmd_break, COMMAND_LEN, 1);
        sht_i2c_send(cmd_status_clear, COMMAND_LEN, 1);
        mutex_unlock(&init_mutex);
    }
    file->private_data = (void *)(long)measurement_count;
    return 0;
}

/**
 * fops_release - Release the device file
 */
static int fops_release(struct inode *inode, struct file *file) {
    struct task_struct *task = current;
    pr_info("Process %s (PID %d) closing device\n", task->comm, task->pid);
    if (atomic_dec_and_test(&dev_use_count)) {
        if (measurement_mode != SHT3X_SINGLE_SHOT_LOW) {
            break_and_cleanup();
        }
    }
    return 0;
}

/**
 * fops_ioctl - Handle IOCTL commands (extended)
 */
static long fops_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    int i2cret;
    unsigned char status[STATUS_LEN];
    struct sht3x_config config;

    pr_info("IOCTL cmd=0x%X arg=%ld by PID %d\n", cmd, arg, current->pid);

    switch (cmd) {
    case SHT3X_BREAK:
        if (break_and_cleanup()) {
            return -EIO;
        }
        break;

    case SHT3X_HEATER_CONTROL:
        if (arg == SHT3X_HEATER_DISABLE) {
            i2cret = sht_i2c_send(cmd_heater_disable, COMMAND_LEN, MAX_READ_RETRIES);
        } else if (arg == SHT3X_HEATER_ENABLE) {
            i2cret = sht_i2c_send(cmd_heater_enable, COMMAND_LEN, MAX_READ_RETRIES);
        } else {
            pr_err("Invalid heater control argument: %ld\n", arg);
            return -EINVAL;
        }
        if (i2cret != 0) {
            pr_err("Heater command failed\n");
            return -EIO;
        }
        break;

    case SHT3X_STATUS:
        if (arg == SHT3X_STATUS_READ) {
            i2cret = sht_i2c_send(cmd_status_read, COMMAND_LEN, MAX_READ_RETRIES);
            if (i2cret != 0) {
                pr_err("Status read command failed\n");
                return -EIO;
            }
            i2cret = sht_i2c_recv(status, STATUS_LEN, MAX_READ_RETRIES);
            if (i2cret != 0) {
                pr_err("Status read failed\n");
                return -EIO;
            }
            // Fix: Correct shift for 2-byte status
            return ((long)status[0] << 8) | (long)status[1];
        } else if (arg == SHT3X_STATUS_CLEAR) {
            i2cret = sht_i2c_send(cmd_status_clear, COMMAND_LEN, MAX_READ_RETRIES);
            if (i2cret != 0) {
                pr_err("Status clear command failed\n");
                return -EIO;
            }
        } else {
            pr_err("Invalid status argument: %ld\n", arg);
            return -EINVAL;
        }
        break;

    case SHT3X_MEASUREMENT_MODE:
        if (arg >= ARRAY_SIZE(measurement_commands)) {
            pr_err("Invalid measurement mode: %ld\n", arg);
            return -EINVAL;
        }
        if (measurement_mode != SHT3X_SINGLE_SHOT_LOW) {
            pr_err("Break required before changing measurement mode\n");
            return -EINVAL;
        }
        measurement_mode = arg;
        pr_info("Measurement mode set to %d\n", measurement_mode);
        if (arg >= SHT3X_PERIODIC_0P5_LOW) {
            i2cret = sht_i2c_send(measurement_commands[arg], COMMAND_LEN, MAX_READ_RETRIES);
            if (i2cret != 0) {
                pr_err("Measurement command failed\n");
                return -EIO;
            }
            int interval = custom_interval_ms ? custom_interval_ms : periodic_time_ms();
            mod_timer(&periodic_timer, jiffies + msecs_to_jiffies(interval));
        }
        break;

    case SHT3X_CRC_CHECK:
        if (arg == SHT3X_CRC_CHECK_DISABLE) {
            crc_check_enabled = SHT3X_CRC_CHECK_DISABLE;
            pr_info("CRC check disabled\n");
        } else if (arg == SHT3X_CRC_CHECK_ENABLE) {
            crc_check_enabled = SHT3X_CRC_CHECK_ENABLE;
            pr_info("CRC check enabled\n");
        } else {
            pr_err("Invalid CRC check argument: %ld\n", arg);
            return -EINVAL;
        }
        break;

    case SHT3X_SET_INTERVAL:
        if (arg < 50 || arg > 10000) {  // Reasonable range
            pr_err("Invalid interval: %ld ms\n", arg);
            return -EINVAL;
        }
        custom_interval_ms = arg;
        pr_info("Custom interval set to %ld ms\n", arg);
        // Restart timer if periodic
        if (measurement_mode >= SHT3X_PERIODIC_0P5_LOW) {
            mod_timer(&periodic_timer, jiffies + msecs_to_jiffies(custom_interval_ms));
        }
        break;

    case SHT3X_GET_CONFIG:
        mutex_lock(&measurement_mutex);
        config.mode = measurement_mode;
        config.heater = 0;  // Placeholder, can read from status
        config.crc_check = crc_check_enabled;
        config.interval_ms = custom_interval_ms;
        // Get status
        sht_i2c_send(cmd_status_read, COMMAND_LEN, 1);
        sht_i2c_recv(status, STATUS_LEN, 1);
        config.status = (status[0] << 8) | status[1];
        mutex_unlock(&measurement_mutex);
        if (copy_to_user((void __user *)arg, &config, sizeof(config))) {
            return -EFAULT;
        }
        break;

    default:
        pr_err("Invalid IOCTL command: 0x%X\n", cmd);
        return -EINVAL;
    }
    return 0;
}

/**
 * fops_read - Read measurement data (improved wait)
 */
static ssize_t fops_read(struct file *filp, char __user *buf, size_t len, loff_t *off) {
    unsigned char single_measurement[MEASUREMENT_LEN];
    int _count, ret;
    int wait_time_ms;

    if (len > MEASUREMENT_LEN) len = MEASUREMENT_LEN;

    if (measurement_mode <= SHT3X_SINGLE_SHOT_HIGH) {
        ret = sht_i2c_send(measurement_commands[measurement_mode], COMMAND_LEN, MAX_READ_RETRIES);
        if (ret != 0) {
            pr_err("Single-shot send failed\n");
            return -EIO;
        }
        // Dynamic wait based on mode
        wait_time_ms = (measurement_mode == SHT3X_SINGLE_SHOT_LOW) ? 4 :
                       (measurement_mode == SHT3X_SINGLE_SHOT_MED) ? 12 : 15;
        msleep(wait_time_ms);
        ret = sht_i2c_recv(single_measurement, MEASUREMENT_LEN, MAX_READ_RETRIES);
        if (ret != 0) {
            pr_err("Single-shot read failed\n");
            return -EIO;
        }
        if (crc_check_enabled && validate_measurement_crc(single_measurement)) {
            pr_err("Single-shot CRC check failed\n");
            return -EIO;
        }
        if (copy_to_user(buf, single_measurement, len)) {
            return -EFAULT;
        }
        // Copy to ring buffer
        memcpy(&ring_buffer[ring_head * MEASUREMENT_LEN], single_measurement, MEASUREMENT_LEN);
        ring_head = (ring_head + 1) % RING_BUFFER_SIZE;
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
        pr_err("Periodic CRC check failed\n");
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
    // Update ring buffer
    memcpy(&ring_buffer[ring_head * MEASUREMENT_LEN], measurement, MEASUREMENT_LEN);
    ring_head = (ring_head + 1) % RING_BUFFER_SIZE;
    return len;
}

/**
 * fops_write - Write custom I2C command
 */
static ssize_t fops_write(struct file *filp, const char __user *buf, size_t len, loff_t *off) {
    unsigned char cmd[COMMAND_LEN];
    if (len != COMMAND_LEN) {
        pr_err("Invalid write length: %zu, expected %d\n", len, COMMAND_LEN);
        return -EINVAL;
    }
    if (copy_from_user(cmd, buf, COMMAND_LEN)) {
        return -EFAULT;
    }
    int ret = sht_i2c_send(cmd, COMMAND_LEN, MAX_READ_RETRIES);
    if (ret != 0) {
        pr_err("Write command failed\n");
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
 * fops_mmap - Mmap ring buffer for file management
 */
static int fops_mmap(struct file *filp, struct vm_area_struct *vma) {
    size_t size = vma->vm_end - vma->vm_start;
    if (size > MEASUREMENT_LEN * RING_BUFFER_SIZE) return -EINVAL;
    return remap_vmalloc_range(vma, ring_buffer, 0);
}

/**
 * periodic_timer_callback - Timer callback for periodic measurements
 */
static void periodic_timer_callback(struct timer_list *data) {
    if (!queue_work(workqueue, &worker)) {
        pr_warn("Work queue busy\n");
    }
}

/**
 * worker_read_measurement - Work function to read periodic measurements
 */
static void worker_read_measurement(struct work_struct *work) {
    static int read_failed = 0;
    unsigned char sensor_data[MEASUREMENT_LEN];
    int i2cret;

    i2cret = sht_i2c_recv(sensor_data, MEASUREMENT_LEN, MAX_READ_RETRIES);
    mutex_lock(&measurement_mutex);
    if (i2cret != 0) {
        read_failed++;
        pr_warn("Periodic read failed (%d)\n", read_failed);
        if (read_failed >= MAX_READ_RETRIES) {
            pr_err("Too many read failures, issuing break\n");
            break_and_cleanup();
            read_failed = 0;
        } else {
            int interval = custom_interval_ms ? custom_interval_ms : 100;
            mod_timer(&periodic_timer, jiffies + msecs_to_jiffies(interval));
        }
        measurement_status = 1;
    } else {
        read_failed = 0;
        if (!crc_check_enabled || validate_measurement_crc(sensor_data) == 0) {
            measurement_count++;
            memcpy(measurement, sensor_data, MEASUREMENT_LEN);
            measurement_status = 0;
        } else {
            pr_err("Periodic CRC check failed\n");
            measurement_status = 1;
        }
        int interval = custom_interval_ms ? custom_interval_ms : periodic_time_ms();
        mod_timer(&periodic_timer, jiffies + msecs_to_jiffies(interval));
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
    i2cret = sht_i2c_send(cmd_break, COMMAND_LEN, 1);
    if (i2cret != 0) {
        pr_err("Break command failed\n");
        return -EIO;
    }
    return 0;
}

/**
 * periodic_time_ms - Get timer interval for periodic measurements (default)
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
        pr_err("Invalid measurement mode: %d\n", measurement_mode);
        return 1000;
    }
}

module_init(sht3x_driver_init);
module_exit(sht3x_driver_exit);
