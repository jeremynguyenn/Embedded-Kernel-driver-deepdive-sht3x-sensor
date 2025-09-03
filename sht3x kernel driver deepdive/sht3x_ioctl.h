#ifndef SHT3X_IOCTL_H
#define SHT3X_IOCTL_H

#include <linux/ioctl.h>

/**
 * SHT3X IOCTL Commands
 * These commands control the SHT3X sensor's heater, measurement mode, status, and CRC checking.
 */
#define SHT3X_IOC_MAGIC 'x'

#define SHT3X_HEATER_CONTROL    _IOW(SHT3X_IOC_MAGIC, 0x00, int) // Control heater (enable/disable)
#define SHT3X_MEASUREMENT_MODE  _IOW(SHT3X_IOC_MAGIC, 0x01, int) // Set measurement mode
#define SHT3X_BREAK             _IO(SHT3X_IOC_MAGIC, 0x02)       // Stop periodic measurement
#define SHT3X_STATUS            _IOWR(SHT3X_IOC_MAGIC, 0x03, int) // Read/clear status register
#define SHT3X_CRC_CHECK         _IOW(SHT3X_IOC_MAGIC, 0x04, int) // Enable/disable CRC check

// Heater control arguments
#define SHT3X_HEATER_DISABLE    0UL // Disable heater
#define SHT3X_HEATER_ENABLE     1UL // Enable heater

// Measurement mode arguments
#define SHT3X_SINGLE_SHOT_LOW   0UL // Single-shot, low repeatability
#define SHT3X_SINGLE_SHOT_MED   1UL // Single-shot, medium repeatability
#define SHT3X_SINGLE_SHOT_HIGH  2UL // Single-shot, high repeatability
#define SHT3X_PERIODIC_0P5_LOW  3UL // Periodic, 0.5 measurements/s, low repeatability
#define SHT3X_PERIODIC_0P5_MED  4UL // Periodic, 0.5 measurements/s, medium repeatability
#define SHT3X_PERIODIC_0P5_HIGH 5UL // Periodic, 0.5 measurements/s, high repeatability
#define SHT3X_PERIODIC_1_LOW    6UL // Periodic, 1 measurement/s, low repeatability
#define SHT3X_PERIODIC_1_MED    7UL // Periodic, 1 measurement/s, medium repeatability
#define SHT3X_PERIODIC_1_HIGH   8UL // Periodic, 1 measurement/s, high repeatability
#define SHT3X_PERIODIC_2_LOW    9UL // Periodic, 2 measurements/s, low repeatability
#define SHT3X_PERIODIC_2_MED    10UL // Periodic, 2 measurements/s, medium repeatability
#define SHT3X_PERIODIC_2_HIGH   11UL // Periodic, 2 measurements/s, high repeatability
#define SHT3X_PERIODIC_4_LOW    12UL // Periodic, 4 measurements/s, low repeatability
#define SHT3X_PERIODIC_4_MED    13UL // Periodic, 4 measurements/s, medium repeatability
#define SHT3X_PERIODIC_4_HIGH   14UL // Periodic, 4 measurements/s, high repeatability
#define SHT3X_PERIODIC_10_LOW   15UL // Periodic, 10 measurements/s, low repeatability
#define SHT3X_PERIODIC_10_MED   16UL // Periodic, 10 measurements/s, medium repeatability
#define SHT3X_PERIODIC_10_HIGH  17UL // Periodic, 10 measurements/s, high repeatability

// Status command arguments
#define SHT3X_STATUS_READ       0UL // Read status register
#define SHT3X_STATUS_CLEAR      1UL // Clear status register

// CRC check arguments
#define SHT3X_CRC_CHECK_DISABLE 0UL // Disable CRC checking
#define SHT3X_CRC_CHECK_ENABLE  1UL // Enable CRC checking

#endif /* SHT3X_IOCTL_H */