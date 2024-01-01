#ifndef ZEPHYR_DRIVERS_SENSOR_DRV2605_DRV2605
#define ZEPHYR_DRIVERS_SENSOR_DRV2605_DRV2605

#include <zephyr/drivers/sensor.h>

#define STATUS_REG          0x00
#define MODE_REG            0x01
#define LIBRARY_REG         0x03
#define WAVEFORM_SEQ1_REG   0x04
#define GO_REG              0x0C
#define RATED_VOLTAGE_REG   0x16
#define OVERDRIVE_CLAMP_VOLTAGE_REG 0x17
#define FEEDBACK_CONTROL_REG    0x1A
#define CONTROL1_REG        0x1B
#define CONTROL2_REG        0x1C
#define CONTROL4_REG        0x1E

enum drv2605_channel {
    EMIT_WAVEFORM_NUM = SENSOR_CHAN_PRIV_START,
};

enum drv2606_emit_waveform_attribute {
    EMIT_WAVEFORM_START = SENSOR_ATTR_PRIV_START,
    EMIT_WAVEFORM_STOP = SENSOR_ATTR_PRIV_START + 1,
};


#endif