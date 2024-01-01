#define DT_DRV_COMPAT ti_drv2605

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdlib.h>
#include <math.h>
#include <zephyr/logging/log.h>

#include "drv2605.h"

LOG_MODULE_REGISTER(DRV2605, CONFIG_SENSOR_LOG_LEVEL);

struct drv2605_data {
    int placeholder;
};

struct drv2605_dev_config {
    struct i2c_dt_spec i2c;
    int motor_type;
    int motor_freq;
    int motor_vrms;
};

static int drv2605_sample_fetch(const struct device *dev, 
                                enum sensor_channel chan) {
	
    return 0;
}

static int drv2605_channel_get(const struct device *dev, 
                               enum sensor_channel chan, 
                               struct sensor_value *val) {

    /// get the register value you want here    
    return 0;
}

static int drv2605_attr_set(const struct device *dev, 
                            enum sensor_channel chan, 
                            enum sensor_attribute attr, 
                            const struct sensor_value *val) {

    const struct drv2605_dev_config *config = dev->config;

    /// change the waveform here

    // check if waveform_index is within range
	if(val->val1 > 123) {
        LOG_ERR("invalid waveform selection");
		return -EINVAL;
	}

	// i2c operation error
	uint8_t i2c_operation_error = 0;

	// Deassert Standby and set to internal trigger
	i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, MODE_REG, 0b00000000);
	if(i2c_operation_error) {
		LOG_ERR("failure at EmitWaveform deasserting standby");
		return -EIO;
	}
	
	// Select waveform index to be played and write it address 0x04
	i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, WAVEFORM_SEQ1_REG, val->val1);
	if(i2c_operation_error) {
		LOG_ERR("failure at EmitWaveform selecting waveform");
		return -EIO;
	}
	
	// Set Go bit to to fire the effect
	i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, GO_REG, 0b00000001);
	if(i2c_operation_error) {
		LOG_ERR("failure at EmitWaveform starting effect");
		return -EIO;
	}

	uint8_t firing = 1;
	while(firing) {
		i2c_operation_error = i2c_reg_read_byte_dt(&config->i2c, GO_REG, &firing);
		if(i2c_operation_error) {
			LOG_ERR("failure at step EmitWaveform firing sequence");
			return -EIO;
		}
	}

	// check for output shorts and over_temp here
	uint8_t physical_error = 0;
	i2c_operation_error = i2c_reg_read_byte_dt(&config->i2c, STATUS_REG, &physical_error);
	if(i2c_operation_error) {
		LOG_ERR("failure at step EmitWaveform reading physical errors");
		return -EIO;
	}

	if((physical_error & 0b00000011)) {
		LOG_ERR("failure at step EmitWaveform with physical errors");
		return -EIO;
	}

	// Put device in low power mode by setting the STANDBY bit
	i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, MODE_REG, 0b01000000);
	if(i2c_operation_error) {
		LOG_ERR("failure at EmitWaveform going back to standby");
		return -EIO;
	}

    return 0;
}


int drv2605_set_initial_registers(const struct device* dev) {

    const struct drv2605_dev_config *config = dev->config;
    
    // i2c operation result
	uint8_t i2c_operation_error = 0;
	
	// take the device out of standby mode
	i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, MODE_REG, 0x00);
	if(i2c_operation_error) {
		LOG_ERR("failed taking drv2605 device out of standby mode before starting auto calibration");
        return -EIO;
	}

	/**
	 * Perform auto-calibration
	 * Steps: 
	 * 0). Write 0x07 to register 0x01
	 * 1). Set ERM_LRA bit of reg 0x1A
	 * 2). Set FB_BRAKE_FACTOR bit field of reg 0x1A
	 * 3). Set LOOP_GAIN bit field of reg 0x1A 
	 * 	1-3). Value is 0b10101010
	 * 4). Set RATED_VOLTAGE bit field of reg 0x16
	 * 	4a). LRA used is the VG0832022D lra from Digikey
	 * 	4b). Voltage of 1.8VAC RMS, frequency of 235Hz
	 * 	4c). formula is Vrms = (20.71*10^-3 * RATED_VOLTAGE)/sqrt(1-(4*SAMPLE_TIME + 300 * 10^-6) * fLRA))
	 * 	4d). SAMPLE_TIME is recommended to be 300uS
	 * 	4e). RATED_VOLTAGE is 83.7524... which is 0x53
	 * 5). Set OD_CLAMP bit field of reg 0x17
	 * 	5a). Vclamp = 21.96 * 10^-3 * OD_CLAMP
	 * 	5b). 1.8 = 21.96 * 10^-3 * OD_CLAMP
	 * 	5c). OD_CLAMP = 81.967 = 0x52
	 * 6). Set AUTO_CAL_TIME bit field of reg 0x1E
	 * 	6a). 0b00110000
	 * 7). Set DRIVE_TIME bit field of reg 0x1B
	 * 	7a). drive time(ms) = 0.5 * LRA Period (0.00426s for 235 Hz) = 2.13ms
	 * 	7b). 2.13ms = DRIVE_TIME * 0.1 + 0.5ms, so DRIVE_TIME = 16
	 * 	7c). 0b10010000
	 * 8). Set SAMPLE_TIME bit field of 0x1C
	 * 9). Set BLANKING_TIME bit field of 0x1C
	 * 10). Set IDISS_TIME bit field of 0x1C
	 * 	8-10). 0b11110101 
	 * 11). Set GO bit by writing 0x01 to register 0x0C - wait for GO bit to clear
	 * 12). Check status of DIAG_RESULT of regiester 0x00 to ensure that calibration is complete without faults
	*/

	// step 0: put device in auto-calibration mode
	i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, MODE_REG, 0x07);
	if(i2c_operation_error) {
		LOG_ERR("failure at step 0 of auto calibration");
		return -EIO;
	}

	// step 1-3: write to FEEDBACK_CONTROL_REG
	i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, FEEDBACK_CONTROL_REG, 0b10101010);
	if(i2c_operation_error) {
		LOG_ERR("failure at step 1-3 of auto calibration");
		return -EIO;
	}

	// step 4: write to RATED_VOLTAGE_REG
	i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, RATED_VOLTAGE_REG, 0x53);
	if(i2c_operation_error) {
		LOG_ERR("failure at step 4 of auto calibration");
		return -EIO;
	}

	// step 5: write to OVERDRIVE_CLAMP_VOLTAGE_REG
	i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, OVERDRIVE_CLAMP_VOLTAGE_REG, 0x52);
	if(i2c_operation_error) {
		LOG_ERR("failure at step 5 of auto calibration");
		return -EIO;
	}

	// step 6: set AUTOCAL_TIME by writing to CONTROL4_REG
	i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, CONTROL4_REG, 0b00110000);
	if(i2c_operation_error) {
		LOG_ERR("failure at step 6 of auto calibration");
		return -EIO;
	}

	// step 7: set DRIVE_TIME by writing to CONTROL1_REG
	i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, CONTROL1_REG, 0b10010000);
	if(i2c_operation_error) {
		LOG_ERR("failure at step 7 of auto calibration");
		return -EIO;
	}

	// step 8-10: write to CONTROL2_REG
	i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, CONTROL2_REG, 0b11110101);
	if(i2c_operation_error) {
		LOG_ERR("failure at step 8-10 of auto calibration");
		return -EIO;
	}

	// step 11: write to GO register to start auto-calibration and wait for bit to deassert
	i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, GO_REG, 0x01);
	if(i2c_operation_error) {
		LOG_ERR("failure at step 11 of auto calibration");
		return -EIO;
	}

	uint8_t calibrating = 1;
	while(calibrating) {
		i2c_operation_error = i2c_reg_read_byte_dt(&config->i2c, GO_REG, &calibrating);
		if(i2c_operation_error) {
			LOG_ERR("failure at step 11 of auto calibration waiting for calibration to be finished");
			return -EIO;
		}
	}

	// step 12: check diag_result to make sure that the auto-calibration is finished correctly
	uint8_t autocalibration_result = 0;
	i2c_operation_error = i2c_reg_read_byte_dt(&config->i2c, STATUS_REG, &autocalibration_result);
	if(i2c_operation_error) {
		LOG_ERR("failure at step 12 of auto calibration");
		return -EIO;
	}

	if(!(autocalibration_result & 0b00001000)) {
		printk("failure at step 12 of auto calibration - result did not converge");
		return false;
	}

	// select LRA waveform library
	i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, LIBRARY_REG, 0b00000110);
	if(i2c_operation_error) {
		LOG_ERR("failure at auto calibration waveform library selection");
		return -EIO;
	}
		
	// put device is standby mode
	i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, MODE_REG, 0b01000000);
	if(i2c_operation_error) {
		LOG_ERR("failure at auto calibration waveform library selection");
		return -EIO;
	}

	return 0;
}

int drv2605_init(const struct device *dev) {

    const struct drv2605_dev_config *config = dev->config;
    
    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("failure with i2c bus");
		return -ENODEV;
	}

    /// initialize sensor here

    if(drv2605_set_initial_registers(dev) < 0) {
        LOG_ERR("failure initializing drv2605");
        return -EINVAL;
    }

	return 0;
}

static const struct sensor_driver_api drv2605_api_funcs = {
	.sample_fetch = drv2605_sample_fetch,
	.channel_get = drv2605_channel_get,
	.attr_set = drv2605_attr_set,
};

#define DRV2605_DEVICE_INIT(inst)					\
	SENSOR_DEVICE_DT_INST_DEFINE(inst,				\
			      drv2605_init,				        \
			      NULL,					            \
			      &drv2605_data_##inst,			    \
			      &drv2605_config_##inst,			\
			      POST_KERNEL,				        \
			      CONFIG_SENSOR_INIT_PRIORITY,		\
			      &drv2605_api_funcs);

#define DRV2605_CONFIG(inst)								\
    {                                                       \
		.i2c = I2C_DT_SPEC_INST_GET(inst),					\
		.motor_type = DT_INST_PROP(inst, motor_type),		\
		.motor_freq = DT_INST_PROP(inst, motor_frequency),  \
		.motor_vrms = DT_INST_PROP(inst, motor_voltagerms)	\
    }                                                       \

#define DRV2605_DEFINE(inst)                                        \
    static struct drv2605_data drv2605_data_##inst;                 \
    static const struct drv2605_dev_config drv2605_config_##inst =	\
        DRV2605_CONFIG(inst);				                        \
    DRV2605_DEVICE_INIT(inst)

DT_INST_FOREACH_STATUS_OKAY(DRV2605_DEFINE)
