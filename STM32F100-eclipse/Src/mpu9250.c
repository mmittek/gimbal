#include "mpu9250.h"
#include <stdint.h>
#include <math.h>
#include "stm32f1xx_hal.h"
#include <string.h>

static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};

static struct platform_data_s compass_pdata = {
    .orientation = { 0, 1, 0,
                     1, 0, 0,
                     0, 0, -1}
};

unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";
unsigned long last_sensor_timestamp = 0;


/**
 * @return 0 if successful
 */
int mpu9250_set_sampling_rate(mpu9250_t *p_mpu9250, int rate_hz) {
	int result = 0;
	if(!p_mpu9250) return -1;
	if(rate_hz <=0) return -1;
	if(rate_hz > 200)  return -1;

	unsigned long quat_sample_rate = 1000000/rate_hz;

    result = dmp_set_fifo_rate(rate_hz);
    inv_set_quat_sample_rate(quat_sample_rate);
    return result;
}


void mpu9250_sensor_fusion(mpu9250_t* p_mpu9250) {
    long data[9];
    int8_t accuracy = 0;
    unsigned long timestamp = 0;

    if(!p_mpu9250) return;
    if(!p_mpu9250->empl_data_handler) return;

    if (inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*)&timestamp)) {
    	p_mpu9250->empl_data_handler(p_mpu9250, MPU9250_EMPL_DATA_QUAT, data, timestamp);
    }

    if(inv_get_sensor_type_euler(data, &accuracy, (inv_time_t*)&timestamp)) {
    	p_mpu9250->empl_data_handler(p_mpu9250, MPU9250_EMPL_DATA_EULER, data, timestamp);
    }

    if (inv_get_sensor_type_accel(data, &accuracy, (inv_time_t*)&timestamp)) {
    	p_mpu9250->empl_data_handler(p_mpu9250, MPU9250_EMPL_DATA_ACCEL, data, timestamp);
    }
}

/**
 * @return 0 on success
 */
int mpu9250_self_test(mpu9250_t *p_mpu9250) {
    int result;
    long gyro[3], accel[3];
    int i;
	unsigned short accel_sens;
	float gyro_sens;


    result = mpu_run_6500_self_test(gyro, accel, 0);

    if(result & 0x01) {
    	// Gyro passed
		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long) (gyro[0] * gyro_sens);
		gyro[1] = (long) (gyro[1] * gyro_sens);
		gyro[2] = (long) (gyro[2] * gyro_sens);
		inv_set_gyro_bias(gyro, 3);
    }

    if(result & 0x02) {
    	// Accel passed
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		inv_set_accel_bias(accel, 3);
    }

    if(result & 0x04) {
    	// Compas passed
//    	inv_set_compass_bias();
    }

    inv_accel_was_turned_off();
    inv_gyro_was_turned_off();
    inv_compass_was_turned_off();

    // return 0 if result is 0x07
    return (result != 0x07);
}



void ComplementaryFilter(mpu9250_t* p_mpu9250, uint8_t sensors, short accData[3], short gyrData[3], float dt)
{
    float pitchAcc, rollAcc;
    float alpha = 0.95f;

    // Integrate the gyroscope data -> int(angularSpeed) = angle
    if(sensors & INV_XYZ_GYRO) {
		p_mpu9250->pitch += ((float)gyrData[0] /  p_mpu9250->gyro_sens) * dt; // Angle around the X-axis
		p_mpu9250->roll -= ((float)gyrData[1] /  p_mpu9250->gyro_sens) * dt;    // Angle around the Y-axis
		p_mpu9250->yaw += ((float)gyrData[2] / p_mpu9250->gyro_sens) * dt;    // Angle around the Z-axis
    }

    // Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
    if(sensors & INV_XYZ_ACCEL)
//    if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
    {
	// Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
        p_mpu9250->pitch = p_mpu9250->pitch * alpha + pitchAcc * (1.0f-alpha);

	// Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
        p_mpu9250->roll = p_mpu9250->roll * alpha + rollAcc * (1.0f-alpha);
    }
/*
    while(p_mpu9250->roll > 360) p_mpu9250->roll -= 360;
    while(p_mpu9250->pitch > 360) p_mpu9250->pitch -= 360;
    while(p_mpu9250->roll < 0) p_mpu9250->roll += 360;
    while(p_mpu9250->pitch < 0) p_mpu9250->pitch += 360;
*/
}





void mpu9250_mpu_sampling(mpu9250_t *p_mpu9250) {
	int status = -1;
	int new_data = 0;
	short gyro[3], accel_short[3];
	            unsigned char sensors, more = 0;
	            unsigned long sensor_timestamp;
	do {
		if(mpu_read_fifo(gyro, accel_short, &sensor_timestamp, &sensors, &more)) {
			// Little safety feature to report if it's impossible to read fifo!
			if(p_mpu9250->fault_handler != NULL) {
				p_mpu9250->fault_handler(p_mpu9250, MPU9250_FAULT_UNABLE_TO_READ_FIFO);
			}
			break;
		}
		ComplementaryFilter(p_mpu9250, sensors, accel_short, gyro, 1.0f/(float)DEFAULT_MPU_HZ);
		p_mpu9250->num_samples++;
		if(p_mpu9250->euler_handler != NULL) {
			p_mpu9250->euler_handler(p_mpu9250, p_mpu9250->pitch, p_mpu9250->roll, p_mpu9250->yaw);
		}
		// Do something with the data
	}while( more >0 );
}


void mpu9250_sampling(mpu9250_t *p_mpu9250) {
	unsigned long sensor_timestamp;
	short gyro[3], accel_short[3], sensors;
	unsigned char more;
	long accel[3], quat[4], temperature;
	short compass_short[3];
	long compass[3];
	bool new_data = 0;

	if(!p_mpu9250) return;

	do {
		// GET STUFF FROM FIFO AND BUILD ACCORDINGLY
		if(dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more) == 0) {


			if(sensors & INV_XYZ_GYRO) {
				if(!inv_build_gyro(gyro, sensor_timestamp)) new_data = 1;
				if(mpu_get_temperature(&temperature, &sensor_timestamp) == 0){
					if(!inv_build_temp(temperature, sensor_timestamp)) new_data = 1;
				}
			}

			if(sensors & INV_XYZ_ACCEL) {
				// GOt the fifo, time to convert
				accel[0] = (long)accel_short[0];
				accel[1] = (long)accel_short[1];
				accel[2] = (long)accel_short[2];
				if(!inv_build_accel(accel, 0, sensor_timestamp)) new_data = 1;
			}

			if(sensors & INV_WXYZ_QUAT) {
				// Pass to higher level processing
				if(!inv_build_quat(quat, 0, sensor_timestamp)) new_data = 1;
			}
		}

		if (!mpu_get_compass_reg(compass_short, &sensor_timestamp)) {
			compass[0] = (long)compass_short[0];
			compass[1] = (long)compass_short[1];
			compass[2] = (long)compass_short[2];
			/* NOTE: If using a third-party compass calibration library,
			 * pass in the compass data in uT * 2^16 and set the second
			 * parameter to INV_CALIBRATED | acc, where acc is the
			 * accuracy from 0 to 3.
			 */
			if(!inv_build_compass(compass, 0, sensor_timestamp)) new_data = 1;
		}


		if((new_data) && (inv_execute_on_data() == INV_SUCCESS)){
			mpu9250_sensor_fusion(p_mpu9250);
		}
	}while(more);
}



/**
 * @return 0 on success
 */
int mpu9250_init(mpu9250_t *p_mpu9250, mpu9250_conf_t *p_mpu9250_conf) {
	inv_error_t result = 0;
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;
    unsigned short compass_fsr;

    // Set the struct values here
    memset(p_mpu9250, 0, sizeof(p_mpu9250));
    p_mpu9250->empl_data_handler 		= p_mpu9250_conf->empl_data_handler;
    p_mpu9250->euler_handler 			= p_mpu9250_conf->euler_handler;
    p_mpu9250->fault_handler 			= p_mpu9250_conf->fault_handler;

	result += mpu_init(NULL);
//	result += inv_init_mpl();

/*
	for(int i=0; i<10; i++) {
		if(!mpu9250_self_test(p_mpu9250)) {
			break;
		}
		sd_app_evt_wait();	//basically idle
	}
*/

//	result += inv_enable_quaternion();
//	result += inv_enable_9x_sensor_fusion();
//	result += inv_9x_fusion_use_timestamps(1);
//	result += inv_9x_fusion_enable_jitter_reduction(true);
//	result += inv_enable_heading_from_gyro();
//	result += inv_enable_fast_nomot();

	// Calibration stuff
	/*
	result += inv_enable_gyro_tc();
	result += dmp_enable_gyro_cal(1);
	result += inv_enable_in_use_auto_calibration();
	result += inv_enable_vector_compass_cal();
	result += inv_enable_magnetic_disturbance();
*/
//	result += inv_enable_eMPL_outputs();
//	result += inv_start_mpl();

	result += mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL );
    result += mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    result += mpu_set_sample_rate(DEFAULT_MPU_HZ);

    result += mpu_get_accel_sens( &p_mpu9250->accel_sens );
    result += mpu_get_gyro_sens(&p_mpu9250->gyro_sens);

    result += mpu_get_sample_rate(&gyro_rate);
    result += mpu_get_gyro_fsr(&gyro_fsr);
    result += mpu_get_accel_fsr(&accel_fsr);

    /*
    inv_set_gyro_sample_rate(1000000L / gyro_rate);
    inv_set_accel_sample_rate(1000000L / gyro_rate);
    inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);

    inv_set_gyro_orientation_and_scale(inv_orientation_matrix_to_scalar(gyro_pdata.orientation), (long)gyro_fsr<<15);
    inv_set_accel_orientation_and_scale(inv_orientation_matrix_to_scalar(gyro_pdata.orientation), (long)accel_fsr<<15);
    inv_set_compass_orientation_and_scale(inv_orientation_matrix_to_scalar(compass_pdata.orientation), (long)compass_fsr<<15);
*/

/*
    result += dmp_load_motion_driver_firmware();
	dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
    uint32_t features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL;
	result += dmp_enable_feature(features);
	result += dmp_set_fifo_rate(DEFAULT_MPU_HZ);
	result += mpu_set_dmp_state(1);
*/


	return result;
}
