#ifndef _MPU_9250_
#define _MPU_9250_

#include <stdint.h>
#include <stdbool.h>


#include "invensense_adv.h"
#include "mltypes.h"
#include "data_builder.h"
#include "inv_mpu.h"
#include "mpl.h"
#include "ml_math_func.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "eMPL_outputs.h"


typedef enum {
	MPU9250_EMPL_DATA_ACCEL,
	MPU9250_EMPL_DATA_EULER,
	MPU9250_EMPL_DATA_QUAT,
}mpu9250_empl_data_type_t;


#define COMPASS_ENABLED 1
#define DEFAULT_MPU_HZ  (100)
#define COMPASS_READ_MS (100)

struct platform_data_s {
    signed char orientation[9];
};

typedef struct mpu9250_s mpu9250_t ;
typedef struct mpu9250_conf_s mpu9250_conf_t;

typedef void (*mpu9250_empl_data_handler_t)(mpu9250_t* p_mpu9250, mpu9250_empl_data_type_t type, long *data, unsigned long timestamp);

typedef void (*mpu9250_euler_handler_t)(mpu9250_t* p_mpu9250, float pitch, float roll, float yaw);

struct mpu9250_s{
	unsigned short					accel_sens;
	float 							gyro_sens;
	float							pitch;
	float 							roll;
	float 							yaw;
	mpu9250_empl_data_handler_t 	empl_data_handler;
	mpu9250_euler_handler_t			euler_handler;
	uint32_t 						num_samples;
};

struct mpu9250_conf_s{
	mpu9250_empl_data_handler_t 	empl_data_handler;
	mpu9250_euler_handler_t			euler_handler;
};


int mpu9250_init(mpu9250_t *p_mpu9250, mpu9250_conf_t *p_mpu9250_conf);
void mpu9250_sensor_fusion(mpu9250_t* p_mpu9250);
void mpu9250_sampling(mpu9250_t *p_mpu9250);
void mpu9250_mpu_sampling(mpu9250_t *p_mpu9250);
int mpu9250_set_sampling_rate(mpu9250_t *p_mpu9250, int rate_hz);
void mpu9250_int_handler(mpu9250_t *p_mpu9250);
int mpu9250_self_test(mpu9250_t *p_mpu9250);

#endif // _MPU_9250_
