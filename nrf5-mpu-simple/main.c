 /* 
  * This example is not extensively tested and only 
  * meant as a simple explanation and for inspiration. 
  * NO WARRANTY of ANY KIND is provided. 
  */

#include <stdio.h>
#include "boards.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_mpu.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


void mpu_init(void)
{
    ret_code_t ret_code;
    // Initiate MPU driver
    ret_code = app_mpu_init();
    APP_ERROR_CHECK(ret_code); // Check for errors in return value
    
    // Setup and configure the MPU with intial values
    app_mpu_config_t p_mpu_config = MPU_DEFAULT_CONFIG(); 	// Load default config values
    p_mpu_config.smplrt_div = 7;   													// Change gyro sample rate. 7 gives a sample rate of 1kHz (see section 4.4, MPU Register Map PDF). 
    p_mpu_config.accel_config.afs_sel = AFS_2G; 						// Set accelerometer full scale range to ±2g
    p_mpu_config.gyro_config.fs_sel = GFS_250DPS,						// Set gyroscope full scale range to ±250°/s
	  ret_code = app_mpu_config(&p_mpu_config); 							// Configure the MPU with above values
    APP_ERROR_CHECK(ret_code); 															// Check for errors in return value 
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{    
    uint32_t err_code;
    
    // Initialize.
    log_init();
	  //NRF_LOG_INFO("\033[2J\033[;H"); // Clear screen
    
    mpu_init();
    
    // Start execution.
    NRF_LOG_INFO("MPU Free Fall Interrupt example.");
    
    accel_values_t acc_values;
	  gyro_values_t gyro_values;
    uint32_t sample_number = 0;
	
		double ax_gscale;
		double ay_gscale;
		double az_gscale;
		double gx_degscale;
		double gy_degscale;
		double gz_degscale;
		double acc_LSB = (double) 65535/(2*2);     		//Set accel to ±2g (see section 4.5, MPU Register Map PDF)
	  double gyro_LSB = (double) 65535/(250*2); 		//Set gyro to ±250°/s (see section 4.4, MPU Register Map PDF)
    
    while(1)
    {
        if(NRF_LOG_PROCESS() == false)
        {
            // Read accelerometer sensor values
            err_code = app_mpu_read_accel(&acc_values);
            APP_ERROR_CHECK(err_code);
					  err_code = app_mpu_read_gyro(&gyro_values);
						APP_ERROR_CHECK(err_code);
					 					
						//convert from int16_t to units of g / deg based on LSB
					  ax_gscale = (double) acc_values.x / acc_LSB;
						ay_gscale = (double) acc_values.y / acc_LSB;
						az_gscale = (double) acc_values.z / acc_LSB;
						gx_degscale = (double) gyro_values.x / gyro_LSB;
						gy_degscale = (double) gyro_values.y / gyro_LSB;
						gz_degscale = (double) gyro_values.z / gyro_LSB;
					
            // Clear terminal and print values
					  NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER "," NRF_LOG_FLOAT_MARKER ",", NRF_LOG_FLOAT(ax_gscale), NRF_LOG_FLOAT(ay_gscale));
					  NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER "," NRF_LOG_FLOAT_MARKER ",", NRF_LOG_FLOAT(az_gscale), NRF_LOG_FLOAT(gx_degscale));
					  NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER "," NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(gy_degscale), NRF_LOG_FLOAT(gz_degscale));
					  //NRF_LOG_INFO(NRF_LOG_FLOAT_MARKER "," NRF_LOG_FLOAT_MARKER "," NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(ax_gscale), NRF_LOG_FLOAT(ay_gscale), NRF_LOG_FLOAT(az_gscale));
            //NRF_LOG_INFO(NRF_LOG_FLOAT_MARKER "," NRF_LOG_FLOAT_MARKER "," NRF_LOG_FLOAT_MARKER "," NRF_LOG_FLOAT_MARKER "," NRF_LOG_FLOAT_MARKER "," NRF_LOG_FLOAT_MARKER,
					  //NRF_LOG_FLOAT(ax_gscale), NRF_LOG_FLOAT(ay_gscale), NRF_LOG_FLOAT(az_gscale), NRF_LOG_FLOAT(gx_degscale), NRF_LOG_FLOAT(gy_degscale), NRF_LOG_FLOAT(gz_degscale));
            nrf_delay_ms(500);
        }
    }
}

/** @} */

