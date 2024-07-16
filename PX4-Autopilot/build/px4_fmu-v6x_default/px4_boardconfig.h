#define CONFIG_PLATFORM_NUTTX 1
#define CONFIG_BOARD_PLATFORM "nuttx"
#define CONFIG_BOARD_TOOLCHAIN "arm-none-eabi"
#define CONFIG_BOARD_ARCHITECTURE "cortex-m7"
#define CONFIG_BOARD_ROMFSROOT "px4fmu_common"
#define CONFIG_BOARD_IO "px4_io-v2_default"
#define CONFIG_BOARD_LINKER_PREFIX ""
#define CONFIG_BOARD_COMPILE_DEFINITIONS ""
#define CONFIG_BOARD_ETHERNET 1
#define CONFIG_BOARD_SERIAL_URT6 ""
#define CONFIG_BOARD_SERIAL_GPS1 "/dev/ttyS0"
#define CONFIG_BOARD_SERIAL_GPS2 "/dev/ttyS7"
#define CONFIG_BOARD_SERIAL_GPS3 ""
#define CONFIG_BOARD_SERIAL_GPS4 ""
#define CONFIG_BOARD_SERIAL_GPS5 ""
#define CONFIG_BOARD_SERIAL_TEL1 "/dev/ttyS6"
#define CONFIG_BOARD_SERIAL_TEL2 "/dev/ttyS4"
#define CONFIG_BOARD_SERIAL_TEL3 "/dev/ttyS1"
#define CONFIG_BOARD_SERIAL_TEL4 ""
#define CONFIG_BOARD_SERIAL_TEL5 ""
#define CONFIG_BOARD_SERIAL_RC ""
#define CONFIG_BOARD_SERIAL_WIFI ""
#define CONFIG_BOARD_SERIAL_EXT2 "/dev/ttyS3"
#define CONFIG_DRIVERS_ADC_ADS1115 1
#define CONFIG_DRIVERS_ADC_BOARD_ADC 1
#define CONFIG_DRIVERS_BAROMETER_BMP388 1
#define CONFIG_DRIVERS_BAROMETER_INVENSENSE_ICP201XX 1
#define CONFIG_DRIVERS_BAROMETER_MS5611 1
#define CONFIG_DRIVERS_BATT_SMBUS 1
#define CONFIG_COMMON_DIFFERENTIAL_PRESSURE 1
#define CONFIG_DRIVERS_DIFFERENTIAL_PRESSURE_MS4525DO 1
#define CONFIG_DRIVERS_DIFFERENTIAL_PRESSURE_MS5525DSO 1
#define CONFIG_DRIVERS_DIFFERENTIAL_PRESSURE_SDP3X 1
#define CONFIG_COMMON_DISTANCE_SENSOR 1
#define CONFIG_DRIVERS_DISTANCE_SENSOR_CM8JL65 1
#define CONFIG_DRIVERS_DISTANCE_SENSOR_GY_US42 1
#define CONFIG_DRIVERS_DISTANCE_SENSOR_LEDDAR_ONE 1
#define CONFIG_DRIVERS_DISTANCE_SENSOR_LIGHTWARE_LASER_I2C 1
#define CONFIG_DRIVERS_DISTANCE_SENSOR_LIGHTWARE_LASER_SERIAL 1
#define CONFIG_DRIVERS_DISTANCE_SENSOR_LL40LS 1
#define CONFIG_DRIVERS_DISTANCE_SENSOR_LL40LS_PWM 1
#define CONFIG_DRIVERS_DISTANCE_SENSOR_MAPPYDOT 1
#define CONFIG_DRIVERS_DISTANCE_SENSOR_MB12XX 1
#define CONFIG_DRIVERS_DISTANCE_SENSOR_PGA460 1
#define CONFIG_DRIVERS_DISTANCE_SENSOR_SRF02 1
#define CONFIG_DRIVERS_DISTANCE_SENSOR_TERARANGER 1
#define CONFIG_DRIVERS_DISTANCE_SENSOR_TF02PRO 1
#define CONFIG_DRIVERS_DISTANCE_SENSOR_TFMINI 1
#define CONFIG_DRIVERS_DISTANCE_SENSOR_ULANDING_RADAR 1
#define CONFIG_DRIVERS_DISTANCE_SENSOR_VL53L0X 1
#define CONFIG_DRIVERS_DISTANCE_SENSOR_VL53L1X 1
#define CONFIG_DRIVERS_DSHOT 1
#define CONFIG_DRIVERS_GPS 1
#define CONFIG_DRIVERS_HEATER 1
#define CONFIG_DRIVERS_IMU_ANALOG_DEVICES_ADIS16470 1
#define CONFIG_DRIVERS_IMU_BOSCH_BMI088 1
#define CONFIG_DRIVERS_IMU_INVENSENSE_ICM20602 1
#define CONFIG_DRIVERS_IMU_INVENSENSE_ICM20649 1
#define CONFIG_DRIVERS_IMU_INVENSENSE_ICM20948 1
#define CONFIG_DRIVERS_IMU_INVENSENSE_ICM42670P 1
#define CONFIG_DRIVERS_IMU_INVENSENSE_ICM42688P 1
#define CONFIG_DRIVERS_IMU_INVENSENSE_ICM45686 1
#define CONFIG_DRIVERS_IMU_INVENSENSE_IIM42652 1
#define CONFIG_COMMON_INS 1
#define CONFIG_DRIVERS_INS_VECTORNAV 1
#define CONFIG_COMMON_LIGHT 1
#define CONFIG_DRIVERS_LIGHTS_RGBLED 1
#define CONFIG_DRIVERS_LIGHTS_RGBLED_IS31FL3195 1
#define CONFIG_DRIVERS_LIGHTS_RGBLED_LP5562 1
#define CONFIG_DRIVERS_LIGHTS_RGBLED_NCP5623C 1
#define CONFIG_COMMON_MAGNETOMETER 1
#define CONFIG_DRIVERS_MAGNETOMETER_AKM_AK09916 1
#define CONFIG_DRIVERS_MAGNETOMETER_AKM_AK8963 1
#define CONFIG_DRIVERS_MAGNETOMETER_BOSCH_BMM150 1
#define CONFIG_DRIVERS_MAGNETOMETER_HMC5883 1
#define CONFIG_DRIVERS_MAGNETOMETER_ISENTEK_IST8308 1
#define CONFIG_DRIVERS_MAGNETOMETER_ISENTEK_IST8310 1
#define CONFIG_DRIVERS_MAGNETOMETER_LIS2MDL 1
#define CONFIG_DRIVERS_MAGNETOMETER_LIS3MDL 1
#define CONFIG_DRIVERS_MAGNETOMETER_LSM303AGR 1
#define CONFIG_DRIVERS_MAGNETOMETER_QMC5883L 1
#define CONFIG_DRIVERS_MAGNETOMETER_RM3100 1
#define CONFIG_DRIVERS_MAGNETOMETER_VTRANTECH_VCM1193L 1
#define CONFIG_COMMON_OSD 1
#define CONFIG_DRIVERS_OSD_ATXXXX 1
#define CONFIG_DRIVERS_OSD_MSP_OSD 1
#define CONFIG_DRIVERS_POWER_MONITOR_INA226 1
#define CONFIG_DRIVERS_POWER_MONITOR_INA228 1
#define CONFIG_DRIVERS_POWER_MONITOR_INA238 1
#define CONFIG_DRIVERS_PWM_OUT 1
#define CONFIG_DRIVERS_PX4IO 1
#define CONFIG_COMMON_RC 1
#define CONFIG_DRIVERS_RC_CRSF_RC 1
#define CONFIG_DRIVERS_RC_INPUT 1
#define CONFIG_DRIVERS_RPM_PCF8583 1
#define CONFIG_DRIVERS_SAFETY_BUTTON 1
#define CONFIG_DRIVERS_TONE_ALARM 1
#define CONFIG_DRIVERS_UAVCAN 1
#define CONFIG_BOARD_UAVCAN_INTERFACES 2
#define CONFIG_BOARD_UAVCAN_PERIPHERALS ""
#define CONFIG_BOARD_UAVCAN_TIMER_OVERRIDE 2
#define CONFIG_MODULES_AIRSPEED_SELECTOR 1
#define CONFIG_MODULES_BATTERY_STATUS 1
#define CONFIG_MODULES_COMMANDER 1
#define CONFIG_MODULES_CONTROL_ALLOCATOR 1
#define CONFIG_MODULES_DATAMAN 1
#define CONFIG_MODULES_EKF2 1
#define CONFIG_EKF2_MULTI_INSTANCE 1
#define CONFIG_EKF2_AIRSPEED 1
#define CONFIG_EKF2_AUXVEL 1
#define CONFIG_EKF2_BARO_COMPENSATION 1
#define CONFIG_EKF2_DRAG_FUSION 1
#define CONFIG_EKF2_EXTERNAL_VISION 1
#define CONFIG_EKF2_GNSS_YAW 1
#define CONFIG_EKF2_OPTICAL_FLOW 1
#define CONFIG_EKF2_RANGE_FINDER 1
#define CONFIG_EKF2_SIDESLIP 1
#define CONFIG_MODULES_ESC_BATTERY 1
#define CONFIG_MODULES_EVENTS 1
#define CONFIG_MODULES_FLIGHT_MODE_MANAGER 1
#define CONFIG_MODULES_FW_ATT_CONTROL 1
#define CONFIG_MODULES_FW_AUTOTUNE_ATTITUDE_CONTROL 1
#define CONFIG_MODULES_FW_POS_CONTROL 1
#define CONFIG_MODULES_FW_RATE_CONTROL 1
#define CONFIG_MODULES_GIMBAL 1
#define CONFIG_MODULES_GYRO_CALIBRATION 1
#define CONFIG_MODULES_LAND_DETECTOR 1
#define CONFIG_MODULES_LANDING_TARGET_ESTIMATOR 1
#define CONFIG_MODULES_LOAD_MON 1
#define CONFIG_MODULES_LOGGER 1
#define CONFIG_MODULES_MAG_BIAS_ESTIMATOR 1
#define CONFIG_MODULES_MANUAL_CONTROL 1
#define CONFIG_MODULES_MAVLINK 1
#define CONFIG_MAVLINK_DIALECT "common"
#define CONFIG_MAVLINK_UAVCAN_PARAMETERS 1
#define CONFIG_MODULES_MC_ATT_CONTROL 1
#define CONFIG_MODULES_MC_AUTOTUNE_ATTITUDE_CONTROL 1
#define CONFIG_MODULES_MC_HOVER_THRUST_ESTIMATOR 1
#define CONFIG_MODULES_MC_POS_CONTROL 1
#define CONFIG_MODULES_MC_RATE_CONTROL 1
#define CONFIG_MODULES_MC_RESILIENT_ESTIMATOR 1
#define CONFIG_MODULES_NAVIGATOR 1
#define CONFIG_MODULES_RC_UPDATE 1
#define CONFIG_MODULES_SENSORS 1
#define CONFIG_SENSORS_VEHICLE_AIRSPEED 1
#define CONFIG_SENSORS_VEHICLE_AIR_DATA 1
#define CONFIG_SENSORS_VEHICLE_ANGULAR_VELOCITY 1
#define CONFIG_SENSORS_VEHICLE_ACCELERATION 1
#define CONFIG_SENSORS_VEHICLE_GPS_POSITION 1
#define CONFIG_SENSORS_VEHICLE_MAGNETOMETER 1
#define CONFIG_SENSORS_VEHICLE_OPTICAL_FLOW 1
#define CONFIG_MODULES_TEMPERATURE_COMPENSATION 1
#define CONFIG_MODULES_UXRCE_DDS_CLIENT 1
#define CONFIG_MODULES_VTOL_ATT_CONTROL 1
#define CONFIG_SYSTEMCMDS_ACTUATOR_TEST 1
#define CONFIG_SYSTEMCMDS_BSONDUMP 1
#define CONFIG_SYSTEMCMDS_DMESG 1
#define CONFIG_SYSTEMCMDS_GPIO 1
#define CONFIG_SYSTEMCMDS_HARDFAULT_LOG 1
#define CONFIG_SYSTEMCMDS_I2CDETECT 1
#define CONFIG_SYSTEMCMDS_LED_CONTROL 1
#define CONFIG_SYSTEMCMDS_MFT 1
#define CONFIG_SYSTEMCMDS_MTD 1
#define CONFIG_SYSTEMCMDS_NETMAN 1
#define CONFIG_SYSTEMCMDS_NSHTERM 1
#define CONFIG_SYSTEMCMDS_PARAM 1
#define CONFIG_SYSTEMCMDS_PERF 1
#define CONFIG_SYSTEMCMDS_REBOOT 1
#define CONFIG_SYSTEMCMDS_SYSTEM_TIME 1
#define CONFIG_SYSTEMCMDS_TOP 1
#define CONFIG_SYSTEMCMDS_TOPIC_LISTENER 1
#define CONFIG_SYSTEMCMDS_TUNE_CONTROL 1
#define CONFIG_SYSTEMCMDS_UORB 1
#define CONFIG_SYSTEMCMDS_VER 1
#define CONFIG_SYSTEMCMDS_WORK_QUEUE 1