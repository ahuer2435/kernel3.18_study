typedef struct hardware_info_struct{
char  *nand_device_name;
char  *hq_lcm_name; 
char *hq_tpd_name;
int	tpd_fw_version;
int tpd_sensor_id;
char subCameraName[32];
char mainCameraName[32];
char *alsps_name;
char *gsenor_name;
char *msensor_name;
char *gyro_name;
char *cpu_name;
} hardware_info_struct;
