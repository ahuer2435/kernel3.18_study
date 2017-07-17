#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <linux/xlog.h>
//#include <mach/mt_typedefs.h>
#include <linux/proc_fs.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <mach/upmu_sw.h>
#include <mach/mt_pmic.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

//#include "cust_pmic.h"
#define COULOMETER_DEBUG_LOG

#define CM_SLAVE_ADDR_WRITE   0xBB
#define CM_SLAVE_ADDR_Read    0xAA

//static struct i2c_board_info __initdata i2c_dev={ I2C_BOARD_INFO("couloMeter", 0x55)};
static const struct i2c_device_id cm_i2c_id[] = {{"couloMeter", 0},{}};
static struct i2c_client *cm_iic_client = NULL;

static int cm_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int cm_i2c_remove(struct i2c_client *client);
static int cm_i2c_suspend(struct i2c_client *client,pm_message_t mesg);
static int cm_i2c_resume(struct i2c_client *client);

#ifdef CONFIG_OF
static const struct of_device_id cm_i2c_of_match[] = {
	{.compatible = "mediatek,coulometer",},
	{},
};

//MODULE_DEVICE_TABLE(of, cm_i2c_of_match);
#endif

static struct i2c_driver CM_i2c_driver = {
	.driver = {
		.name    = "couloMeter",
#ifdef CONFIG_OF
		.of_match_table    = cm_i2c_of_match,
#endif
	},
	.probe       = cm_i2c_probe,
	.id_table = cm_i2c_id,
	.remove      = cm_i2c_remove,
	.suspend     = cm_i2c_suspend,
	.resume      = cm_i2c_resume,

};

//add by zym,this flag indicated powersupply is couloMeter or power,
//value 1:couloMeter
//value 0:Power
static int g_power_is_couloMeter = 1;
extern unsigned int PMIC_IMM_GetOneChannelValue(pmic_adc_ch_list_enum dwChannel, int deCount, int trimd);
//function
/**********************************************************
 *
 *   [Global Variable] 
 *
 *********************************************************/

static DEFINE_MUTEX(coulometer_mutex);
static DEFINE_MUTEX(cm_i2c_access);
/**********************************************************
 *
 *   [I2C Function For Read/Write sn65dsi8x] 
 *
 *********************************************************/
static int cm_read_byte(int cmd, int *returnData)
{
	char cmd_buf[1]={0};
	char readData = 0;
	int ret=0;
	int mutex_ret = -1;


	if(NULL == returnData)
	{
		printk("[cm_read_byte] null point exception!!!!!!!\n");
		return -1;
	}
	mutex_lock(&cm_i2c_access); 
	cm_iic_client->ext_flag=((cm_iic_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG | I2C_RS_FLAG;

	cmd_buf[0] = cmd;
#if defined(COULOMETER_DEBUG_LOG)
	printk("[cm_read_byte4] cmd=0x%x\n",cmd);
#endif
	ret = i2c_master_send(cm_iic_client, &cmd_buf[0], (1<<8 | 1));
	if (ret < 0) 
	{    
		cm_iic_client->ext_flag=0;
		mutex_unlock(&cm_i2c_access);
		printk("[cm_read_byte] iic read failed(%d)\n",ret);

		return ret;
	}

	readData = cmd_buf[0];
	*returnData = readData;
	//accquire status of mutex
	mutex_ret = mutex_is_locked(&cm_i2c_access);
#if defined(COULOMETER_DEBUG_LOG)
	printk("[cm_read_byte] mutex_status = %d,*returnData=0x%x\n",mutex_ret,*returnData);
#endif

	cm_iic_client->ext_flag=0;
	mutex_unlock(&cm_i2c_access);    

	return ret;
}
/**********************************************************
 *
 *   [coulometer function For Read/Write coulometer] 
 *
 *********************************************************/
/**********************************************************
 *[name]      :cm_get_Temperature
 *[return]    :battery temperature in units 0.1k 
 *[desciption]:ret*0.1 -273.5
 *********************************************************/
int cm_get_Temperature(void)
{
	int ret = -1,ret_a = 0,ret_b = 0;
	int mutex_ret = -1;
	int count = 0;

	if( 0 == g_power_is_couloMeter )
		return 25;

	mutex_lock(&coulometer_mutex);
	mutex_ret = mutex_is_locked(&coulometer_mutex);
	printk("current mutex status mutex_ret=%d\n",mutex_ret);

	while ((ret < 0) && (count < 3))
	{
		//after firstRegister is repeatStart flag,so firstRegister fail,secondRegister must fail
		ret = cm_read_byte(0x06,&ret_a);
		ret = cm_read_byte(0x07,&ret_b);
		if( ret < 0)
		{
			printk("couloMeter fatal error,+^-^+ ret=%d\n",ret);
			count ++;
			mdelay(10);
		}
	}
	ret = (ret_b << 8) | ret_a;
	//switch k to T
	ret = (ret * 1 - 2735)/10;
	mutex_unlock(&coulometer_mutex);
#if defined(COULOMETER_DEBUG_LOG)
	printk("cm_get_Temperature ret_a=0x%x,ret_b=0x%x,ret=%d\n",ret_a,ret_b,ret);
#endif
	return ret;	
}

/**********************************************************
 *[name]      :cm_get_Voltage
 *[return]    :unsigned integer value 
 *[desciption]:the measured cell-pack voltage in mv with a range of 0 to 6000mv
 *********************************************************/
int cm_get_Voltage(void)
{
	int ret = -1,ret_a = 0,ret_b = 0;
	int mutex_ret = -1;
	int count = 0;

	if( 0 == g_power_is_couloMeter )
		return 4000;

	mutex_lock(&coulometer_mutex);
	mutex_ret = mutex_is_locked(&coulometer_mutex);
	printk("current mutex status mutex_ret=%d\n",mutex_ret);

	while ((ret < 0) && (count < 3))
	{
		//after firstRegister is repeatStart flag,so firstRegister fail,secondRegister must fail
		ret = cm_read_byte(0x08,&ret_a);
		ret = cm_read_byte(0x09,&ret_b);
		if( ret < 0)
		{
			printk("couloMeter fatal error,+^-^+ ret=%d\n",ret);
			count ++;
			mdelay(10);
		}
	}
	ret = (ret_b << 8) | ret_a;
	mutex_unlock(&coulometer_mutex);
#if defined(COULOMETER_DEBUG_LOG)
	printk("cm_get_Voltage ret_a=0x%x,ret_b=0x%x,ret=%d\n",ret_a,ret_b,ret);
#endif
	return ret;	
}

/**********************************************************
 *[name]      :cm_get_NominalAvailableCapacity
 *[return]    :battery capacity remaining
 *[desciption]:uncompenstated battery capacity remaining,units are mAh
 *********************************************************/
int cm_get_NominalAvailableCapacity(void)
{
	int ret = -1,ret_a = 0,ret_b = 0;
	int mutex_ret = -1;
	int count = 0;

	mutex_lock(&coulometer_mutex);
	mutex_ret = mutex_is_locked(&coulometer_mutex);
	printk("current mutex status mutex_ret=%d\n",mutex_ret);

	while ((ret < 0) && (count < 3))
	{
		//after firstRegister is repeatStart flag,so firstRegister fail,secondRegister must fail
		ret = cm_read_byte(0x0C,&ret_a);
		ret = cm_read_byte(0x0D,&ret_b);
		if( ret < 0)
		{
			printk("couloMeter fatal error,+^-^+ ret=%d\n",ret);
			count ++;
			mdelay(10);
		}
	}
	ret = (ret_b << 8) | ret_a;
	mutex_unlock(&coulometer_mutex);
#if defined(COULOMETER_DEBUG_LOG)
	printk("cm_get_NominalAvailableCapacity ret_a=0x%x,ret_b=0x%x,ret=%d\n",ret_a,ret_b,ret);
#endif
	return ret;
}

/**********************************************************
 *[name]      :cm_get_RemainingCapacity
 *[return]    :compensated battery capacity remaining
 *[desciption]:compenstated battery capacity remaining,units are mAh
 *********************************************************/
int cm_get_RemainingCapacity(void)
{
	int ret = -1,ret_a = 0,ret_b = 0;
	int mutex_ret = -1;
	int count = 0;

	mutex_lock(&coulometer_mutex);
	mutex_ret = mutex_is_locked(&coulometer_mutex);
	printk("current mutex status mutex_ret=%d\n",mutex_ret);

	while ((ret < 0) && (count < 3))
	{
		//after firstRegister is repeatStart flag,so firstRegister fail,secondRegister must fail
		ret = cm_read_byte(0x10,&ret_a);
		ret = cm_read_byte(0x11,&ret_b);
		if( ret < 0)
		{
			printk("couloMeter fatal error,+^-^+ ret=%d\n",ret);
			count ++;
			mdelay(10);
		}
	}
	ret = (ret_b << 8) | ret_a;
	mutex_unlock(&coulometer_mutex);
#if defined(COULOMETER_DEBUG_LOG)
	printk("cm_get_RemainingCapacity ret_a=0x%x,ret_b=0x%x,ret=%d\n",ret_a,ret_b,ret);
#endif
	return ret;	
}

/**********************************************************
 *[name]      :cm_get_FullChargeCapacity
 *[return]    :compensated capacity
 *[desciption]:returns the compensated capacity of the battery when fully charged,units are mAh
 *********************************************************/
int cm_get_FullChargeCapacity(void)
{
	int ret = -1,ret_a = 0,ret_b = 0;
	int mutex_ret = -1;
	int count = 0;

	mutex_lock(&coulometer_mutex);
	mutex_ret = mutex_is_locked(&coulometer_mutex);
	printk("current mutex status mutex_ret=%d\n",mutex_ret);

	while ((ret < 0) && (count < 3))
	{
		//after firstRegister is repeatStart flag,so firstRegister fail,secondRegister must fail
		ret = cm_read_byte(0x12,&ret_a);
		ret = cm_read_byte(0x13,&ret_b);
		if( ret < 0)
		{
			printk("couloMeter fatal error,+^-^+ ret=%d\n",ret);
			count ++;
			mdelay(10);
		}
	}
	ret = (ret_b << 8) | ret_a;
	mutex_unlock(&coulometer_mutex);
#if defined(COULOMETER_DEBUG_LOG)
	printk("cm_get_FullChargeCapacity ret_a=0x%x,ret_b=0x%x,ret=%d\n",ret_a,ret_b,ret);
#endif
	return ret;	
}

/**********************************************************
 *[name]      :cm_get_AverageCurrent
 *[return]    :average current flow through the sense resistor
 *[desciption]:units are mAh
 *********************************************************/
int cm_get_AverageCurrent(void)
{
	int ret = -1,ret_a = 0,ret_b = 0;
	int mutex_ret = -1;
	int count = 0;

	if( 0 == g_power_is_couloMeter )
		return 1900;

	mutex_lock(&coulometer_mutex);
	mutex_ret = mutex_is_locked(&coulometer_mutex);
	printk("current mutex status mutex_ret=%d\n",mutex_ret);

	while ((ret < 0) && (count < 3))
	{
		//after firstRegister is repeatStart flag,so firstRegister fail,secondRegister must fail
		ret = cm_read_byte(0x14,&ret_a);
		ret = cm_read_byte(0x15,&ret_b);
		if( ret < 0)
		{
			printk("couloMeter fatal error,+^-^+ ret=%d\n",ret);
			count ++;
			mdelay(10);
		}
	}
	ret = (ret_b << 8) | ret_a;

	if(ret > 32767)
		ret = ret - 65536;
	mutex_unlock(&coulometer_mutex);
#if defined(COULOMETER_DEBUG_LOG)
	printk("cm_get_AverageCurrent ret_a=0x%x,ret_b=0x%x,ret=%d\n",ret_a,ret_b,ret);
#endif
	return ret;	
}

/**********************************************************
 *[name]      :cm_get_AverageTimeToEmpty
 *[return]    :unsigned integer value of the predicted remaining battery life
 *[desciption]:a value of 65535 indicates battery is not being discharged,units are minutes
 *********************************************************/
int cm_get_AverageTimeToEmpty(void)
{
	int ret = -1,ret_a = 0,ret_b = 0;
	int mutex_ret = -1;
	int count = 0;

	mutex_lock(&coulometer_mutex);
	mutex_ret = mutex_is_locked(&coulometer_mutex);
	printk("current mutex status mutex_ret=%d\n",mutex_ret);

	while ((ret < 0) && (count < 3))
	{
		//after firstRegister is repeatStart flag,so firstRegister fail,secondRegister must fail
		ret = cm_read_byte(0x16,&ret_a);
		ret = cm_read_byte(0x17,&ret_b);
		if( ret < 0)
		{
			printk("couloMeter fatal error,+^-^+ ret=%d\n",ret);
			count ++;
			mdelay(10);
		}
	}
	ret = (ret_b << 8) | ret_a;
	mutex_unlock(&coulometer_mutex);
#if defined(COULOMETER_DEBUG_LOG)
	printk("cm_get_AverageTimeToEmpty ret_a=0x%x,ret_b=0x%x,ret=%d\n",ret_a,ret_b,ret);
#endif
	return ret;	
}

/**********************************************************
 *[name]      :cm_get_AverageTimeToFull
 *[return]    :unsigned integer value of predicated remaining time
 *[desciption]:remaining time until the battery reaches full charge,units are minutes
 *********************************************************/
int cm_get_AverageTimeToFull(void)
{
	int ret = -1,ret_a = 0,ret_b = 0;
	int mutex_ret = -1;
	int count = 0;

	mutex_lock(&coulometer_mutex);
	mutex_ret = mutex_is_locked(&coulometer_mutex);
	printk("current mutex status mutex_ret=%d\n",mutex_ret);

	while ((ret < 0) && (count < 3))
	{
		//after firstRegister is repeatStart flag,so firstRegister fail,secondRegister must fail
		ret = cm_read_byte(0x18,&ret_a);
		ret = cm_read_byte(0x19,&ret_b);
		if( ret < 0)
		{
			printk("couloMeter fatal error,+^-^+ ret=%d\n",ret);
			count ++;
			mdelay(10);
		}
	}
	ret = (ret_b << 8) | ret_a;
	mutex_unlock(&coulometer_mutex);
#if defined(COULOMETER_DEBUG_LOG)
	printk("cm_get_AverageTimeToFull ret_a=0x%x,ret_b=0x%x,ret=%d\n",ret_a,ret_b,ret);
#endif
	return ret;	
}

/**********************************************************
 *[name]      :cm_get_InternalTemperature
 *[return]    :internal temperature
 *[desciption]:
 *********************************************************/
int cm_get_InternalTemperature(void)
{
	int ret = -1,ret_a = 0,ret_b = 0;
	int mutex_ret = -1;
	int count = 0;
	
	if( 0 == g_power_is_couloMeter )
		return 25;

	mutex_lock(&coulometer_mutex);
	mutex_ret = mutex_is_locked(&coulometer_mutex);
	printk("current mutex status mutex_ret=%d\n",mutex_ret);

	while ((ret < 0) && (count < 3))
	{
		//after firstRegister is repeatStart flag,so firstRegister fail,secondRegister must fail
		ret = cm_read_byte(0x28,&ret_a);
		ret = cm_read_byte(0x29,&ret_b);
		if( ret < 0)
		{
			printk("couloMeter fatal error,+^-^+ ret=%d\n",ret);
			count ++;
			mdelay(10);
		}
	}
	ret = (ret_b << 8) | ret_a;
	ret = (ret * 1 -2735)/10;
	mutex_unlock(&coulometer_mutex);
#if defined(COULOMETER_DEBUG_LOG)
	printk("cm_get_InternalTemperature ret_a=0x%x,ret_b=0x%x,ret=%d\n",ret_a,ret_b,ret);
#endif
	return ret;
}

/**********************************************************
 *[name]      :cm_get_StateOfCharge
 *[return]    :intege value of the predicted,0%000%
 *[desciption]:
 *********************************************************/
int cm_get_StateOfCharge(void)
{
	int ret = -1,ret_a = 0,ret_b = 0;
	int mutex_ret = -1;
	int count = 0;

	if( 0 == g_power_is_couloMeter )
		return 50;

	mutex_lock(&coulometer_mutex);
	mutex_ret = mutex_is_locked(&coulometer_mutex);
	printk("current mutex status mutex_ret=%d\n",mutex_ret);

	while ((ret < 0) && (count < 3))
	{
		//after firstRegister is repeatStart flag,so firstRegister fail,secondRegister must fail
		ret = cm_read_byte(0x2C,&ret_a);
		ret = cm_read_byte(0x2D,&ret_b);
		if( ret < 0)
		{
			printk("couloMeter fatal error,+^-^+ ret=%d\n",ret);
			count ++;
			mdelay(10);
		}
	}
	ret = (ret_b << 8) | ret_a;
	mutex_unlock(&coulometer_mutex);
#if defined(COULOMETER_DEBUG_LOG)
	printk("cm_get_StateOfCharge ret_a=0x%x,ret_b=0x%x,ret=%d\n",ret_a,ret_b,ret);
#endif
	return ret;
}

/**********************************************************
 *[name]      :cm_get_StateOfHealth
 *[return]    :unsigned integer value
 *[desciption]:SOH percentage is 0x00 to 0x64,indicating 0 to 100% correspondingly
 *********************************************************/
int cm_get_StateOfHealth(void)
{
	int ret = -1,ret_a = 0,ret_b = 0;
	int mutex_ret = -1;
	int count = 0;

	mutex_lock(&coulometer_mutex);
	mutex_ret = mutex_is_locked(&coulometer_mutex);
	printk("current mutex status mutex_ret=%d\n",mutex_ret);

	while ((ret < 0) && (count < 3))
	{
		//after firstRegister is repeatStart flag,so firstRegister fail,secondRegister must fail
		ret = cm_read_byte(0x2E,&ret_a);
		ret = cm_read_byte(0x2F,&ret_b);
		if( ret < 0)
		{
			printk("couloMeter fatal error,+^-^+ ret=%d\n",ret);
			count ++;
			mdelay(10);
		}
	}
	ret = (ret_b << 8) | ret_a;
	mutex_unlock(&coulometer_mutex);
#if defined(COULOMETER_DEBUG_LOG)
	printk("cm_get_StateOfHealth ret_a=0x%x,ret_b=0x%x,ret=%d\n",ret_a,ret_b,ret);
#endif
	return ret;
}

//export public function
EXPORT_SYMBOL(cm_get_Temperature);
EXPORT_SYMBOL(cm_get_Voltage);
EXPORT_SYMBOL(cm_get_NominalAvailableCapacity);
EXPORT_SYMBOL(cm_get_RemainingCapacity);
EXPORT_SYMBOL(cm_get_FullChargeCapacity);
EXPORT_SYMBOL(cm_get_AverageTimeToEmpty);
EXPORT_SYMBOL(cm_get_AverageCurrent);
EXPORT_SYMBOL(cm_get_AverageTimeToFull);
EXPORT_SYMBOL(cm_get_InternalTemperature);
EXPORT_SYMBOL(cm_get_StateOfCharge);
EXPORT_SYMBOL(cm_get_StateOfHealth);
/**********************************************************
 *
 *   [I2C probe For Read/Write coulometer] 
 *
 *********************************************************/
static int cm_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err;

	printk("[CM_i2c_probe]Start\n");
	if (!(cm_iic_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit;
	}    
	memset(cm_iic_client, 0, sizeof(struct i2c_client));

	cm_iic_client = client;  
	//according to datasheet,set Baud Rate is 100k
	cm_iic_client->timing = 100;

	printk("[CM_i2c_probe]Attached!! \n");
	return 0;
exit:
	return 0;
}

static int cm_i2c_remove(struct i2c_client *client)
{
	printk("[CM_i2c_probe]Start\n");
	return 0;
}

static int cm_i2c_suspend(struct i2c_client *client,pm_message_t mesg)
{
	printk("[cm_i2c_suspend]Start\n");
	return 0;
}

static int cm_i2c_resume(struct i2c_client *client)
{
	printk("[cm_i2c_resume]Start\n");
	return 0;
}


static int cm_probe(struct platform_device *pdev)
{
	mutex_init(&coulometer_mutex);
	mutex_init(&cm_i2c_access);
	return 0;
}

static int cm_remove(struct platform_device *pdev)
{
	printk("[cm_remove] here we start !\n");
	return 0;
}

static int cm_suspend(struct platform_device *pdev,pm_message_t state)
{
	printk("[cm_suspend] here we start !\n");
	return 0;
}

static int cm_resume(struct platform_device *pdev)
{
	printk("[cm_resume] here we start !\n");
	return 0;
}

static ssize_t battery_cmd_read(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
	char *ptr = page;
	int ret_1,ret_2,ret_3,ret_4,ret_5,ret_6,ret_7,ret_8,ret_9,ret_10,ret_11;

	if (*ppos)  // CMD call again
	{
		return 0;
	}

	ret_1 = cm_get_Temperature();
	ret_2 = cm_get_Voltage();
	ret_3 = cm_get_NominalAvailableCapacity();
	ret_4 = cm_get_RemainingCapacity();
	ret_5 = cm_get_FullChargeCapacity();
	ret_6 = cm_get_AverageTimeToEmpty();
	ret_7 = cm_get_AverageCurrent();
	ret_8 = cm_get_AverageTimeToFull();
	ret_9 = cm_get_InternalTemperature();
	ret_10 = cm_get_StateOfCharge();
	ret_11 = cm_get_StateOfHealth();
	printk("gongbo Temp:%d Vol:%d SOC:%d Current:%d\n", ret_1, ret_2, ret_10, ret_7);
	/*
	ptr += sprintf(ptr, "\
			cm_get_Temperature =%d\n\
			cm_get_Voltage=%d(mv)\n\
			cm_get_NominalAvailableCapacity=%d(mAh)\n\
			cm_get_RemainingCapacity=%d(mAh)\n\
			cm_get_FullChargeCapacity=%d(mAh)\n\
			cm_get_AverageTimeToEmpty=%d(min)\n\
			cm_get_AverageCurrent=%d(mAh)\n\
			cm_get_AverageTimeToFull=%d(min)\n\
			cm_get_InternalTemperature=%d(T)\n\
			cm_get_StateOfCharge=%d(%)\n\
			cm_get_StateOfHealth=%d(%)\n",ret_1,ret_2,ret_3,ret_4,ret_5,ret_6,ret_7,ret_8,ret_9,ret_10,ret_11);
			*/

	*ppos += ptr - page;
	return (ptr - page);
}

static ssize_t battery_cmd_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
	//do nothing
	return -EINVAL;
}


static const struct file_operations battery_cmd_proc_fops = { 
	.read  = battery_cmd_read,
	.write = battery_cmd_write,
};

static int mt_couloMeter_probe(struct platform_device *dev)    
{

	//struct proc_dir_entry *entry = NULL;
	struct proc_dir_entry *battery_dir = NULL;

	printk("******** mt_batteryNotify_probe!! ********\n" );

	battery_dir = proc_mkdir("cm_cmd", NULL);
	if (!battery_dir)
	{
		printk("[%s]: mkdir /proc/cm_cmd failed\n", __FUNCTION__);
	}
	else
	{
		proc_create("couloMeter_cmd", S_IRUGO | S_IWUSR, battery_dir, &battery_cmd_proc_fops);
#if defined(COULOMETER_DEBUG_LOG)
		printk("proc_create battery_cmd_proc_fops\n");
#endif
	}
#if defined(COULOMETER_DEBUG_LOG)
	printk( "******** mtk_battery_cmd!! ********\n" );    
#endif		

	return 0;

}

//platform devices
static struct platform_driver g_cm_Driver = {
	.probe        = cm_probe,
	.remove    = cm_remove,
	.suspend  = cm_suspend,
	.resume   = cm_resume,
	.driver        = {
		.name    = "couloMeter",
	}
};
static struct platform_device g_cm_device = {
	.name = "couloMeter",
	.id = -1,
};

//for porc debugfs
struct platform_device MT_couloMeter_device = {
	.name   = "mt_coulometer",
	.id        = -1,
};

static struct platform_driver mt_couloMeter_driver = {
	.probe        = mt_couloMeter_probe,
	.driver       = {
		.name = "mt_coulometer",
	},
};

static int __init cm_i2C_init(void)
{
	int data = 0;
	printk("couloMeter_i2c_init\n");
	//i2c_register_board_info(1, &i2c_dev, 1);
	if(i2c_add_driver(&CM_i2c_driver) == 0)
		printk("Success to register couloMeter i2c driver.\n");
	else
		printk("failed to register couloMeter i2c driver.\n");
	
	if(platform_device_register(&g_cm_device)){
		printk("failed to register couloMeter driver\n");
		return -ENODEV;
	}

	if(platform_driver_register(&g_cm_Driver)){
		printk("Failed to register couloMeter driver\n");
		return -ENODEV;
	}

	if (platform_device_register(&MT_couloMeter_device)) {
		printk("****[mt_couloMeter] Unable to device register\n");
		return -ENODEV;
	}

	if (platform_driver_register(&mt_couloMeter_driver)) {
		printk( "****[mt_couloMeter] Unable to register driver\n");
		return -ENODEV;
	}

	//add by zym,disable this solution,as facotry has fake battery
	data = PMIC_IMM_GetOneChannelValue(AUX_BATON_AP,5,0);
	//>250 power is coulometer,or power is machine power
	if( data > 250)
		g_power_is_couloMeter = 1;
	else
		g_power_is_couloMeter = 0;

	//g_power_is_couloMeter = 0;
	//i2c_register_board_info(1, &i2c_dev, 1);
	//printk("+++zym+++ i2c_register_board_info(1,$i2c_dev,1)\n");

	printk("[++++++zym+++++++]{%s} current bat_on voltage value is: (%d|%d)\n", __func__, data,g_power_is_couloMeter);
	
	return 0;
}

static void __exit cm_i2C_exit(void)
{
	printk("couloMeter_i2c_exit\n");
	i2c_del_driver(&CM_i2c_driver);
	platform_driver_unregister(&g_cm_Driver);
	platform_driver_unregister(&mt_couloMeter_driver);
}

module_init(cm_i2C_init);
module_exit(cm_i2C_exit);