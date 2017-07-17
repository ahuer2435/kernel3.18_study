/* BMA253 motion sensor driver
 *
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 *
 * VERSION: V1.3
 * HISTORY: V1.0 --- Driver creation
 *          V1.1 --- Add share I2C address function
 *          V1.2 --- Fix the bug that sometimes sensor is stuck after system resume.
 *          V1.3 --- Add FIFO interfaces.
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include "upmu_sw.h"
#include "upmu_common.h"
#include "batch.h" 

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#ifdef CONFIG_OF
#define USE_NEW_SENSOR_ARCH
#include <accel.h>
#endif
/* add by sugar -start -2015/09/23 */
#ifdef CUSTOM_KERNEL_SENSORHUB
#include <SCP_sensorHub.h>
#endif//#ifdef CUSTOM_KERNEL_SENSORHUB
/* add by sugar -end -2015/09/23 */

#include <cust_acc.h>
#include "bma253_acc.h"
#include "../../hwmon/include/sensors_io.h"

/*----------------------------------------------------------------------------*/
#define I2C_DRIVERID_BMA253 253
/*----------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/
//#define CONFIG_BMA253_LOWPASS   /*apply low pass filter on output*/       
#define SW_CALIBRATION
#define CONFIG_I2C_BASIC_FUNCTION
#define MAX_FIFO_F_LEVEL 32
#define MAX_FIFO_F_BYTES 6

/*----------------------------------------------------------------------------*/
#define BMA253_AXIS_X          0
#define BMA253_AXIS_Y          1
#define BMA253_AXIS_Z          2
#define BMA253_AXES_NUM        3
#define BMA253_DATA_LEN        6
#define BMA253_DEV_NAME        "BMA253"

#define BMA253_MODE_NORMAL      0
#define BMA253_MODE_LOWPOWER    1
#define BMA253_MODE_SUSPEND     2

#define BMA253_ACC_X_LSB__POS           4
#define BMA253_ACC_X_LSB__LEN           4
#define BMA253_ACC_X_LSB__MSK           0xF0
//#define BMA253_ACC_X_LSB__REG           BMA253_X_AXIS_LSB_REG

#define BMA253_ACC_X_MSB__POS           0
#define BMA253_ACC_X_MSB__LEN           8
#define BMA253_ACC_X_MSB__MSK           0xFF
//#define BMA253_ACC_X_MSB__REG           BMA253_X_AXIS_MSB_REG

#define BMA253_ACC_Y_LSB__POS           4
#define BMA253_ACC_Y_LSB__LEN           4
#define BMA253_ACC_Y_LSB__MSK           0xF0
//#define BMA253_ACC_Y_LSB__REG           BMA253_Y_AXIS_LSB_REG

#define BMA253_ACC_Y_MSB__POS           0
#define BMA253_ACC_Y_MSB__LEN           8
#define BMA253_ACC_Y_MSB__MSK           0xFF
//#define BMA253_ACC_Y_MSB__REG           BMA253_Y_AXIS_MSB_REG

#define BMA253_ACC_Z_LSB__POS           4
#define BMA253_ACC_Z_LSB__LEN           4
#define BMA253_ACC_Z_LSB__MSK           0xF0
//#define BMA253_ACC_Z_LSB__REG           BMA253_Z_AXIS_LSB_REG

#define BMA253_ACC_Z_MSB__POS           0
#define BMA253_ACC_Z_MSB__LEN           8
#define BMA253_ACC_Z_MSB__MSK           0xFF
//#define BMA253_ACC_Z_MSB__REG           BMA253_Z_AXIS_MSB_REG

#define BMA253_EN_LOW_POWER__POS          6
#define BMA253_EN_LOW_POWER__LEN          1
#define BMA253_EN_LOW_POWER__MSK          0x40
#define BMA253_EN_LOW_POWER__REG          BMA253_REG_POWER_CTL

#define BMA253_EN_SUSPEND__POS            7
#define BMA253_EN_SUSPEND__LEN            1
#define BMA253_EN_SUSPEND__MSK            0x80
#define BMA253_EN_SUSPEND__REG            BMA253_REG_POWER_CTL

#define BMA253_RANGE_SEL__POS             0
#define BMA253_RANGE_SEL__LEN             4
#define BMA253_RANGE_SEL__MSK             0x0F
#define BMA253_RANGE_SEL__REG             BMA253_REG_DATA_FORMAT

#define BMA253_BANDWIDTH__POS             0
#define BMA253_BANDWIDTH__LEN             5
#define BMA253_BANDWIDTH__MSK             0x1F
#define BMA253_BANDWIDTH__REG             BMA253_REG_BW_RATE

/* fifo mode*/
#define BMA253_FIFO_MODE__POS                 6
#define BMA253_FIFO_MODE__LEN                 2
#define BMA253_FIFO_MODE__MSK                 0xC0
#define BMA253_FIFO_MODE__REG                 BMA253_FIFO_MODE_REG

#define BMA253_FIFO_FRAME_COUNTER_S__POS             0
#define BMA253_FIFO_FRAME_COUNTER_S__LEN             7
#define BMA253_FIFO_FRAME_COUNTER_S__MSK             0x7F
#define BMA253_FIFO_FRAME_COUNTER_S__REG             BMA253_STATUS_FIFO_REG

#define BMA253_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)

#define BMA253_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id BMA253_i2c_id[] = {{BMA253_DEV_NAME,0},{}};
static struct i2c_board_info __initdata BMA253_i2c_info ={ I2C_BOARD_INFO(BMA253_DEV_NAME, BMA253_I2C_ADDR)};

/*----------------------------------------------------------------------------*/
extern int strict_strtoul(const char *cp, unsigned int base, unsigned long *res);
static int BMA253_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int BMA253_i2c_remove(struct i2c_client *client);
static int BMA253_suspend(struct i2c_client *client, pm_message_t msg);
static int BMA253_resume(struct i2c_client *client);
#ifdef USE_NEW_SENSOR_ARCH
static int gsensor_local_init(void);
static int gsensor_remove(void);
static int BMA253_init_flag =-1; // 0<==>OK -1 <==> fail
static struct acc_init_info BMA253_init_info = {
    .name = "BMA253",
    .init = gsensor_local_init,
    .uninit = gsensor_remove,
};
#endif
/*----------------------------------------------------------------------------*/
typedef enum {
    BMA_TRC_FILTER  = 0x01,
    BMA_TRC_RAWDATA = 0x02,
    BMA_TRC_IOCTL   = 0x04,
    BMA_TRC_CALI	= 0X08,
    BMA_TRC_INFO	= 0X10,
} BMA_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor{
    u8  whole;
    u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
    s16 raw[C_MAX_FIR_LENGTH][BMA253_AXES_NUM];
    int sum[BMA253_AXES_NUM];
    int num;
    int idx;
};
/*----------------------------------------------------------------------------*/
struct BMA253_i2c_data {
    struct i2c_client *client;
    struct acc_hw *hw;
    struct hwmsen_convert   cvt;
    
    /*misc*/
    struct data_resolution *reso;
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
	atomic_t				filter;
    s16                     cali_sw[BMA253_AXES_NUM+1];
    struct mutex lock;

    /*data*/
    s8                      offset[BMA253_AXES_NUM+1];  /*+1: for 4-byte alignment*/
    s16                     data[BMA253_AXES_NUM+1];
    u8			    fifo_count;

#if defined(CONFIG_BMA253_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif 
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver BMA253_i2c_driver = {
    .driver = {
        .name           = BMA253_DEV_NAME,
    },
	.probe      		= BMA253_i2c_probe,
	.remove    			= BMA253_i2c_remove,
#if !defined(CONFIG_HAS_EARLYSUSPEND)    
    .suspend            = BMA253_suspend,
    .resume             = BMA253_resume,
#endif
	.id_table = BMA253_i2c_id,
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *BMA253_i2c_client = NULL;
//atic struct platform_driver BMA253_gsensor_driver;
static struct BMA253_i2c_data *obj_i2c_data = NULL;
static bool sensor_power = true;
static struct GSENSOR_VECTOR3D gsensor_gain;
static struct mutex i2c_lock;
/*----------------------------------------------------------------------------*/
#define GSE_TAG                  "[Gsensor] "
#define GSE_FUN(f)               printk(GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    printk(GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk(GSE_TAG fmt, ##args)
 
/*----------------------------------------------------------------------------*/
static struct data_resolution BMA253_data_resolution[1] = {
 /* combination by {FULL_RES,RANGE}*/
    {{ 1, 0}, 1024},   // dataformat +/-2g  in 12-bit resolution;  { 1, 0} = 1.0= (2*2*1000)/(2^12);  1024 = (2^12)/(2*2)          
};
/*----------------------------------------------------------------------------*/ 

static struct acc_hw bma253_cust_acc_hw = {
    .i2c_num = 2,
    .direction = 5,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
    .firlen = 0, //old value 16                /*!< don't enable low pass fileter */
};

struct acc_hw* bma253_get_cust_acc_hw(void) 
{
    return &bma253_cust_acc_hw;
}
 

/* I2C operation functions */
static int bma_i2c_read_block(struct i2c_client *client,
			u8 addr, u8 *data, u8 len)
{
#ifdef CONFIG_I2C_BASIC_FUNCTION
	u8 beg = addr;
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,	.flags = 0,
			.len = 1,		.buf = &beg
		},
		{
			.addr = client->addr,	.flags = I2C_M_RD,
			.len = len,		.buf = data,
		}
	};
	int err;

	if (!client)
		return -EINVAL;
/*
	else if (len > C_I2C_FIFO_SIZE) {
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}
*/

	mutex_lock(&i2c_lock);
	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	mutex_unlock(&i2c_lock);
	if (err != 2) {
		GSE_ERR("i2c_transfer error: (%d %p %d) %d\n",
			addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;/*no error*/
	}

	return err;
#else
	int err = 0;
	err = i2c_smbus_read_i2c_block_data(client, addr, len, data);
	if (err < 0)
		return -1;
	return 0;
#endif
}
#define I2C_BUFFER_SIZE 256
static int bma_i2c_write_block(struct i2c_client *client, u8 addr,
			u8 *data, u8 len)
{
#ifdef CONFIG_I2C_BASIC_FUNCTION
	/*
	*because address also occupies one byte,
	*the maximum length for write is 7 bytes
	*/
	int err, idx = 0, num = 0;
	char buf[32];

	if (!client)
		return -EINVAL;
/*
	else if (len > C_I2C_FIFO_SIZE) {
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}
*/

	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
		buf[num++] = data[idx];

	mutex_lock(&i2c_lock);
	err = i2c_master_send(client, buf, num);
	mutex_unlock(&i2c_lock);
	if (err < 0) {
		GSE_ERR("send command error!!\n");
		return -EFAULT;
	} else {
		err = 0;/*no error*/
	}
	return err;
#else
	int err = 0;
	err = i2c_smbus_write_i2c_block_data(client, addr, len, data);
	if (err < 0)
		return -1;
	return 0;
#endif
}

/*--------------------BMA253 power control function----------------------------------*/
static void BMA253_power(struct acc_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	if(hw->power_id != POWER_NONE_MACRO)		// have externel LDO
	{        
		GSE_LOG("power %s\n", on ? "on" : "off");
		if(power_on == on)	// power status not change
		{
			GSE_LOG("ignore power control: %d\n", on);
		}
		else if(on)	// power on
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "BMA253"))
			{
				GSE_ERR("power on fails!!\n");
			}
		}
		else	// power off
		{
			if (!hwPowerDown(hw->power_id, "BMA253"))
			{
				GSE_ERR("power off fail!!\n");
			}			  
		}
	}
	power_on = on;    
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int BMA253_SetDataResolution(struct BMA253_i2c_data *obj)
{

/*set g sensor dataresolution here*/

/*BMA253 only can set to 10-bit dataresolution, so do nothing in BMA253 driver here*/

/*end of set dataresolution*/
 
 /*we set measure range from -2g to +2g in BMA253_SetDataFormat(client, BMA253_RANGE_2G), 
                                                    and set 10-bit dataresolution BMA253_SetDataResolution()*/
                                                    
 /*so BMA253_data_resolution[0] set value as {{ 3, 9}, 256} when declaration, and assign the value to obj->reso here*/  

 	obj->reso = &BMA253_data_resolution[0];
	return 0;
	
/*if you changed the measure range, for example call: BMA253_SetDataFormat(client, BMA253_RANGE_4G), 
you must set the right value to BMA253_data_resolution*/

}
/*----------------------------------------------------------------------------*/
static int BMA253_ReadData(struct i2c_client *client, s16 data[BMA253_AXES_NUM])
{     
	u8 addr = BMA253_REG_DATAXLOW;
	u8 buf[BMA253_DATA_LEN] = {0};
	int err = 0;

	if(NULL == client)
	{
		err = -EINVAL;
	}
	else if((err = bma_i2c_read_block(client, addr, buf, BMA253_DATA_LEN)))
	{
		GSE_ERR("error: %d\n", err);
	}
	else
	{
		/* Convert sensor raw data to 16-bit integer */
		data[BMA253_AXIS_X] = BMA253_GET_BITSLICE(buf[0], BMA253_ACC_X_LSB)
			|(BMA253_GET_BITSLICE(buf[1],
						BMA253_ACC_X_MSB)<<BMA253_ACC_X_LSB__LEN);
		data[BMA253_AXIS_X] = data[BMA253_AXIS_X] << (sizeof(short)*8-(BMA253_ACC_X_LSB__LEN
					+ BMA253_ACC_X_MSB__LEN));
		data[BMA253_AXIS_X] = data[BMA253_AXIS_X] >> (sizeof(short)*8-(BMA253_ACC_X_LSB__LEN
					+ BMA253_ACC_X_MSB__LEN));
		data[BMA253_AXIS_Y] = BMA253_GET_BITSLICE(buf[2], BMA253_ACC_Y_LSB)
			| (BMA253_GET_BITSLICE(buf[3],
						BMA253_ACC_Y_MSB)<<BMA253_ACC_Y_LSB__LEN);
		data[BMA253_AXIS_Y] = data[BMA253_AXIS_Y] << (sizeof(short)*8-(BMA253_ACC_Y_LSB__LEN
					+ BMA253_ACC_Y_MSB__LEN));
		data[BMA253_AXIS_Y] = data[BMA253_AXIS_Y] >> (sizeof(short)*8-(BMA253_ACC_Y_LSB__LEN
					+ BMA253_ACC_Y_MSB__LEN));
		data[BMA253_AXIS_Z] = BMA253_GET_BITSLICE(buf[4], BMA253_ACC_Z_LSB)
			| (BMA253_GET_BITSLICE(buf[5],
						BMA253_ACC_Z_MSB)<<BMA253_ACC_Z_LSB__LEN);
		data[BMA253_AXIS_Z] = data[BMA253_AXIS_Z] << (sizeof(short)*8-(BMA253_ACC_Z_LSB__LEN
					+ BMA253_ACC_Z_MSB__LEN));
		data[BMA253_AXIS_Z] = data[BMA253_AXIS_Z] >> (sizeof(short)*8-(BMA253_ACC_Z_LSB__LEN
					+ BMA253_ACC_Z_MSB__LEN));

#ifdef CONFIG_BMA253_LOWPASS
		if(atomic_read(&priv->filter))
		{
			if(atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend))
			{
				int idx, firlen = atomic_read(&priv->firlen);   
				if(priv->fir.num < firlen)
				{                
					priv->fir.raw[priv->fir.num][BMA253_AXIS_X] = data[BMA253_AXIS_X];
					priv->fir.raw[priv->fir.num][BMA253_AXIS_Y] = data[BMA253_AXIS_Y];
					priv->fir.raw[priv->fir.num][BMA253_AXIS_Z] = data[BMA253_AXIS_Z];
					priv->fir.sum[BMA253_AXIS_X] += data[BMA253_AXIS_X];
					priv->fir.sum[BMA253_AXIS_Y] += data[BMA253_AXIS_Y];
					priv->fir.sum[BMA253_AXIS_Z] += data[BMA253_AXIS_Z];
					if(atomic_read(&priv->trace) & BMA_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", priv->fir.num,
							priv->fir.raw[priv->fir.num][BMA253_AXIS_X], priv->fir.raw[priv->fir.num][BMA253_AXIS_Y], priv->fir.raw[priv->fir.num][BMA253_AXIS_Z],
							priv->fir.sum[BMA253_AXIS_X], priv->fir.sum[BMA253_AXIS_Y], priv->fir.sum[BMA253_AXIS_Z]);
					}
					priv->fir.num++;
					priv->fir.idx++;
				}
				else
				{
					idx = priv->fir.idx % firlen;
					priv->fir.sum[BMA253_AXIS_X] -= priv->fir.raw[idx][BMA253_AXIS_X];
					priv->fir.sum[BMA253_AXIS_Y] -= priv->fir.raw[idx][BMA253_AXIS_Y];
					priv->fir.sum[BMA253_AXIS_Z] -= priv->fir.raw[idx][BMA253_AXIS_Z];
					priv->fir.raw[idx][BMA253_AXIS_X] = data[BMA253_AXIS_X];
					priv->fir.raw[idx][BMA253_AXIS_Y] = data[BMA253_AXIS_Y];
					priv->fir.raw[idx][BMA253_AXIS_Z] = data[BMA253_AXIS_Z];
					priv->fir.sum[BMA253_AXIS_X] += data[BMA253_AXIS_X];
					priv->fir.sum[BMA253_AXIS_Y] += data[BMA253_AXIS_Y];
					priv->fir.sum[BMA253_AXIS_Z] += data[BMA253_AXIS_Z];
					priv->fir.idx++;
					data[BMA253_AXIS_X] = priv->fir.sum[BMA253_AXIS_X]/firlen;
					data[BMA253_AXIS_Y] = priv->fir.sum[BMA253_AXIS_Y]/firlen;
					data[BMA253_AXIS_Z] = priv->fir.sum[BMA253_AXIS_Z]/firlen;
					if(atomic_read(&priv->trace) & BMA_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
						priv->fir.raw[idx][BMA253_AXIS_X], priv->fir.raw[idx][BMA253_AXIS_Y], priv->fir.raw[idx][BMA253_AXIS_Z],
						priv->fir.sum[BMA253_AXIS_X], priv->fir.sum[BMA253_AXIS_Y], priv->fir.sum[BMA253_AXIS_Z],
						data[BMA253_AXIS_X], data[BMA253_AXIS_Y], data[BMA253_AXIS_Z]);
					}
				}
			}
		}	
#endif         
	}
	return err;
}
/*----------------------------------------------------------------------------*/
 
/*----------------------------------------------------------------------------*/
static int BMA253_ResetCalibration(struct i2c_client *client)
{
	struct BMA253_i2c_data *obj = i2c_get_clientdata(client);
	//u8 ofs[4]={0,0,0,0};
	//t err;
	
	#ifdef SW_CALIBRATION
		
	#else
		if(err = bma_i2c_write_block(client, BMA253_REG_OFSX, ofs, 4))
		{
			GSE_ERR("error: %d\n", err);
		}
	#endif

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));
	return 0;    
}
/*----------------------------------------------------------------------------*/
static int BMA253_ReadCalibration(struct i2c_client *client, int dat[BMA253_AXES_NUM])
{
    struct BMA253_i2c_data *obj = i2c_get_clientdata(client);
    int err = 0;
    int mul;

	#ifdef SW_CALIBRATION
		mul = 0;//only SW Calibration, disable HW Calibration
	#else
	    if ((err = BMA253_ReadOffset(client, obj->offset))) {
        GSE_ERR("read offset fail, %d\n", err);
        return err;
    	}    
    	mul = obj->reso->sensitivity/BMA253_offset_resolution.sensitivity;
	#endif

    dat[obj->cvt.map[BMA253_AXIS_X]] = obj->cvt.sign[BMA253_AXIS_X]*(obj->offset[BMA253_AXIS_X]*mul + obj->cali_sw[BMA253_AXIS_X]);
    dat[obj->cvt.map[BMA253_AXIS_Y]] = obj->cvt.sign[BMA253_AXIS_Y]*(obj->offset[BMA253_AXIS_Y]*mul + obj->cali_sw[BMA253_AXIS_Y]);
    dat[obj->cvt.map[BMA253_AXIS_Z]] = obj->cvt.sign[BMA253_AXIS_Z]*(obj->offset[BMA253_AXIS_Z]*mul + obj->cali_sw[BMA253_AXIS_Z]);                        
                                       
    return err;
}
/*----------------------------------------------------------------------------*/
static int BMA253_ReadCalibrationEx(struct i2c_client *client, int act[BMA253_AXES_NUM], int raw[BMA253_AXES_NUM])
{  
	/*raw: the raw calibration data; act: the actual calibration data*/
	struct BMA253_i2c_data *obj = i2c_get_clientdata(client);
	//int err;
	int mul;

	#ifdef SW_CALIBRATION
		mul = 0;//only SW Calibration, disable HW Calibration
	#else
		if(err = BMA253_ReadOffset(client, obj->offset))
		{
			GSE_ERR("read offset fail, %d\n", err);
			return err;
		}   
		mul = obj->reso->sensitivity/BMA253_offset_resolution.sensitivity;
	#endif
	
	raw[BMA253_AXIS_X] = obj->offset[BMA253_AXIS_X]*mul + obj->cali_sw[BMA253_AXIS_X];
	raw[BMA253_AXIS_Y] = obj->offset[BMA253_AXIS_Y]*mul + obj->cali_sw[BMA253_AXIS_Y];
	raw[BMA253_AXIS_Z] = obj->offset[BMA253_AXIS_Z]*mul + obj->cali_sw[BMA253_AXIS_Z];

	act[obj->cvt.map[BMA253_AXIS_X]] = obj->cvt.sign[BMA253_AXIS_X]*raw[BMA253_AXIS_X];
	act[obj->cvt.map[BMA253_AXIS_Y]] = obj->cvt.sign[BMA253_AXIS_Y]*raw[BMA253_AXIS_Y];
	act[obj->cvt.map[BMA253_AXIS_Z]] = obj->cvt.sign[BMA253_AXIS_Z]*raw[BMA253_AXIS_Z];                        
	                       
	return 0;
}
/*----------------------------------------------------------------------------*/
static int BMA253_WriteCalibration(struct i2c_client *client, int dat[BMA253_AXES_NUM])
{
	struct BMA253_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int cali[BMA253_AXES_NUM], raw[BMA253_AXES_NUM];
	//int divisor = obj->reso->sensitivity/lsb;

	if((err = BMA253_ReadCalibrationEx(client, cali, raw)))	/*offset will be updated in obj->offset*/
	{ 
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}

	GSE_LOG("OLDOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
		raw[BMA253_AXIS_X], raw[BMA253_AXIS_Y], raw[BMA253_AXIS_Z],
		obj->offset[BMA253_AXIS_X], obj->offset[BMA253_AXIS_Y], obj->offset[BMA253_AXIS_Z],
		obj->cali_sw[BMA253_AXIS_X], obj->cali_sw[BMA253_AXIS_Y], obj->cali_sw[BMA253_AXIS_Z]);

	/*calculate the real offset expected by caller*/
	cali[BMA253_AXIS_X] += dat[BMA253_AXIS_X];
	cali[BMA253_AXIS_Y] += dat[BMA253_AXIS_Y];
	cali[BMA253_AXIS_Z] += dat[BMA253_AXIS_Z];

	GSE_LOG("UPDATE: (%+3d %+3d %+3d)\n", 
		dat[BMA253_AXIS_X], dat[BMA253_AXIS_Y], dat[BMA253_AXIS_Z]);

#ifdef SW_CALIBRATION
	obj->cali_sw[BMA253_AXIS_X] = obj->cvt.sign[BMA253_AXIS_X]*(cali[obj->cvt.map[BMA253_AXIS_X]]);
	obj->cali_sw[BMA253_AXIS_Y] = obj->cvt.sign[BMA253_AXIS_Y]*(cali[obj->cvt.map[BMA253_AXIS_Y]]);
	obj->cali_sw[BMA253_AXIS_Z] = obj->cvt.sign[BMA253_AXIS_Z]*(cali[obj->cvt.map[BMA253_AXIS_Z]]);	
#else
	obj->offset[BMA253_AXIS_X] = (s8)(obj->cvt.sign[BMA253_AXIS_X]*(cali[obj->cvt.map[BMA253_AXIS_X]])/(divisor));
	obj->offset[BMA253_AXIS_Y] = (s8)(obj->cvt.sign[BMA253_AXIS_Y]*(cali[obj->cvt.map[BMA253_AXIS_Y]])/(divisor));
	obj->offset[BMA253_AXIS_Z] = (s8)(obj->cvt.sign[BMA253_AXIS_Z]*(cali[obj->cvt.map[BMA253_AXIS_Z]])/(divisor));

	/*convert software calibration using standard calibration*/
	obj->cali_sw[BMA253_AXIS_X] = obj->cvt.sign[BMA253_AXIS_X]*(cali[obj->cvt.map[BMA253_AXIS_X]])%(divisor);
	obj->cali_sw[BMA253_AXIS_Y] = obj->cvt.sign[BMA253_AXIS_Y]*(cali[obj->cvt.map[BMA253_AXIS_Y]])%(divisor);
	obj->cali_sw[BMA253_AXIS_Z] = obj->cvt.sign[BMA253_AXIS_Z]*(cali[obj->cvt.map[BMA253_AXIS_Z]])%(divisor);

	GSE_LOG("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
		obj->offset[BMA253_AXIS_X]*divisor + obj->cali_sw[BMA253_AXIS_X], 
		obj->offset[BMA253_AXIS_Y]*divisor + obj->cali_sw[BMA253_AXIS_Y], 
		obj->offset[BMA253_AXIS_Z]*divisor + obj->cali_sw[BMA253_AXIS_Z], 
		obj->offset[BMA253_AXIS_X], obj->offset[BMA253_AXIS_Y], obj->offset[BMA253_AXIS_Z],
		obj->cali_sw[BMA253_AXIS_X], obj->cali_sw[BMA253_AXIS_Y], obj->cali_sw[BMA253_AXIS_Z]);

	if(err = bma_i2c_write_block(obj->client, BMA253_REG_OFSX, obj->offset, BMA253_AXES_NUM))
	{
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
#endif

	return err;
}
/*----------------------------------------------------------------------------*/
static int BMA253_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[2];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*2);    

	res = bma_i2c_read_block(client, BMA253_REG_DEVID, databuf, 0x01);
	res = bma_i2c_read_block(client, BMA253_REG_DEVID, databuf, 0x01);
	if(res < 0)
		goto exit_BMA253_CheckDeviceID;

	if(databuf[0]!=BMA253_FIXED_DEVID)
	{
		printk("BMA253_CheckDeviceID %d failt!\n ", databuf[0]);
		return BMA253_SUCCESS;//BMA253_ERR_IDENTIFICATION;
	}
	else
	{
		printk("BMA253_CheckDeviceID %d pass!\n ", databuf[0]);
	}

	exit_BMA253_CheckDeviceID:
	if (res < 0)
	{
		return BMA253_ERR_I2C;
	}
	
	return BMA253_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int BMA253_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0};    
	int res = 0;
	struct BMA253_i2c_data *obj = i2c_get_clientdata(client);
	u8 actual_power_mode = 0;
	
	if(enable == sensor_power )
	{
		GSE_LOG("Sensor power status is newest!\n");
		return BMA253_SUCCESS;
	}
	
	mutex_lock(&obj->lock);
	if(enable == 1)
	{
		actual_power_mode = BMA253_MODE_NORMAL;
	}
	else
	{
		actual_power_mode = BMA253_MODE_SUSPEND;
	}
	
	res = bma_i2c_read_block(client,
			BMA253_MODE_CTRL_REG, &databuf[0], 1);
	res += bma_i2c_read_block(client,
		BMA253_LOW_POWER_CTRL_REG, &databuf[1], 1);

	switch (actual_power_mode) {
	case BMA253_MODE_NORMAL:
		databuf[0] = BMA253_SET_BITSLICE(databuf[0],
			BMA253_MODE_CTRL, 0);
		databuf[1] = BMA253_SET_BITSLICE(databuf[1],
			BMA253_LOW_POWER_MODE, 0);
		res += bma_i2c_write_block(client,
			BMA253_MODE_CTRL_REG, &databuf[0], 1);
		mdelay(1);
		res += bma_i2c_write_block(client,
			BMA253_LOW_POWER_CTRL_REG, &databuf[1], 1);
		mdelay(1);
	break;
	case BMA253_MODE_SUSPEND:
		databuf[0] = BMA253_SET_BITSLICE(databuf[0],
			BMA253_MODE_CTRL, 4);
		databuf[1] = BMA253_SET_BITSLICE(databuf[1],
			BMA253_LOW_POWER_MODE, 0);
		res += bma_i2c_write_block(client,
			BMA253_LOW_POWER_CTRL_REG, &databuf[1], 1);
		mdelay(1);
		res += bma_i2c_write_block(client,
			BMA253_MODE_CTRL_REG, &databuf[0], 1);
		mdelay(1);
	break;
	}

	if(res < 0)
	{
		GSE_ERR("set power mode failed, res = %d\n", res);
		mutex_unlock(&obj->lock);
		return BMA253_ERR_I2C;
	}
	sensor_power = enable;
	mutex_unlock(&obj->lock);
	
	return BMA253_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int BMA253_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	struct BMA253_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[2] = {0};    
	int res = 0;

	mutex_lock(&obj->lock);
	res = bma_i2c_read_block(client,
		BMA253_RANGE_SEL_REG, &databuf[0], 1);
	databuf[0] = BMA253_SET_BITSLICE(databuf[0],
		BMA253_RANGE_SEL, dataformat);
	res += bma_i2c_write_block(client,
		BMA253_RANGE_SEL_REG, &databuf[0], 1);
	mdelay(1);

	if(res < 0)
	{
		GSE_ERR("set data format failed, res = %d\n", res);
		mutex_unlock(&obj->lock);
		return BMA253_ERR_I2C;
	}
	mutex_unlock(&obj->lock);
	
	return BMA253_SetDataResolution(obj);    
}
/*----------------------------------------------------------------------------*/
static int BMA253_SetBWRate(struct i2c_client *client, u8 bwrate)
{
	u8 databuf[2] = {0};    
	int res = 0;
	struct BMA253_i2c_data *obj = i2c_get_clientdata(client);

	mutex_lock(&obj->lock);
	res = bma_i2c_read_block(client,
		BMA253_BANDWIDTH__REG, &databuf[0], 1);
	databuf[0] = BMA253_SET_BITSLICE(databuf[0],
		BMA253_BANDWIDTH, bwrate);
	res += bma_i2c_write_block(client,
		BMA253_BANDWIDTH__REG, &databuf[0], 1);
	mdelay(1);

	if(res < 0)
	{
		GSE_ERR("set bandwidth failed, res = %d\n", res);
		mutex_unlock(&obj->lock);
		return BMA253_ERR_I2C;
	}
	mutex_unlock(&obj->lock);

	return BMA253_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int BMA253_SetIntEnable(struct i2c_client *client, u8 intenable)
{
	int res = 0;
	struct BMA253_i2c_data *obj = i2c_get_clientdata(client);
	
	mutex_lock(&obj->lock);
	res = bma_i2c_write_block(client, BMA253_INT_REG_1, &intenable, 0x01);
	mdelay(1);
	if(res != BMA253_SUCCESS) 
	{
		mutex_unlock(&obj->lock);
		return res;
	}

	res = bma_i2c_write_block(client, BMA253_INT_REG_2, &intenable, 0x01);
	mdelay(1);
	if(res != BMA253_SUCCESS) 
	{
		mutex_unlock(&obj->lock);
		return res;
	}
	mutex_unlock(&obj->lock);
	printk("BMA253 disable interrupt ...\n");

	/*for disable interrupt function*/

	return BMA253_SUCCESS;	  
}

/*----------------------------------------------------------------------------*/
static int BMA253_init_client(struct i2c_client *client, int reset_cali)
{
	struct BMA253_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;
	printk("BMA253_init_client \n");

	res = BMA253_CheckDeviceID(client); 
	if(res != BMA253_SUCCESS)
	{
		return res;
	}	
	printk("BMA253_CheckDeviceID ok \n");
	
	res = BMA253_SetBWRate(client, BMA253_BW_100HZ);
	if(res != BMA253_SUCCESS ) 
	{
		return res;
	}
	printk("BMA253_SetBWRate OK!\n");
	
	res = BMA253_SetDataFormat(client, BMA253_RANGE_2G);
	if(res != BMA253_SUCCESS) 
	{
		return res;
	}
	printk("BMA253_SetDataFormat OK!\n");

	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;

	res = BMA253_SetIntEnable(client, 0x00);        
	if(res != BMA253_SUCCESS)
	{
		return res;
	}
	printk("BMA253 disable interrupt function!\n");

	res = BMA253_SetPowerMode(client, false);
	if(res != BMA253_SUCCESS)
	{
		return res;
	}
	printk("BMA253_SetPowerMode OK!\n");

	if(0 != reset_cali)
	{ 
		/*reset calibration only in power on*/
		res = BMA253_ResetCalibration(client);
		if(res != BMA253_SUCCESS)
		{
			return res;
		}
	}
	printk("BMA253_init_client OK!\n");
#ifdef CONFIG_BMA253_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif

	mdelay(20);

	return BMA253_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int BMA253_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];    

	memset(databuf, 0, sizeof(u8)*10);

	if((NULL == buf)||(bufsize<=30))
	{
		return -1;
	}
	
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "BMC056 Chip");
	return 0;
}
/*----------------------------------------------------------------------------*/ 
/*----------------------------------------------------------------------------*/
static int BMA253_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	struct BMA253_i2c_data *obj = (struct BMA253_i2c_data*)i2c_get_clientdata(client);
	//u8 databuf[20];
	int acc[BMA253_AXES_NUM];
	int res = 0;
	s16 databuf[BMA253_AXES_NUM];
	//memset(databuf, 0, sizeof(u8)*10);

	if(NULL == buf)
	{
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	if(sensor_power == 0)
	{
		res = BMA253_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on BMA253 error %d!\n", res);
		}
	}

	if((res = BMA253_ReadData(client, databuf)))
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{
		//printk("raw data x=%d, y=%d, z=%d \n",obj->data[BMA253_AXIS_X],obj->data[BMA253_AXIS_Y],obj->data[BMA253_AXIS_Z]);
		databuf[BMA253_AXIS_X] += obj->cali_sw[BMA253_AXIS_X];
		databuf[BMA253_AXIS_Y] += obj->cali_sw[BMA253_AXIS_Y];
		databuf[BMA253_AXIS_Z] += obj->cali_sw[BMA253_AXIS_Z];
		
		//printk("cali_sw x=%d, y=%d, z=%d \n",obj->cali_sw[BMA253_AXIS_X],obj->cali_sw[BMA253_AXIS_Y],obj->cali_sw[BMA253_AXIS_Z]);
		
		/*remap coordinate*/
		acc[obj->cvt.map[BMA253_AXIS_X]] = obj->cvt.sign[BMA253_AXIS_X]*databuf[BMA253_AXIS_X];
		acc[obj->cvt.map[BMA253_AXIS_Y]] = obj->cvt.sign[BMA253_AXIS_Y]*databuf[BMA253_AXIS_Y];
		acc[obj->cvt.map[BMA253_AXIS_Z]] = obj->cvt.sign[BMA253_AXIS_Z]*databuf[BMA253_AXIS_Z];
		//printk("cvt x=%d, y=%d, z=%d \n",obj->cvt.sign[BMA253_AXIS_X],obj->cvt.sign[BMA253_AXIS_Y],obj->cvt.sign[BMA253_AXIS_Z]);

		//GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[BMA253_AXIS_X], acc[BMA253_AXIS_Y], acc[BMA253_AXIS_Z]);

		//Out put the mg
		//printk("mg acc=%d, GRAVITY=%d, sensityvity=%d \n",acc[BMA253_AXIS_X],GRAVITY_EARTH_1000,obj->reso->sensitivity);
		acc[BMA253_AXIS_X] = acc[BMA253_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[BMA253_AXIS_Y] = acc[BMA253_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[BMA253_AXIS_Z] = acc[BMA253_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;		

		sprintf(buf, "%04x %04x %04x", acc[BMA253_AXIS_X], acc[BMA253_AXIS_Y], acc[BMA253_AXIS_Z]);
		if(atomic_read(&obj->trace) & BMA_TRC_IOCTL)
		{
			GSE_LOG("gsensor data: %s!\n", buf);
		}
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int BMA253_ReadRawData(struct i2c_client *client, char *buf)
{
	int res = 0;
	s16 databuf[BMA253_AXES_NUM];

	if (!buf || !client)
	{
		return EINVAL;
	}
	
	if((res = BMA253_ReadData(client, databuf)))
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return EIO;
	}
	else
	{
		sprintf(buf, "BMA253_ReadRawData %04x %04x %04x", databuf[BMA253_AXIS_X], 
			databuf[BMA253_AXIS_Y], databuf[BMA253_AXIS_Z]);
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/ 
 
/*----------------------------------------------------------------------------*/
int gsensor_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value, sample_delay;	
	struct BMA253_i2c_data *priv = (struct BMA253_i2c_data*)self;
	struct hwm_sensor_data* gsensor_data;
	char buff[BMA253_BUFSIZE];
	
	//GSE_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value <= 5)
				{
					sample_delay = BMA253_BW_200HZ;
				}
				else if(value <= 10)
				{
					sample_delay = BMA253_BW_100HZ;
				}
				else
				{
					sample_delay = BMA253_BW_50HZ;
				}
				
				//err = BMA253_SetBWRate(priv->client, sample_delay);
				if(err != BMA253_SUCCESS ) //0x2C->BW=100Hz
				{
					GSE_ERR("Set delay parameter error!\n");
				}

				if(value >= 50)
				{
					atomic_set(&priv->filter, 0);
				}
				else
				{	
				#if defined(CONFIG_BMA253_LOWPASS)
					priv->fir.num = 0;
					priv->fir.idx = 0;
					priv->fir.sum[BMA253_AXIS_X] = 0;
					priv->fir.sum[BMA253_AXIS_Y] = 0;
					priv->fir.sum[BMA253_AXIS_Z] = 0;
					atomic_set(&priv->filter, 1);
				#endif
				}
			}
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(((value == 0) && (sensor_power == false)) ||((value == 1) && (sensor_power == true)))
				{
					GSE_LOG("Gsensor device have updated!\n");
				}
				else
				{
					err = BMA253_SetPowerMode( priv->client, !sensor_power);
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				GSE_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				gsensor_data = (struct hwm_sensor_data *)buff_out;
				BMA253_ReadSensorData(priv->client, buff, BMA253_BUFSIZE);
				sscanf(buff, "%x %x %x", &gsensor_data->values[0], 
					&gsensor_data->values[1], &gsensor_data->values[2]);				
				gsensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;				
				gsensor_data->value_divide = 1000;
			}
			break;
		default:
			GSE_ERR("gsensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int BMA253_open(struct inode *inode, struct file *file)
{
	file->private_data = BMA253_i2c_client;

	if(file->private_data == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int BMA253_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
static long BMA253_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct BMA253_i2c_data *obj = (struct BMA253_i2c_data*)i2c_get_clientdata(client);	
	char strbuf[BMA253_BUFSIZE];
	void __user *data;
	struct SENSOR_DATA sensor_data;
	long err = 0;
	int cali[3];

	//GSE_FUN(f);
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case GSENSOR_IOCTL_INIT:
			BMA253_init_client(client, 0);			
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			BMA253_ReadChipInfo(client, strbuf, BMA253_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;
			}				 
			break;	  

		case GSENSOR_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			BMA253_ReadSensorData(client, strbuf, BMA253_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}				 
			break;

		case GSENSOR_IOCTL_READ_GAIN:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &gsensor_gain, sizeof(struct GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		case GSENSOR_IOCTL_READ_RAW_DATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			BMA253_ReadRawData(client, strbuf);
			if(copy_to_user(data, &strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}
			break;	  

		case GSENSOR_IOCTL_SET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&sensor_data, data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;	  
			}
			if(atomic_read(&obj->suspend))
			{
				GSE_ERR("Perform calibration in suspend state!!\n");
				err = -EINVAL;
			}
			else
			{
				cali[BMA253_AXIS_X] = sensor_data.x * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[BMA253_AXIS_Y] = sensor_data.y * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[BMA253_AXIS_Z] = sensor_data.z * obj->reso->sensitivity / GRAVITY_EARTH_1000;			  
				err = BMA253_WriteCalibration(client, cali);			 
			}
			break;

		case GSENSOR_IOCTL_CLR_CALI:
			err = BMA253_ResetCalibration(client);
			break;

		case GSENSOR_IOCTL_GET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if((err = BMA253_ReadCalibration(client, cali)))
			{
				break;
			}
			
			sensor_data.x = cali[BMA253_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.y = cali[BMA253_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.z = cali[BMA253_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			if(copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;
			}		
			break;
		

		default:
			GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}

	return err;
}

/* add by sugar -start -2015/09/23 */
#ifdef CONFIG_COMPAT
static long BMA253_compat_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
    long err = 0;

	void __user *arg32 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;
	
    switch (cmd)
    {
        case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:

            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
		        return err;
		    }
        break;

		/* add by sugar - start - 2015/9/23 */
		case COMPAT_GSENSOR_IOCTL_SET_CALI:

            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_SET_CALI unlocked_ioctl failed.");
		        return err;
		    }
        break;

		case COMPAT_GSENSOR_IOCTL_GET_CALI:

            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_CALI, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_GET_CALI unlocked_ioctl failed.");
		        return err;
		    }
        break;

		case COMPAT_GSENSOR_IOCTL_CLR_CALI:

            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_CLR_CALI, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_CLR_CALI unlocked_ioctl failed.");
		        return err;
		    }
        break;

		/* add by sugar - end   - 2015/9/23 */

        default:
            GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
            err = -ENOIOCTLCMD;
        break;

    } 

    return err;
} 
#endif
/* add by sugar -end -2015/09/23 */

/*----------------------------------------------------------------------------*/
static struct file_operations BMA253_fops = {
	//.owner = THIS_MODULE,
	.open = BMA253_open,
	.release = BMA253_release,
	.unlocked_ioctl = BMA253_unlocked_ioctl,
	/* add by sugar -start -2015/09/23 */
	#ifdef CONFIG_COMPAT
	.compat_ioctl = BMA253_compat_ioctl,
	#endif
	/* add by sugar -end -2015/09/23 */
};
/*----------------------------------------------------------------------------*/
static struct miscdevice BMA253_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &BMA253_fops,
};
/*----------------------------------------------------------------------------*/
#ifndef CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int BMA253_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct BMA253_i2c_data *obj = i2c_get_clientdata(client);    
	int err = 0;
	
	GSE_FUN();    

	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(obj == NULL)
		{
			GSE_ERR("null pointer!!\n");
			return -EINVAL;
		}
		atomic_set(&obj->suspend, 1);
		if((err = BMA253_SetPowerMode(obj->client, false)))
		{
			GSE_ERR("write power control fail!!\n");
			return 1;
		}       
		BMA253_power(obj->hw, 0);
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int BMA253_resume(struct i2c_client *client)
{
	struct BMA253_i2c_data *obj = i2c_get_clientdata(client);        
	int err;
	
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}

	BMA253_power(obj->hw, 1);
	if((err = BMA253_init_client(client, 0)))
	{
		GSE_ERR("initialize client fail!!\n");
		return err;        
	}

	atomic_set(&obj->suspend, 0);

	return 0;
}
/*----------------------------------------------------------------------------*/
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void BMA253_early_suspend(struct early_suspend *h) 
{
	struct BMA253_i2c_data *obj = container_of(h, struct BMA253_i2c_data, early_drv);   
	int err;
	
	GSE_FUN();    

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}
	atomic_set(&obj->suspend, 1); 
	if(err = BMA253_SetPowerMode(obj->client, false))
	{
		GSE_ERR("write power control fail!!\n");
		return;
	}

	BMA253_power(obj->hw, 0);
}
/*----------------------------------------------------------------------------*/
static void BMA253_late_resume(struct early_suspend *h)
{
	struct BMA253_i2c_data *obj = container_of(h, struct BMA253_i2c_data, early_drv);         
	int err;
	
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}

	BMA253_power(obj->hw, 1);
	if(err = BMA253_init_client(obj->client, 0))
	{
		GSE_ERR("initialize client fail!!\n");
		return;        
	}

	atomic_set(&obj->suspend, 0);    
}
/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/
/*----------------------------------------------------------------------------*/
static int BMA253_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct BMA253_i2c_data *obj;
	struct hwmsen_object sobj;
	int err = 0;
	
	GSE_FUN();

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	
	memset(obj, 0, sizeof(struct BMA253_i2c_data));

	obj->hw = bma253_get_cust_acc_hw();
	
	if((err = hwmsen_get_convert(obj->hw->direction, &obj->cvt)))
	{
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit;
	}

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client,obj);
	
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	mutex_init(&obj->lock);
	mutex_init(&i2c_lock);
	
#ifdef CONFIG_BMA253_LOWPASS
	if(obj->hw->firlen > C_MAX_FIR_LENGTH)
	{
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	}	
	else
	{
		atomic_set(&obj->firlen, obj->hw->firlen);
	}
	
	if(atomic_read(&obj->firlen) > 0)
	{
		atomic_set(&obj->fir_en, 1);
	}
	
#endif

	BMA253_i2c_client = new_client;	

	if((err = BMA253_init_client(new_client, 1)))
	{
		goto exit_init_failed;
	}

	if((err = misc_register(&BMA253_device)))
	{
		GSE_ERR("BMA253_device register failed\n");
		goto exit_misc_device_register_failed;
	} 
 
	sobj.self = obj;
	sobj.polling = 1;
	sobj.sensor_operate = gsensor_operate;
	if((err = hwmsen_attach(ID_ACCELEROMETER, &sobj)))
	{
		GSE_ERR("attach fail = %d\n", err);
		goto exit_kfree;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = BMA253_early_suspend,
	obj->early_drv.resume   = BMA253_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif 
#ifdef USE_NEW_SENSOR_ARCH
    BMA253_init_flag =1;
#endif
	GSE_LOG("%s: OK\n", __func__);    
	return 0;
	 
	exit_misc_device_register_failed: 
	misc_deregister(&BMA253_device);
	exit_init_failed:
	//i2c_detach_client(new_client);
	exit_kfree:
	kfree(obj);
	exit:
	GSE_ERR("%s: err = %d\n", __func__, err);        
	return err;
}

/*----------------------------------------------------------------------------*/
static int BMA253_i2c_remove(struct i2c_client *client)
{
	int err = 0;	 

	if((err = misc_deregister(&BMA253_device)))
	{
		GSE_ERR("misc_deregister fail: %d\n", err);
	}

	if((err = hwmsen_detach(ID_ACCELEROMETER)))
	{
		GSE_ERR("hwmsen_detach fail: %d\n", err);
	}	    
 
	BMA253_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}
/*----------------------------------------------------------------------------*/
#ifdef USE_NEW_SENSOR_ARCH
static int gsensor_local_init(void)
{
	struct acc_hw *hw = bma253_get_cust_acc_hw();
	
	GSE_FUN();

	BMA253_power(hw, 1);
	if(i2c_add_driver(&BMA253_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		return -1;
	}
	if(-1 == BMA253_init_flag)
	{
	   return -1;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int gsensor_remove(void)
{
    struct acc_hw *hw = bma253_get_cust_acc_hw();

    GSE_FUN();    
    BMA253_power(hw, 0);    
    i2c_del_driver(&BMA253_i2c_driver);
    return 0;
}
#else
static int BMA253_probe(struct platform_device *pdev) 
{
	struct acc_hw *hw = bma253_get_cust_acc_hw();
	
	GSE_FUN();

	BMA253_power(hw, 1);
	if(i2c_add_driver(&BMA253_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		return -1;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int BMA253_remove(struct platform_device *pdev)
{
    struct acc_hw *hw = bma253_get_cust_acc_hw();

    GSE_FUN();    
    BMA253_power(hw, 0);    
    i2c_del_driver(&BMA253_i2c_driver);
    return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver BMA253_gsensor_driver = {
	.probe      = BMA253_probe,
	.remove     = BMA253_remove,    
	.driver     = {
		.name  = "gsensor",
	}
};
#endif
/*----------------------------------------------------------------------------*/
static int __init BMA253_init(void)
{
	struct acc_hw *hw = bma253_get_cust_acc_hw();

	GSE_FUN();
	GSE_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num);
	i2c_register_board_info(hw->i2c_num, &BMA253_i2c_info, 1);
#ifdef USE_NEW_SENSOR_ARCH
		acc_driver_add(&BMA253_init_info);
#else
	if(platform_driver_register(&BMA253_gsensor_driver))
	{
		GSE_ERR("failed to register driver");
		return -ENODEV;
	}
#endif
	return 0;    
}
/*----------------------------------------------------------------------------*/
static void __exit BMA253_exit(void)
{
	GSE_FUN();
#ifndef USE_NEW_SENSOR_ARCH	
	platform_driver_unregister(&BMA253_gsensor_driver);
#endif
}
/*----------------------------------------------------------------------------*/
module_init(BMA253_init);
module_exit(BMA253_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BMA253 I2C driver");
MODULE_AUTHOR("hongji.zhou@bosch-sensortec.com");
