#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
// #include <mach/mt_pm_ldo.h>
#include <linux/delay.h>
#include "mt_gpio.h"
#include "gpio_const.h"
#include "mt_gpio_core.h"

//*****************for debug *****************//
#define DEBUG_SWITCH_CAPSENSOR 1
//open the debug log
#if    DEBUG_SWITCH_CAPSENSOR
#define CAPSENSOR_DEBUG(fmt,arg...)      printk("<<-CAP SENSOR->> "fmt"\n",##arg)
#else
#define CAPSENSOR_DEBUG(fmt,args...) /*do nothing */
#endif

#define CAP_PRINT(fmt,args...) printk("<<-CAP SENSOR->>"fmt"\n",##arg)

//****************Variables and define****************//

#ifndef FALSE
  #define FALSE (0)
#endif

#ifndef TRUE
  #define TRUE  (1)
#endif


bool capsensor_switch=FALSE;

bool capsensor_near=FALSE;

#define GPIO_CAP_EINT_PIN GPIO7  //liuzhigang 2015.11.17


static struct regulator *reg;

/*void init(struct device *dev)
{
//prototype: struct regulator *regulator_get_exclusive(struct device *dev, const char *id)
//dev is a device node, should be assign at driver probe stage
reg=regulator_get(dev,”vcn33_wifi”); // get pointer to regulator structure
}*/

extern struct regulator *regulator_get(struct device *dev, const char *id);
extern int regulator_enable(struct regulator *regulator);
extern int regulator_disable(struct regulator *regulator);
extern int regulator_set_voltage(struct regulator *regulator, int min_uV, int max_uV);


static int cap_enable(void)
{
    return regulator_enable(reg); //enable regulator
}

static int cap_disable(void)
{
    return regulator_disable(reg); //disable regulator
}

static int cap_set_voltage(void)
{
    return regulator_set_voltage(reg, 3300000, 3300000); // set 3.3v, min_uV should be same with max_uV
}


//*****************sys file system start*****************//
static ssize_t show_CapSensor_Switch(struct device *dev,struct device_attribute *attr, char *buf)
{
	if(TRUE==capsensor_switch)
	  return  sprintf(buf, "Capsensor is enabled.\n");
	else
	  return  sprintf(buf, "Capsensor is disabled.\n");
}
static ssize_t store_CapSensor_Switch(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	if((strcmp(buf,"1\n")==0)||(strcmp(buf,"enable\n")==0))
	  {
	  	CAPSENSOR_DEBUG("power on iqs128.\n");
        if(0 != cap_enable()){
            CAPSENSOR_DEBUG("power enable failed!");
            return -1;
        }
		capsensor_switch=TRUE;
	  }
	else if((strcmp(buf,"0\n")==0)||(strcmp(buf,"disable\n")==0))
	  {   
	  	CAPSENSOR_DEBUG("power off iqs128.\n");

        if(0 != cap_disable()){
        CAPSENSOR_DEBUG("power shutdown failed!");
        return -1;
        }

		capsensor_switch=FALSE;
	  } 
	else
	  {
		printk("<<-CAPSENSOR->> your input capsensor_switch =%s data is error\n",buf);
	  }
		  
    return size;
}

/*
* CapSensor_Data:
*664:  debug capsensor use adb command
*644:  just for meeting CTS test
*/
static DEVICE_ATTR(CapSensor_Switch, 0644, show_CapSensor_Switch, store_CapSensor_Switch);


static ssize_t show_CapSensor_Data(struct device *dev,struct device_attribute *attr, char *buf)
{
	  int result;
	if(TRUE==capsensor_switch)
	  {    
        result = mt_get_gpio_in(GPIO_CAP_EINT_PIN);
		printk("CapSensor_Data is %d\n", result);
		if(1 == result)
		   {  
			   CAPSENSOR_DEBUG("away");
			   capsensor_near=FALSE;
			   return sprintf(buf,"removed\n");
			}
		  else
		   {   CAPSENSOR_DEBUG("near");
			  capsensor_near=TRUE;
			  return sprintf(buf,"near\n");
		   } 
	  }
	else
	{
		CAPSENSOR_DEBUG("Capsensor is disabled,please enable it!");
	  	return  sprintf(buf, "Capsensor is disabled,please enable it!\n");
    }
}

/*
* CapSensor_Data:
*664:  debug capsensor use adb command
*644:  just for meeting CTS test
*/
static DEVICE_ATTR(CapSensor_Data,  0644, show_CapSensor_Data, NULL);
//*****************sys file system end*****************//
static int hw_init(struct device *dev)
{
    reg = regulator_get(dev, "vcn33_wifi"); // get pointer to regulator structure

    if(0 != cap_set_voltage()){
            CAPSENSOR_DEBUG("set voltage failed!");
            return -1;
    }

    if(0 != cap_enable()){
            CAPSENSOR_DEBUG("power enable failed!");
            return -1;
    }
	capsensor_switch = TRUE; //enbale the read data switch
    return 0;
}


static int iqs128_prob(struct platform_device *dev)
{ 
  int ret_device_file=0;
  CAPSENSOR_DEBUG("iqs128_prob");
  hw_init(&(dev->dev));
  ret_device_file = device_create_file(&(dev->dev), &dev_attr_CapSensor_Switch);
  ret_device_file = device_create_file(&(dev->dev), &dev_attr_CapSensor_Data);
  return 0;
}

static int iqs128_remove(struct platform_device *dev)    
{
    CAPSENSOR_DEBUG("iqs128_remove");
	
    return 0;
}

static void iqs128_shutdown(struct platform_device *dev)    
{
    CAPSENSOR_DEBUG("iqs128_shutdown");
    cap_disable();
 
}

static int iqs128_suspend(struct platform_device *dev, pm_message_t state)    
{
    CAPSENSOR_DEBUG("iqs128_suspend");
    return 0;
}

static int iqs128_resume(struct platform_device *dev)
{
   CAPSENSOR_DEBUG("iqs128_resume");
   return 0;
}

struct platform_device capsensor_device = {
    .name   = "capsensor",
    .id     = -1,
};

static struct platform_driver capsensor_driver = {
    .probe       = iqs128_prob,
    .remove      = iqs128_remove,
    .shutdown    = iqs128_shutdown,
    .suspend     = iqs128_suspend,
    .resume      = iqs128_resume,
    .driver      = {
    .name = "capsensor",
    },
};


static int capsensor_iqs128_init(void)
{
    int ret;
    printk("[capsensor][%s]:beging\n", __func__);
    ret = platform_device_register(&capsensor_device);
    if (ret) {
    printk("<<-CAPSENSOR->> Unable to register device (%d)\n", ret);
	return ret;
    }
    
    ret = platform_driver_register(&capsensor_driver);
    if (ret) {
    printk("<<-CAPSENSOR->> Unable to register driver (%d)\n", ret);
	return ret;
    }
    return 0;    
}

static void capsensor_iqs128_exit(void)
{
	CAPSENSOR_DEBUG("capsensor_iqs128_exit");
	platform_device_unregister(&capsensor_device);
	platform_driver_unregister(&capsensor_driver);

}

module_init(capsensor_iqs128_init);
module_exit(capsensor_iqs128_exit);

MODULE_AUTHOR("zhaozhenfei@huaqin.com");
MODULE_DESCRIPTION("capsensor iqs128 Device Driver");
MODULE_LICENSE("GPL");
