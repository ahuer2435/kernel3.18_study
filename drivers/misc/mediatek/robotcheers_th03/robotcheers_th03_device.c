#include <linux/irq.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>

#include <linux/gpio/consumer.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "robotcheers_th03_device.h"
#include <linux/sched.h>
#include <linux/param.h>

static const struct i2c_device_id ifly_i2c_id[] = {{"yanQiao", 0},{}};
static struct i2c_client *ifly_iic_client = NULL;


static DECLARE_WAIT_QUEUE_HEAD(robotcheers_th03_ifly_chip_status_waitq);
static volatile int ifly_chip_status_changed = 0;
static const struct gpio_desc *ifly_chip_gpio;

static int ifly_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int ifly_i2c_remove(struct i2c_client *client);
static int ifly_i2c_suspend(struct i2c_client *client,pm_message_t mesg);
static int ifly_i2c_resume(struct i2c_client *client);

static const struct of_device_id ifly_i2c_of_match[] = {
	{.compatible = "mediatek,yanqiao",},
	{},
};
MODULE_DEVICE_TABLE(of, ifly_i2c_of_match);

static struct i2c_driver ifly_i2c_driver = {
	.driver = {
		.name    = "yanQiao",
		.of_match_table    = ifly_i2c_of_match,
	},
	.probe       = ifly_i2c_probe,
	.id_table = ifly_i2c_id,
};

static const struct file_operations ifly_cmd_proc_fops = { 
	.read  = ifly_cmd_read,
	.write = ifly_cmd_write,
	.poll = ifly_cmd_poll,
	.unlocked_ioctl	= ifly_device_ioctl,
};

//platform devices
static struct platform_driver g_ifly_Driver = {
	.probe        = ifly_probe,
	.driver        = {
		.name    = "yanqiao",
	}
};
static struct platform_device g_ifly_device = {
	.name = "yanqiao",
	.id = -1,
};

//for porc debugfs
struct platform_device MT_ifly_device = {
	.name   = "mt_yanqiao",
	.id        = -1,
};

static struct platform_driver mt_ifly_driver = {
	.probe        = mt_ifly_probe,
	.driver       = {
		.name = "mt_yanqiao",
	},
};


/**********************************************************
 *
 *   [Global Variable] 
 *
 *********************************************************/

static DEFINE_MUTEX(ifly_reset_mutex);
static DEFINE_MUTEX(ifly_wakeup_mutex);
static DEFINE_MUTEX(ifly_i2c_access);
static DEFINE_MUTEX(ifly_wakeup_angle);

/**********************************************************
 *
 *   [I2C Function For Read/Write sn65dsi8x] 
 *
 *********************************************************/
void wait_s(int n)
{
	unsigned long j1;
	j1 = jiffies + HZ * n;
	while(time_before(jiffies,j1))
	{
		schedule();
	}
}


/**********************************************************
 *[name]	  		:ifly_read
 *[parameter]	  	:reg is register address which is wanted, read_value store read value,it is a int pointer.
 *[return]	  		:0, read success, others fail.
 *[desciption]:		read value is in read_value pointer, 
 *********************************************************/
static int ifly_read(uint8_t reg,int *read_value)
{
	uint8_t cmd_buf[1],value[5];
	int ret=0;
	int mutex_ret = -1;
	cmd_buf[0] = reg; 

	if(NULL == read_value)
	{
		printk("[ifly_read_byte] null point exception!!!!!!!\n");
		wait_s(1);
		return -1;
	}

	mutex_lock(&ifly_i2c_access);

	if(ifly_iic_client == NULL)
	{
		wait_s(1);
		return -1;
	}
	ret = i2c_master_send(ifly_iic_client, cmd_buf,1);

	if (ret != 1) 
	{    
		ifly_iic_client->ext_flag=0;
		mutex_unlock(&ifly_i2c_access);
		wait_s(1);
		return ret;
	}
	ret = i2c_master_recv(ifly_iic_client,value,4);
	if (ret != 4) 
	{    
		ifly_iic_client->ext_flag=0;
		mutex_unlock(&ifly_i2c_access);
		wait_s(1);
		return ret;
	}
	*read_value = ((value[0] << 0) | (value[1] << 8) | (value[2] << 16)| (value[3] << 24));
	mutex_ret = mutex_is_locked(&ifly_i2c_access);
	ifly_iic_client->ext_flag=0;
	mutex_unlock(&ifly_i2c_access);

	return 0;
}

/**********************************************************
 *[name]	  		:ifly_write
 *[parameter]	  	:reg is register address which is writed, data  is a write data,it is a int.
 *[return]	  		:0, write success, others fail.
 *[desciption]:		:NULL 
 *********************************************************/
static int ifly_write( struct i2c_client* client,uint8_t reg,uint32_t data)  
{  
	char buffer[5]; 
	int ret=0;
	mutex_lock(&ifly_i2c_access);
	printk("ifly cmd_write start!!!!!!!\n");
	buffer[0] = reg;
	buffer[1] = ((data >> 0) & 0xFF);
	buffer[2] = ((data >> 8) & 0xFF);
	buffer[3] = ((data >> 16) & 0xFF);
	buffer[4] = ((data >> 24) & 0xFF);
	ret = i2c_master_send(ifly_iic_client, buffer,5);
	if (ret < 0) 
	{	 
		ifly_iic_client->ext_flag=0;
		mutex_unlock(&ifly_i2c_access);
		printk("ifly_write write fail(%d)\n",ret);
		return ret;
	}

	ifly_iic_client->ext_flag=0;	
	mutex_unlock(&ifly_i2c_access);
	return 0;  
}

static int get_ifly_wakeup_angle(void)
{
	int angle = 0;
	int write_ret = 0;
	int read_ret = 0;
	int mutex_ret = -1;
	int flag = 0;
	int count_read = 0;
	
	mutex_lock(&ifly_wakeup_angle);
	mutex_ret = mutex_is_locked(&ifly_wakeup_angle);


	write_ret = ifly_write(ifly_iic_client,0x00, 0x00001000);
	if(write_ret != 0)
	{
		printk("get_ifly_wakeup_angle ifly_write fatal error,+^-^+ write_ret=%d\n",write_ret);
		wait_s(1);
		mutex_unlock(&ifly_wakeup_angle);
		return write_ret;
	}

	while((flag&0x01) != 0x01)
	{	
		mdelay(1);
		read_ret = ifly_read(0x00,&flag);
		if(read_ret != 0)
		{
			printk("get_ifly_wakeup_angle ifly_read fatal error,+^-^+ read_ret=%d\n",read_ret);
			wait_s(1);
			mutex_unlock(&ifly_wakeup_angle);
			return read_ret;
		}
		count_read++;
		if(count_read > 20)
		{
			return -20;
		}
	}
	
	read_ret = ifly_read(0x01,&angle);
	angle = (angle & 0xFF);
	if(read_ret != 0)
	{
		printk("get_ifly_wakeup_angle ifly_read fatal error,+^-^+ read_ret=%d\n",read_ret);
		wait_s(1);
		mutex_unlock(&ifly_wakeup_angle);
		return read_ret;
	}

	mutex_unlock(&ifly_wakeup_angle);
	return angle;
}

	
/**********************************************************
 *[name]	  :ifly_chip_reset
 *[return]	  :0, reset success, others fail.
 *[desciption]:reset ifly chip
 *********************************************************/
static int ifly_chip_reset(void)
{
	int write_ret = 0;
	int mutex_ret = -1;
	
	mutex_lock(&ifly_reset_mutex);
	mutex_ret = mutex_is_locked(&ifly_reset_mutex);

	write_ret = ifly_write(ifly_iic_client,0x00, 0x00001100);
	//wait_s(1);
	if(write_ret != 0)
	{
		wait_s(1);
		mutex_unlock(&ifly_reset_mutex);
		return write_ret;
	}
	mutex_unlock(&ifly_reset_mutex);
	return 0;	
}


/**********************************************************
 *[name]	  :get_ifly_chip_status
 *[return]	  :0, normal status, 1reset status.
 *[desciption]:get ifly status
 *********************************************************/
static int get_ifly_chip_status(void)
{
	int ret = gpiod_get_value(ifly_chip_gpio);

	return ret;
}

/**********************************************************
 *[name]	  :ifly_chip_wakeup
 *[return]	  :0, wakeup success, others fail.
 *[desciption]:wakeup ifly chip
 *********************************************************/
static int ifly_chip_wakeup(void)
{
	int write_ret = 0;
	int mutex_ret = -1;
	int flag = 0;
	int8_t cmd_complete = 0;
	int8_t cmd_ret = 0;
	int count_read = 0;
	int read_ret = 0;
	
	mutex_lock(&ifly_wakeup_mutex);
	mutex_ret = mutex_is_locked(&ifly_wakeup_mutex);
		
	write_ret = ifly_write(ifly_iic_client,0x00, 0x00001200);
	//wait_s(1);
	if(write_ret != 0)
	{
		printk("ifly_chip_wakeup ifly_write fatal error,+^-^+ write_ret=%d\n",write_ret);
		wait_s(1);
		mutex_unlock(&ifly_wakeup_mutex);
		return write_ret;
	}

	while(1)
	{	
		mdelay(1);
		read_ret = ifly_read(0x00,&flag);
		if(read_ret != 0)
		{
			mutex_unlock(&ifly_wakeup_mutex);
			return read_ret;
		}
		cmd_complete = (flag & 0xFF);
		cmd_ret = ((flag >> 16) & 0xFF);
		if((cmd_complete == 0x1)&&(cmd_ret == 0x3))
		{
			break;
		}
		count_read++;
		if(count_read > 20)
		{
			return -20;
		}
	}

	mutex_unlock(&ifly_wakeup_mutex);
	return 0;	
}


/**********************************************************
 *
 *   [I2C probe For Read/Write ifly] 
 *
 *********************************************************/
static int ifly_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err;
	if (!(ifly_iic_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit;
	}    
	memset(ifly_iic_client, 0, sizeof(struct i2c_client));
	ifly_iic_client = client;  
	ifly_iic_client->timing = 100;
	return 0;
exit:
	return 0;
}

static int ifly_probe(struct platform_device *pdev)
{
	mutex_init(&ifly_reset_mutex);
	mutex_init(&ifly_wakeup_mutex);			
	mutex_init(&ifly_i2c_access);
	mutex_init(&ifly_wakeup_angle);	
	return 0;
}

static ssize_t ifly_cmd_read(struct file *filp, char __user *buffer, size_t count, loff_t *ppos)
{
	unsigned long err;
	char c;

	if (count < 1) {
		return -EINVAL;
	}

	if (!ifly_chip_status_changed) {
		if (filp->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		} else {
			wait_event_interruptible(robotcheers_th03_ifly_chip_status_waitq, ifly_chip_status_changed);
		}
	}
	ifly_chip_status_changed = 0;

	c = get_ifly_chip_status() ? '1' : '0';
	err = copy_to_user(buffer, &c, 1);
	return err ? -EFAULT : 1;
}

static ssize_t ifly_cmd_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
#if 1
	int ret;
	ret = ifly_chip_wakeup();
	printk("ifly_chip_wakeup end ret= %d\n",ret );
	return 0;
#endif
	return -EINVAL;
}

static unsigned int ifly_cmd_poll( struct file *filp, struct poll_table_struct *wait)
{
	unsigned int mask = 0;
	poll_wait(filp, &robotcheers_th03_ifly_chip_status_waitq, wait);

	if (ifly_chip_status_changed)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static long ifly_device_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret;
	switch(cmd) 
	{
	case ROBOTCHEERS_TH03_IOCRESET:
		ret = ifly_chip_reset();
		break;
	case ROBOTCHEERS_TH03_IOCSTATUS:
		ret = get_ifly_chip_status();
		break;
	case ROBOTCHEERS_TH03_IOCWAKEUP:
		ret = ifly_chip_wakeup();
		break;
	case ROBOTCHEERS_TH03_IOCWAKEUP_ANGLE:
		ret = get_ifly_wakeup_angle();
		break;	
	default:
		return -EINVAL;
	}
	return ret;
}

void init_gpio_data(const struct gpio_desc *gpio)
{
	ifly_chip_gpio = gpio;
}

void RobotCheersTH03VoiceNotify(const struct gpio_desc *gpio)
{
	#if 1
	ifly_chip_gpio = gpio;
	ifly_chip_status_changed = 1;
	wake_up_interruptible(&robotcheers_th03_ifly_chip_status_waitq);
	#endif
}

static int mt_ifly_probe(struct platform_device *dev)    
{
	struct proc_dir_entry *ifly_dir = NULL;
	ifly_dir = proc_mkdir("cm_ifly_cmd", NULL);
	if (!ifly_dir){
		printk("[%s]: mkdir /proc/cm_ifly_cmd failed\n", __FUNCTION__);
	}
	else {
		proc_create("ifly_cmd", S_IRUGO | S_IWUSR, ifly_dir, &ifly_cmd_proc_fops);
	}
	return 0;
}

static int __init robotcheers_th03_device_init(void)
{

	printk("ifly_i2c_init\n");
	if(i2c_add_driver(&ifly_i2c_driver) == 0)
	{
		printk("Success to register ifly i2c driver.\n");
	}	
	else
	{
		printk("failed to register ifly i2c driver.\n");
	}
		
	if(platform_device_register(&g_ifly_device)){
		printk("failed to register ifly driver\n");
		return -ENODEV;
	}

	if(platform_driver_register(&g_ifly_Driver)){
		printk("Failed to register ifly driver\n");
		return -ENODEV;
	}

	if (platform_device_register(&MT_ifly_device)) {
		printk("****[mt_ifly] Unable to device register\n");
		return -ENODEV;
	}

	if (platform_driver_register(&mt_ifly_driver)) {
		printk( "****[mt_ifly] Unable to register driver\n");
		return -ENODEV;
	}

	init_gpio_data(gpio_to_desc(11));
	return 0;
}

static void __exit robotcheers_th03_device_exit(void)

{
	printk("ifly_i2c_exit\n");
	i2c_del_driver(&ifly_i2c_driver);
	platform_driver_unregister(&g_ifly_Driver);
	platform_driver_unregister(&mt_ifly_driver);
}

EXPORT_SYMBOL(RobotCheersTH03VoiceNotify);
MODULE_LICENSE("GPL"); 

module_init(robotcheers_th03_device_init);
module_exit(robotcheers_th03_device_exit);
