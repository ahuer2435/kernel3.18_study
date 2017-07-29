/*
 * linux/drivers/char/misc.c
 *
 * Generic misc open routine by Johan Myreen
 *
 * Based on code from Linus
 *
 * Teemu Rantanen's Microsoft Busmouse support and Derrick Cole's
 *   changes incorporated into 0.97pl4
 *   by Peter Cervasio (pete%q106fm.uucp@wupost.wustl.edu) (08SEP92)
 *   See busmouse.c for particulars.
 *
 * Made things a lot mode modular - easy to compile in just one or two
 * of the misc drivers, as they are now completely independent. Linus.
 *
 * Support for loadable modules. 8-Sep-95 Philip Blundell <pjb27@cam.ac.uk>
 *
 * Fixed a failing symbol register to free the device registration
 *		Alan Cox <alan@lxorguk.ukuu.org.uk> 21-Jan-96
 *
 * Dynamic minors and /proc/mice by Alessandro Rubini. 26-Mar-96
 *
 * Renamed to misc and miscdevice to be more accurate. Alan Cox 26-Mar-96
 *
 * Handling of mouse minor numbers for kerneld:
 *  Idea by Jacques Gelinas <jack@solucorp.qc.ca>,
 *  adapted by Bjorn Ekwall <bj0rn@blox.se>
 *  corrected by Alan Cox <alan@lxorguk.ukuu.org.uk>
 *
 * Changes for kmod (from kerneld):
 *	Cyrus Durgin <cider@speakeasy.org>
 *
 * Added devfs support. Richard Gooch <rgooch@atnf.csiro.au>  10-Jan-1998
 */

#include <linux/module.h>

#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/kmod.h>
#include <linux/gfp.h>

/*
 * Head entry for the doubly linked miscdevice list
 */
 //定义一个双向链表头，只包含头尾指针，不含数据。
 //相当于基类，用于其他结构继承，以此可以用链表方式访问其他结构，
 //是一种面向对象思想。
static LIST_HEAD(misc_list);

// 定义一个互斥锁，初始状态为未锁定。
static DEFINE_MUTEX(misc_mtx);

/*
 * Assigned numbers, used for dynamic minors
 */
#define DYNAMIC_MINORS 64 /* like dynamic majors */
static DECLARE_BITMAP(misc_minors, DYNAMIC_MINORS);

#ifdef CONFIG_PROC_FS
static void *misc_seq_start(struct seq_file *seq, loff_t *pos)
{
	mutex_lock(&misc_mtx);
	return seq_list_start(&misc_list, *pos);
}

static void *misc_seq_next(struct seq_file *seq, void *v, loff_t *pos)
{
	return seq_list_next(v, &misc_list, pos);
}

static void misc_seq_stop(struct seq_file *seq, void *v)
{
	mutex_unlock(&misc_mtx);
}

static int misc_seq_show(struct seq_file *seq, void *v)
{
	const struct miscdevice *p = list_entry(v, struct miscdevice, list);

	seq_printf(seq, "%3i %s\n", p->minor, p->name ? p->name : "");
	return 0;
}


static const struct seq_operations misc_seq_ops = {
	.start = misc_seq_start,
	.next  = misc_seq_next,
	.stop  = misc_seq_stop,
	.show  = misc_seq_show,
};

static int misc_seq_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &misc_seq_ops);
}

static const struct file_operations misc_proc_fops = {
	.owner	 = THIS_MODULE,
	.open    = misc_seq_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release,
};
#endif

/*
* 输入-- inode 和file指针。
* 输出-- 通过inode 获取次设备号，使用次设备号和内核链表获取相应的misc 结构，
* 然后获取其中的操作指针，将misc 设备和操作指针赋给filp 结构中相应的域，然后调用相应的open函数。
*/
static int misc_open(struct inode * inode, struct file * file)
{
	int minor = iminor(inode);		//获取次设备号。
	struct miscdevice *c;
	int err = -ENODEV;
	const struct file_operations *new_fops = NULL;

	mutex_lock(&misc_mtx);

	//获取次设备号对应的底层文件操作指针。
	list_for_each_entry(c, &misc_list, list) {
		if (c->minor == minor) {
			new_fops = fops_get(c->fops);		
			break;
		}
	}
		
	if (!new_fops) {
		mutex_unlock(&misc_mtx);
		//如果指针获取失败，加载次设备号对应的模块。
		request_module("char-major-%d-%d", MISC_MAJOR, minor);
		mutex_lock(&misc_mtx);
		//更新次设备号对应操作指针。
		list_for_each_entry(c, &misc_list, list) {
			if (c->minor == minor) {
				new_fops = fops_get(c->fops);
				break;
			}
		}
		if (!new_fops)
			goto fail;
	}

	err = 0;
	//file对应主设备号，这一步更新filp结构中的操作指针，使其指向次设备号对应的操作指针。
	replace_fops(file, new_fops);
	//如果次设备号对应设备的open函数不为空，则将次设备号对应的misc 结构赋值到file 结构中，
	// 并调用相应的open函数。
	if (file->f_op->open) {
		file->private_data = c;
		err = file->f_op->open(inode,file);
	}
fail:
	mutex_unlock(&misc_mtx);
	return err;
}

static struct class *misc_class;

static const struct file_operations misc_fops = {
	.owner		= THIS_MODULE,
	.open		= misc_open,
	.llseek		= noop_llseek,
};

/**
 *	misc_register	-	register a miscellaneous device
 *	@misc: device structure
 *	
 *	Register a miscellaneous device with the kernel. If the minor
 *	number is set to %MISC_DYNAMIC_MINOR a minor number is assigned
 *	and placed in the minor field of the structure. For other cases
 *	the minor number requested is used.
 *
 *	The structure passed is linked into the kernel and may not be
 *	destroyed until it has been unregistered.
 *
 *	A zero is returned on success and a negative errno code for
 *	failure.
 */
 /*
 * 输入-- miscdevice 设备结构。
 * 作用-- (1) 初始化参数misc 结构变量。
 * 		     (2) 确定次设备号，并基于设备模型 创建设备文件。
 * 		     (3) 将设置好的misc 结构加入内核链表misc_list。
*/
int misc_register(struct miscdevice * misc)
{
	dev_t dev;
	int err = 0;

	INIT_LIST_HEAD(&misc->list);		//初始化list 节点。

	mutex_lock(&misc_mtx);				//上锁。
	
	//动态分配此设备号或者检查指定的此设备号是否已经使用。
	if (misc->minor == MISC_DYNAMIC_MINOR) {
		int i = find_first_zero_bit(misc_minors, DYNAMIC_MINORS);
		if (i >= DYNAMIC_MINORS) {
			err = -EBUSY;
			goto out;
		}
		misc->minor = DYNAMIC_MINORS - i - 1;
		set_bit(i, misc_minors);
	} else {
		struct miscdevice *c;

		list_for_each_entry(c, &misc_list, list) {
			if (c->minor == misc->minor) {
				err = -EBUSY;
				goto out;
			}
		}
	}

	//计算设备号
	dev = MKDEV(MISC_MAJOR, misc->minor);

	//在/sys/class/misc/ 下创建设备文件。使用到了misc 结构中的this_device 和parent 指针，以及
	// Linux 统一设备模型。
	misc->this_device = device_create(misc_class, misc->parent, dev,
					  misc, "%s", misc->name);
	if (IS_ERR(misc->this_device)) {
		int i = DYNAMIC_MINORS - misc->minor - 1;
		if (i < DYNAMIC_MINORS && i >= 0)
			clear_bit(i, misc_minors);
		err = PTR_ERR(misc->this_device);
		goto out;
	}

	/*
	 * Add it to the front, so that later devices can "override"
	 * earlier defaults
	 */
	 //将这个misc 设备加入内核维护的misc_list 链表中。
	 //这个链表头是上面static 声明的misc_list
	list_add(&misc->list, &misc_list);
 out:
	mutex_unlock(&misc_mtx);
	return err;
}

/**
 *	misc_deregister - unregister a miscellaneous device
 *	@misc: device to unregister
 *
 *	Unregister a miscellaneous device that was previously
 *	successfully registered with misc_register(). Success
 *	is indicated by a zero return, a negative errno code
 *	indicates an error.
 */
/*
* 输入-- misc 结构
* 功能--(1) 从内核链表misc_list 中删除此misc 结构。
* 		    (2) 删除设备文件。
		    (3) 注销次设备号。
*/
int misc_deregister(struct miscdevice *misc)
{
	int i = DYNAMIC_MINORS - misc->minor - 1;

	if (WARN_ON(list_empty(&misc->list)))
		return -EINVAL;

	mutex_lock(&misc_mtx);
	list_del(&misc->list);
	device_destroy(misc_class, MKDEV(MISC_MAJOR, misc->minor));
	if (i < DYNAMIC_MINORS && i >= 0)
		clear_bit(i, misc_minors);
	mutex_unlock(&misc_mtx);
	return 0;
}

EXPORT_SYMBOL(misc_register);
EXPORT_SYMBOL(misc_deregister);

static char *misc_devnode(struct device *dev, umode_t *mode)
{
	struct miscdevice *c = dev_get_drvdata(dev);

	if (mode && c->mode)
		*mode = c->mode;
	if (c->nodename)
		return kstrdup(c->nodename, GFP_KERNEL);
	return NULL;
}

/*
* 功能-- 创建设备文件或者misc 类目录。
			使用字符设备接口注册misc 设备。
			主设备号为10.
*/
static int __init misc_init(void)
{
	int err;

#ifdef CONFIG_PROC_FS
	proc_create("misc", 0, NULL, &misc_proc_fops);
#endif
	misc_class = class_create(THIS_MODULE, "misc");
	err = PTR_ERR(misc_class);
	if (IS_ERR(misc_class))
		goto fail_remove;

	err = -EIO;
	if (register_chrdev(MISC_MAJOR,"misc",&misc_fops))
		goto fail_printk;
	misc_class->devnode = misc_devnode;
	return 0;

fail_printk:
	printk("unable to get major %d for misc devices\n", MISC_MAJOR);
	class_destroy(misc_class);
fail_remove:
	remove_proc_entry("misc", NULL);
	return err;
}

/*
* 向内核注册misc 子系统。
*/
subsys_initcall(misc_init);
