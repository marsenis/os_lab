/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * < Your name here >
 *
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

#define my_min(a, b) ( (a) < (b) ? (a) : (b) )

/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	
	WARN_ON ( !(sensor = state->sensor));
	/* ? */

	return state->buf_timestamp < sensor->msr_data[state->type]->last_update;
	/* The following return is bogus, just for the stub to compile */
	//return 0; /* ? */
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	uint16_t data;
	uint32_t timestamp;
	long value;
	int i;
	
	debug("entering state_update\n");

	/*
	 * Grab the raw data quickly, hold the
	 * spinlock for as little as possible.
	 */
	WARN_ON( !(sensor = state->sensor) );

	spin_lock(&sensor->lock);

	data = (uint16_t) sensor->msr_data[state->type]->values[0];
	timestamp = sensor->msr_data[state->type]->last_update;

	spin_unlock(&sensor->lock);
	/* ? */
	/* Why use spinlocks? See LDD3, p. 119 */

	/*
	 * Any new data available?
	 */
	if (state->buf_timestamp >= timestamp)
		return -EAGAIN;
	/* ? */

	/*
	 * Now we can take our time to format them,
	 * holding only the private state semaphore
	 */
	state->buf_timestamp = timestamp;

	switch (state->type) {
		case BATT:
			value = lookup_voltage[data];
			break;
		case TEMP:
			value = lookup_temperature[data];
			break;
		case LIGHT:
			value = lookup_light[data];
			break;
		default:
			printk(KERN_DEBUG "Internal Error: Unknown meassurement type");
			return -EINVAL;
	}
	
	sprintf(state->buf_data, "%ld.%.3ld\n", value / 1000, value % 1000);
	debug("Should return %s", state->buf_data);

	for (i = 0; i < LUNIX_CHRDEV_BUFSZ; i++)
		if (state->buf_data[i] == '\0')
			break;

	if (i > LUNIX_CHRDEV_BUFSZ) {
		printk(KERN_DEBUG "Buffer overflow detected");
		return -EFAULT;
	}

	state->buf_lim = i;
	/* ? */

	debug("leaving\n");
	return 0;
}

/*************************************
 * Implementation of file operations
* for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	/* Declarations */
	int minor;
	struct lunix_chrdev_state_struct *state;
	/* ? */
	int ret;

	debug("entering\n");
	ret = -ENODEV;
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;

	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */
	minor = iminor(inode);

	/* Allocate a new Lunix character device private state structure */
	state = (struct lunix_chrdev_state_struct *)
				kzalloc( sizeof(struct lunix_chrdev_state_struct), GFP_KERNEL );
	// TODO: Check kzalloc return value
	// TODO: Check type and sensor validity
	state->sensor = &lunix_sensors[minor >> 3];
	state->type = minor & 0x7;
	state->buf_timestamp = 0;
	sema_init(&state->lock, 1);

	filp->private_data = state;
	/* ? */
out:
	debug("leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	/* ? */
	struct lunix_chrdev_state_struct *state;

	state = (struct lunix_chrdev_state_struct *) filp->private_data;
	// TODO: Check for NULL pointer
	kfree(state);

	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	/* Why? */
	return -EINVAL;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	ssize_t ret;
	int bytes_to_copy;

	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;

	state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);

	/* Lock? */
	down(&state->lock);

	/*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement, do so
	 */
	if (*f_pos == 0) {
		while (lunix_chrdev_state_update(state) == -EAGAIN) {
			/* ? */
			up(&state->lock);

			debug("Wating...");

			// TODO: Check for errors in
			//	wait_event_interruptible
			if (wait_event_interruptible(sensor->wq, lunix_chrdev_state_needs_refresh(state)))
				return -ERESTARTSYS;
			/* The process needs to sleep */
			/* See LDD3, page 153 for a hint */

			debug("Woke up!");

			down(&state->lock);
		}

	}

	/* End of file */
	// marsenis: What end of file?
	/* ? */
	
	/* Determine the number of cached bytes to copy to userspace */
	bytes_to_copy = my_min(cnt, state->buf_lim - *f_pos);
	debug("Copying %d bytes", bytes_to_copy);
	if (bytes_to_copy < 0) {
		ret = -EINVAL;
		goto out;
	}

	ret = copy_to_user(usrbuf, state->buf_data + *f_pos, bytes_to_copy);
	if (ret != 0) {
		ret = -EINVAL;
		goto out;
	} else {
		ret = bytes_to_copy;
	}
	*f_pos += bytes_to_copy;
	/* ? */

	/* Auto-rewind on EOF mode? */
	if (*f_pos >= state->buf_lim) {
		*f_pos = 0;
	}
	/* ? */

out:
	up(&state->lock);
	/* Unlock? */
	return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct file_operations lunix_chrdev_fops = 
{
	.owner          = THIS_MODULE,
	.open           = lunix_chrdev_open,
	.release        = lunix_chrdev_release,
	.read           = lunix_chrdev_read,
	.unlocked_ioctl = lunix_chrdev_ioctl,
	.mmap           = lunix_chrdev_mmap
};

int lunix_chrdev_init(void)
{
	/*
	 * Register the character device with the kernel, asking for
	 * a range of minor numbers (number of sensors * 8 measurements / sensor)
	 * beginning with LINUX_CHRDEV_MAJOR:0
	 */
	int ret;
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;
	
	debug("initializing character device\n");
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;
	
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	/* ? */
	ret = register_chrdev_region(dev_no, lunix_minor_cnt, "Lunix:TNG");
	/* register_chrdev_region? */
	if (ret < 0) {
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}	
	/* ? */
	ret = cdev_add(&lunix_chrdev_cdev, dev_no, lunix_minor_cnt);
	/* cdev_add? */
	if (ret < 0) {
		debug("failed to add character device\n");
		goto out_with_chrdev_region;
	}
	debug("completed successfully\n");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
	return ret;
}

void lunix_chrdev_destroy(void)
{
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;
		
	debug("entering\n");
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("leaving\n");
}
