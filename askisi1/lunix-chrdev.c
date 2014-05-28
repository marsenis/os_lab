/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * Makis Arsenis <marsenis@gmail.com>
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
long *lookups[] = { lookup_voltage, lookup_temperature, lookup_light };

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	
	WARN_ON ( !(sensor = state->sensor));

	return state->buf_timestamp < sensor->msr_data[state->type]->last_update;
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
	uint32_t timestamp, magic;
	long value;
	
	debug("entering state_update\n");

	/*
	 * Grab the raw data quickly, hold the
	 * spinlock for as little as possible.
	 */
	WARN_ON( !(sensor = state->sensor) );

	spin_lock(&sensor->lock);

	magic = sensor->msr_data[state->type]->magic;
	data = (uint16_t) sensor->msr_data[state->type]->values[0];
	timestamp = sensor->msr_data[state->type]->last_update;

	spin_unlock(&sensor->lock);
	/* Why use spinlocks? See LDD3, p. 119 */

	WARN_ON( magic != LUNIX_MSR_MAGIC );

	/*
	 * Any new data available?
	 */
	if (state->buf_timestamp >= timestamp)
		return -EAGAIN;

	/*
	 * Now we can take our time to format them,
	 * holding only the private state semaphore
	 */
	state->buf_timestamp = timestamp;

	switch (state->mode) {
		case LUNIX_IOC_COOKED:
			WARN_ON( state->type >= N_LUNIX_MSR );
			value = lookups[state->type][data];
			
			state->buf_lim = snprintf(state->buf_data, LUNIX_CHRDEV_BUFSZ, "%+ld.%.3ld\n", value / 1000, value % 1000);
			break;

		case LUNIX_IOC_RAW:
			state->buf_lim = snprintf(state->buf_data, LUNIX_CHRDEV_BUFSZ, "%hu\n", data);
			break;
	}
			

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
	unsigned int minor, sensor_id, sensor_type;
	struct lunix_chrdev_state_struct *state;
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
	if (!state) {
		ret = -EFAULT;
		goto out;
	}
	
	sensor_id   = minor >> 3;
	sensor_type = minor & 0x7;

	WARN_ON(sensor_id >= lunix_sensor_cnt || sensor_type >= N_LUNIX_MSR);

	state->sensor = &lunix_sensors[sensor_id];
	WARN_ON( state->sensor == NULL );

	state->type   = sensor_type;
	state->buf_timestamp = 0;
	state->mode = LUNIX_IOC_COOKED;

	sema_init(&state->lock, 1);

	filp->private_data = state;

out:
	debug("leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	struct lunix_chrdev_state_struct *state;

	state = (struct lunix_chrdev_state_struct *) filp->private_data;
	WARN_ON(!state);
	kfree(state);

	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct lunix_chrdev_state_struct *state;

	if (cmd != LUNIX_IOC_MODE)
		return -EINVAL;
	
	WARN_ON( (state = filp->private_data) == NULL );

	/*
	 * mode = 0 : "Cooked" data (default)
	 * mode = 1 : "Raw" data
	 */
	state->mode = !!arg;

	return 0;
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
			/* The process needs to sleep */
			/* See LDD3, page 153 for a hint */
			up(&state->lock);

			if (wait_event_interruptible(sensor->wq, lunix_chrdev_state_needs_refresh(state)))
				return -ERESTARTSYS;

			down(&state->lock);
		}

	}

	/* End of file */
	// There is no EOF
	
	/* Determine the number of cached bytes to copy to userspace */
	bytes_to_copy = my_min(cnt, state->buf_lim - *f_pos);
	WARN_ON( bytes_to_copy < 0 );

	ret = copy_to_user(usrbuf, state->buf_data + *f_pos, bytes_to_copy);
	if (ret != 0) {
		ret = -EFAULT;
		goto out;
	} else {
		ret = bytes_to_copy;
	}
	*f_pos += bytes_to_copy;

	/* Auto-rewind on EOF mode? */
	if (*f_pos >= state->buf_lim) {
		*f_pos = 0;
	}

out:
	/* Unlock? */
	up(&state->lock);
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
