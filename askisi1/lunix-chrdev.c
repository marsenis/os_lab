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
#include <linux/strings.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;
char op[3][6] = { "BATT", "TEMP", "LIGHT" }; // *** TESTING ***
int minor, sensor_id, operation; // *** TESTING ***

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	
	WARN_ON ( !(sensor = state->sensor));
	/* ? */

	/* The following return is bogus, just for the stub to compile */
	return 0; /* ? */
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	uint16_t raw_data;
   int cooked_data, ipart, fpart;

	debug("entering\n");

	/*
	 * Grab the raw data quickly, hold the
	 * spinlock for as little as possible.
	 */
	/* ? */
	/* Why use spinlocks? See LDD3, p. 119 */
   
   sensor = state->sensor;
   spin_lock(&sensor->lock);
   
   /*
	 * Any new data available?
	 */
	/* ? */
   if (state->buf_timestamp == sensor->msr_data[state->type]->last_update) {
      debug("No new data...");
      spin_unlock(&sensor->lock);
      return -EAGAIN;
   }

   
   raw_data = (uint16_t) sensor->msr_data[state->type]->values[0];
   state->buf_timestamp = sensor->msr_data[state->type]->last_update;

   spin_unlock(&sensor->lock);

	/*
	 * Now we can take our time to format them,
	 * holding only the private state semaphore
	 */
   //it's alredy locked from lunix_chrdev_read

   switch(state->type) {
      case BATT:
         debug("convert voltage...");
         cooked_data = lookup_voltage[raw_data];
         break;
      case TEMP:
         debug("convert temperature...");
         cooked_data = lookup_temperature[raw_data];
         break;
      case LIGHT:
         debug("convert light...");
         cooked_data = lookup_light[raw_data];
         break;
      default:
         debug("WTF type is this?");
         return -1;
   }

   ipart = cooked_data/1000;
   fpart = cooked_data%1000;
   
   state->buf_lim = sprintf(state->data_buff, "%d.%d", ipart, fpart);
   //maybe we want state->buf_lim++, to include the null character
   //
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
	/* ? */
	int ret;
   struct lunix_chrdev_state_struct *state = NULL;

	debug("entering\n");
	ret = -ENODEV;
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;

	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */
	//minor = MINOR(inode->i_rdev);
   minor = iminor(inode);
	sensor_id = minor >> 3;
	operation = minor & 0x7;
	
	/* Allocate a new Lunix character device private state structure */
	/* ? */
   state = (struct lunix_chrdev_state_struct*) vmalloc(sizeof(struct lunix_chrdev_state_struct));
   if (lunix_chrdev_stat == NULL) {
      debug("Out of memory...\n");
      ret = -ENOMEM;
      goto out;
   }

   //switch-case for safety
   switch (operation) {
      case BATT:
         debug("type is BATT...\n");
         state->type = BATT;
         break;
      case TEMP:
         debug("type is TEMP...\n");
         state->type = TEMP;
         break;
      case LIGHT:
         debug("type is LIGHT...\n");
         state->type = LIGHT;
         break;
      default:
         debug("wrong minor number...\n");
         ret = -1;
         goto out;
   }

   sema_init(&state->lock, 1);
   
   state->sensor = &lunix_sensors[sensor_id];

   state->buf_timestamp = 0; //state->sensor->msr_data[operation]->last_update;

   filp->private_date = state;
out:
	debug("leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	/* ? */
   vfree(file->private_data); //free lunix_chrdev_state_struct
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

	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;

   state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);

	/* BEGIN: *** TESTING *** *
	char *mybuf;

	mybuf = kzalloc(35, GFP_KERNEL);

	sprintf(mybuf, "Sensor id: %d\nOperation: %s\n", sensor_id, op[operation]);
	ret = copy_to_user(usrbuf, mybuf, 33);
	
	kfree(mybuf);

	if (ret != 0) {
		ret = -EINVAL;
	} else {
		ret = 33;
	}

	goto out;
	* END:   *** TESTING *** */

	/* Lock? */
	/*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement, do so
	 */
	if (*f_pos == 0) {
      if (down_interruptible(&state->lock))
         return -ERESTARTSYS;

		while (lunix_chrdev_state_update(state) == -EAGAIN) {
			/* ? */
			/* The process needs to sleep */
			/* See LDD3, page 153 for a hint */
         up(&state->lock);

         //if (filp->f_flags & O_NONBLOCK)
         //    retun -EAGAIN;
         
         debug("feeling sleepy...");
         
         if (wait_event_interruptible(sensor->wq, state->buf_timestamp != sensor->msr_data[state->type]->last_update))
            return -ERESTARTSYS;
         
         if (down_interruptible(&state->lock))
            return -ERESTARTSYS;
		}
	}

	/* End of file */
	/* ? */
	
	/* Determine the number of cached bytes to copy to userspace */
	/* ? */
   ret = state->buf_lim - *f_pos;

   if (ret <= cnt) { //maybe needs casting
      if (copy_to_user(usrbuf, state->buf_data + *f_pos, ret)) {
         ret = -EFAULT;
         goto out;
      }
      *f_pos = 0;
   }
   else {
      if (copy_to_user(usrbuf, state->buf_data + *f_pos, cnt)) {
         ret = -EFAULT;
         goto out;
      }
      ret = cnt;
      *f_pos += cnt;
   }
   
   /* Auto-rewind on EOF mode*/
   /* ? */

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
