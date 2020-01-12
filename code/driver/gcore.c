/*
 * Wrapper Driver used to control a two-channel Xilinx DMA Engine
 */
#include "gcore.h"

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/iopoll.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <xen/page.h>

#include <linux/dma/xilinx_dma.h>
#include <linux/platform_device.h>

#define DMA_SIZE_HALF (DMA_SIZE/2)

/*
 * Base register address is set in devicetree.
 *
 * These are the offsets for each of the 32-bit registers.
 *
 */

#define GCORE_CONTROL_REGISTER 0x00
#define GCORE_STATUS_REGISTER 0x04
#define GCORE_ADDR_REGISTER 0x08
#define GCORE_DATA_REGISTER 0x0C
#define GCORE_A1_STATUS_REGISTER 0x10
#define GCORE_A2_STATUS_REGISTER 0x14


/*
 * gcore driver internal device
 * @regs: I/O mapped base address
 * @tx_chan: transmit dma channel
 * @tx_cmp: tx channel completion struct
 * @rx_chan: receive dma channel
 * @rx_cmp: rx channel completion struct
 * @control_offset: offset of the control reg
 * @status_offset: offset of the status reg
 * @addr_offset: offset of the addr reg
 * @data_offset: offset of the data reg
 * @control_reg: value of the control reg
 * @status_reg: value of the status reg
 * @addr_reg: value of the addr reg
 * @data_reg: value of the data reg
 *
 */
struct gcore_dev {
	void __iomem *regs;
    struct resource *res;
    struct dma_chan *tx_chan;	
    struct completion *tx_cmp;	
    struct dma_chan *rx_chan;	
    struct completion *rx_cmp;

    // offsets
    u32 control_offset;
    u32 status_offset;
    u32 addr_offset;
    u32 data_offset;
    u32 a1_status_offset;
    u32 a2_status_offset;

    // contents
	u32 control_reg;
	u32 status_reg;
	u32 addr_reg;
	u32 data_reg;
	u32 a1_status_reg;
	u32 a2_status_reg;
};


/*
 * gcore_system
 * @subcore_state: state of subcore to load
 * @artix_select: which artix unit are we currently working with
 * @is_busy: status bit indicating the device is busy
 * @gdev: gemini core device 
 * @buffer: contiguous coherent memory block used by mmap.
 * @dma_handle: the dma bus address
 */
struct gcore_system {
    // system
    enum subcore_states subcore_state;
    enum artix_selects artix_select;
    bool is_busy;
    struct gcore_dev *gdev;

    // locks
    spinlock_t lock;
    struct mutex sem;    
    
    // driver
    struct device *dev;
    dev_t chrdev_num;
    struct cdev chrdev;
    struct class *class;

    // dma
    char *buffer;
    size_t buffer_size;
    dma_addr_t dma_handle;
	
    // bitstream
    bool endian_swap;
};

/*
 * =======================================================
 * prototypes 
 *
 */

static void gcore_chan_config(struct gcore_system *gsys, struct gcore_chan_cfg *chan_cfg);
static int gcore_chan_prep(struct gcore_system *gsys,  struct gcore_chan_cfg *chan_cfg);
static int gcore_dma_start(struct gcore_transfer *trans, bool measure_time);
static void gcore_dma_stop(struct dma_chan *chan);

/*
 * =======================================================
 * helper functions 
 *
 */

static inline void reg_write(struct gcore_dev *gdev, u32 reg, u32 value)
{
	iowrite32(value, gdev->regs + reg);
}

static inline u32 reg_read(struct gcore_dev *gdev, u32 reg)
{
	return ioread32(gdev->regs + reg);
}

static void gcore_get_reg_info(struct gcore_system *gsys, struct gcore_registers *registers)
{
    struct gcore_dev *gdev = gsys->gdev;
    // read register values from gemini_core
    gdev->control_reg = reg_read(gdev, gdev->control_offset); 
    gdev->status_reg = reg_read(gdev, gdev->status_offset); 
    gdev->addr_reg = reg_read(gdev, gdev->addr_offset); 
    gdev->data_reg = reg_read(gdev, gdev->data_offset);
    gdev->a1_status_reg = reg_read(gdev, gdev->a1_status_offset); 
    gdev->a2_status_reg = reg_read(gdev, gdev->a2_status_offset); 
    
    // save them to the struct
    registers->control = gdev->control_reg;
    registers->status = gdev->status_reg;
    registers->addr = gdev->addr_reg;
    registers->data = gdev->data_reg;
    registers->a1_status = gdev->a1_status_reg;
    registers->a2_status = gdev->a2_status_reg;
    
    return;
}

static void gcore_get_userdev_info(struct gcore_system *gsys, struct gcore_userdev *userdev)
{
    struct gcore_dev *gdev = gsys->gdev;
    struct gcore_userdev *udev;

    /*
     * Userspace requested a gcore_dev so malloc a userdev and
     * populated it with the info and copy it back to userspace.
     *
     */
    if(gdev){
        udev = (struct gcore_userdev *)kzalloc(sizeof(struct gcore_userdev), GFP_KERNEL);
        if(udev){
            udev->tx_chan = (u32) gdev->tx_chan;
            udev->tx_cmp = (u32) gdev->tx_cmp;
            udev->rx_chan = (u32) gdev->rx_chan;
            udev->rx_cmp = (u32) gdev->rx_cmp;
	        memcpy(userdev, udev, sizeof(struct gcore_userdev));
        }
    }
}

static enum dma_transfer_direction gcore_to_dma_direction(enum gcore_direction gcore_dir)
{
	enum dma_transfer_direction dma_dir;

	switch (gcore_dir) {
	case GCORE_MEM_TO_DEV:
		dma_dir = DMA_MEM_TO_DEV;
		break;
	case GCORE_DEV_TO_MEM:
		dma_dir = DMA_DEV_TO_MEM;
		break;
	default:
		dma_dir = DMA_TRANS_NONE;
		break;
	}

	return dma_dir;
}

/*
 * =======================================================
 * subcore functions 
 *
 */

static inline u32 subcore_reset(struct gcore_system *gsys)
{
    u32 rc = 0;
    u32 reg = 0x00000000;
    struct gcore_dev * gdev = gsys->gdev;
    
    // reset core
    gdev->control_reg = reg_read(gdev, gdev->control_offset); 
    gdev->control_reg |= GCORE_CONTROL_RESET_MASK;
    reg_write(gdev, gdev->control_offset, gdev->control_reg);
    
    // wait until core resets
    rc = readl_poll_timeout (
        gdev->regs+gdev->status_offset, 
        reg, 
        !(reg & GCORE_STATUS_RESET_MASK), 
        250, // delay us
        1000000 // timout us
    );

    // timeout!
    if(rc){
        printk(KERN_ERR "%s: subcore failed to reset.\n", MODULE_NAME);
        return -ETIMEDOUT;
    }
    return 0;
}


/*
 * Performs a load operation. 
 *
 * Note: artix_select must already be selected either by a sysfs 
 * flag, or an ioctl call.
 *
 * Note: gcore subcore_state must be set as well.
 *
 */
static inline u32 subcore_load(struct gcore_system *gsys)
{
    char state_name[64];
    struct gcore_dev *gdev = gsys->gdev;
    u32 rc = 0;
    u32 reg = 0x00000000;
   
    // select artix unit
    switch(gsys->artix_select){
        case ARTIX_SELECT_NONE:
            reg |= GCORE_ARTIX_SELECT_NONE;
            break;
        case ARTIX_SELECT_A1:
            reg |= GCORE_ARTIX_SELECT_A1;
            break;
        case ARTIX_SELECT_A2:
            reg |= GCORE_ARTIX_SELECT_A2;
            break;
        case ARTIX_SELECT_BOTH:
            printk(KERN_ERR "%s: subcore load failed, \
               cannot select both artix units.\n", MODULE_NAME);
            return -1;
            break;
    }

    // load subcore with a state
    switch(gsys->subcore_state){
        case SUBCORE_IDLE:
            strcpy(state_name, "IDLE");
            reg |= GCORE_SUBCORE_MODE_IDLE;
            break;
        case SUBCORE_PAUSED:
            strcpy(state_name, "PAUSED");
            printk(KERN_ERR "%s: subcore load failed, \
               cannot load paused state directly.\n", MODULE_NAME);
            return -1;
            break;
        case CONFIG_SETUP:
            strcpy(state_name, "CONFIG_SETUP");
            reg |= GCORE_SUBCORE_MODE_CONFIG_SETUP;
            break;
        case CONFIG_LOAD:
            strcpy(state_name, "CONFIG_LOAD");
        case CONFIG_WAIT:
            strcpy(state_name, "CONFIG_WAIT");
            printk(KERN_ERR "%s: subcore load failed, \
               cannot set config load or wait directly.\n", MODULE_NAME);
            return -1;
            break;
        case AGENT_STARTUP:
            strcpy(state_name, "AGENT_STARTUP");
            reg |= GCORE_SUBCORE_MODE_AGENT_STARTUP;
            break;
        case SETUP_BURST:
            strcpy(state_name, "SETUP_BURST");
            reg |= GCORE_SUBCORE_MODE_SETUP_BURST;
            break;
        case SETUP_WRITE:
            strcpy(state_name, "SETUP_WRITE");
            reg |= GCORE_SUBCORE_MODE_SETUP_WRITE;
            break;
        case SETUP_READ:
            strcpy(state_name, "SETUP_READ");
            reg |= GCORE_SUBCORE_MODE_SETUP_READ;
            break;
        case SETUP_CLEANUP:
            strcpy(state_name, "SETUP_CLEANUP");
            reg |= GCORE_SUBCORE_MODE_SETUP_CLEANUP;
            break;
        case CTRL_WRITE:
            strcpy(state_name, "CTRL_WRITE");
            reg |= GCORE_SUBCORE_MODE_CTRL_WRITE;
            break;
        case CTRL_READ:
            strcpy(state_name, "CTRL_READ");
            reg |= GCORE_SUBCORE_MODE_CTRL_READ;
            break;
        case CTRL_RUN:
            strcpy(state_name, "CTRL_RUN");
            reg |= GCORE_SUBCORE_MODE_CTRL_RUN;
            break;
        case DMA_WRITE:
            strcpy(state_name, "DMA_WRITE");
            reg |= GCORE_SUBCORE_MODE_DMA_WRITE;
            break;
        case DMA_READ:
            strcpy(state_name, "DMA_READ");
            reg |= GCORE_SUBCORE_MODE_DMA_READ;
            break;
        default:
            printk(KERN_ERR "%s: subcore load failed, invalid state 0x%08X.\n", 
                MODULE_NAME, gsys->subcore_state);
            return -1;
    }

    // Check if artix_select is NONE for the following, which would
    // be a fatal error.
    switch(gsys->subcore_state){
        case SUBCORE_IDLE:
            break;
        case SUBCORE_PAUSED:
            break;
        case CONFIG_SETUP:
        case CONFIG_LOAD:
        case CONFIG_WAIT:
        case AGENT_STARTUP:
        case SETUP_BURST:
        case SETUP_WRITE:
        case SETUP_READ:
        case SETUP_CLEANUP:
        case CTRL_WRITE:
        case CTRL_READ:
        case CTRL_RUN:
        case DMA_WRITE:
        case DMA_READ:
            if (gsys->artix_select == ARTIX_SELECT_NONE){
                printk(KERN_ERR "%s: subcore load: subcore %i selected but artix select is NONE.\n", 
                    MODULE_NAME, gsys->subcore_state);
                return -1;
            }
            break;
    }
    
    printk(KERN_DEBUG "%s: subcore load: 0x%08X %s\n", MODULE_NAME, reg, state_name);

    // write data to the data register
    reg_write(gdev, gdev->data_offset, reg);

    // initiate the load by assert the load bit
    gdev->control_reg = reg_read(gdev, gdev->control_offset); 
    gdev->control_reg |= GCORE_CONTROL_LOAD_MASK;    
    reg_write(gdev, gdev->control_offset, gdev->control_reg);
    
    // wail until control reg load bit de-asserts
    reg = 0;
    rc = readl_poll_timeout (
        gdev->regs+gdev->control_offset, 
        reg, 
        !(reg & GCORE_CONTROL_LOAD_MASK), 
        250, // delay us
        2000000 // timout us
    );
    
    // timeout!
    if(rc){
        printk(KERN_ERR "%s: subcore load: bit failed to de-assert.\n", MODULE_NAME);
        return -ETIMEDOUT;
    }

    // after a load, subcore will enter the paused state 
    // waiting for the run command.
    reg = 0;
    rc = readl_poll_timeout (
        gdev->regs+gdev->status_offset, 
        reg, 
        (reg & GCORE_STATUS_PAUSED_MASK), 
        250, // delay us
        2000000 // timout us
    );
    
    // subcore failed to enter paused state within 2 seconds! 
    if(rc){
        printk(KERN_ERR "%s: subcore load: paused bit failed to assert.\n", MODULE_NAME);
        return -ETIMEDOUT;
    }
    
    return 0;
}

/*
 * Once subcore is in the paused state, it's ready to roll. 
 * You can now assert the run bit to perform the loaded action.
 *
 * Once run is initiated, the running status bit will go high.
 *
 * Note: if subcore_state is set to CONFIG, subcore will block on axis
 * valid. Once we stop writing data, kernel will automatically
 * assert the tlast signal and subcore will enter the WAIT_CONFIG
 * state. At which point we need to block until done bit goes high.
 *
 */
static inline u32 subcore_run(struct gcore_system *gsys)
{
    struct gcore_dev *gdev = gsys->gdev;
    u32 rc_run = 0, rc_running = 0;
    u32 reg_run = 0x00000000, reg_running = 0x00000000;
    
    gdev->status_reg = reg_read(gdev, gdev->status_offset); 
    if((gdev->status_reg & GCORE_STATUS_PAUSED_MASK) != GCORE_STATUS_PAUSED_MASK){
        printk(KERN_ERR "%s: subcore run failed because not in paused state. \
            status: 0x%08X\n", MODULE_NAME, gdev->status_reg);
        return -1;
    }

    // initiate the run by assert the run bit
    gdev->control_reg = reg_read(gdev, gdev->control_offset); 
    gdev->control_reg |= GCORE_CONTROL_RUN_MASK;    
    reg_write(gdev, gdev->control_offset, gdev->control_reg);

    // wail until control reg load bit de-asserts
    reg_run = 0;
    rc_run = readl_poll_timeout (
        gdev->regs+gdev->control_offset, 
        reg_run, 
        !(reg_run & GCORE_CONTROL_RUN_MASK), 
        0, // delay us
        10000 // timout us
    );
    
    // Run bit will de-assert right after subcore gets it,
    // so error out if it doesn't.
    if(rc_run){
        printk(KERN_ERR "%s: subcore run bit failed to de-assert.\n", MODULE_NAME);
        return -ETIMEDOUT;
    }

    // after a load, subcore will enter the paused state 
    // waiting for the run command. Agent startup takes
    // at least ~1 sec for init calib to complete so 
    // set timeout to 2 seconds.
    rc_running = readl_poll_timeout (
        gdev->regs+gdev->status_offset, 
        reg_running, 
        (reg_running & GCORE_STATUS_RUNNING_MASK), 
        0, // delay us
        2000000 // timout us
    );
    
    if(rc_running){
        reg_running = reg_read(gdev, gdev->status_offset); 
        if((reg_running & GCORE_STATUS_INIT_ERROR_MASK) == GCORE_STATUS_INIT_ERROR_MASK){
            printk(KERN_ERR 
                "%s: subcore ran but init_error asserted with code 0x%08X.\n", 
                MODULE_NAME, reg_running & GCORE_STATUS_ERROR_CODE_MASK);
            return -1;
        }else if((reg_running & GCORE_STATUS_DONE_ERROR_MASK) == GCORE_STATUS_DONE_ERROR_MASK){
            printk(KERN_ERR 
                "%s: subcore ran but done_error asserted with code 0x%08X.\n", 
                MODULE_NAME, reg_running & GCORE_STATUS_ERROR_CODE_MASK);
            return -1;
        }else {
            printk(KERN_ERR "%s: subcore running bit failed to assert.\n", MODULE_NAME);
        }

        return -ETIMEDOUT;
    }
    
    return 0;
}

/*
 * Wait for subcore to go back to IDLE state at which point you can
 * check the status of done_error.
 *
 */
static inline u32 subcore_idle(struct gcore_system *gsys){
    
    struct gcore_dev *gdev = gsys->gdev;
    u32 status = 0;
    u32 rc = 0;
    u32 reg = 0x00000000;
    
    // wail until status idle bit asserts
    rc = readl_poll_timeout (
        gdev->regs+gdev->status_offset, 
        reg, 
        (reg & GCORE_STATUS_IDLE_MASK), 
        250, // delay us
        2000000 // timout us
    );
    
    // subcore never went back to idle. It only waits 5ms for done to go high before
    // setting done_error and then going back to idle state.
    if(rc){
        printk(KERN_ERR "%s: subcore: idle failed to assert.\n", MODULE_NAME);
        status = -ETIMEDOUT;
        goto error;
    }
 
    // Subcore is now in IDLE state.
error:
    return status;

}

/*
 * Performs a control register write sending what's in the 
 * addr and data regs to subcore.
 *
 */
static inline u32 subcore_ctrl_write(struct gcore_system *gsys, struct gcore_ctrl_packet *ctrl_packet) {
    struct gcore_dev *gdev = gsys->gdev;
	int ret = 0;
    u32 rc = 0;
    u32 reg = 0x00000000;

    // lock
    ret = mutex_lock_interruptible(&gsys->sem);
    if (ret){
        goto err_unlock;
    }
    
    // set write mask
    gdev->control_reg = reg_read(gdev, gdev->control_offset); 
    gdev->control_reg |= GCORE_CONTROL_WRITE_MASK;    
    
    // set rank_sel bit
    if(ctrl_packet->rank_select) {
        gdev->control_reg |= GCORE_CONTROL_RANK_SELECT_MASK;    
    } else {
        gdev->control_reg &= ~GCORE_CONTROL_RANK_SELECT_MASK;
    }

    // write addr and data
    gdev->addr_reg = ctrl_packet->addr;
    gdev->data_reg = ctrl_packet->data;
    reg_write(gdev, gdev->addr_offset, gdev->addr_reg);
    reg_write(gdev, gdev->data_offset, gdev->data_reg);
    
    // perform the ctrl_write
    reg_write(gdev, gdev->control_offset, gdev->control_reg);
    rc = readl_poll_timeout (
        gdev->regs+gdev->control_offset, 
        reg, 
        !(reg & GCORE_CONTROL_WRITE_MASK), 
        100, // delay us
        200000 // timout us
    );
    if(rc){
        printk(KERN_ERR "%s ioctl: control write bit failed to de-assert\n", MODULE_NAME);
        ret = -ETIMEDOUT;
        goto err_unlock;
    }
err_unlock:
    mutex_unlock(&gsys->sem);
    return ret;
}

/*
 * Performs a control register read, reading from subcore into 
 * the addr and data regs.
 *
 */
static inline u32 subcore_ctrl_read(struct gcore_system *gsys, struct gcore_ctrl_packet *ctrl_packet) {
    struct gcore_dev *gdev = gsys->gdev;
	int ret = 0;
    u32 rc = 0;
    u32 reg = 0x00000000;

    // grab the mutex
    ret = mutex_lock_interruptible(&gsys->sem);
    if (ret){
        goto err_unlock;
    }
    
    // set the read mask
    gdev->control_reg = reg_read(gdev, gdev->control_offset); 
    gdev->control_reg |= GCORE_CONTROL_READ_MASK;    

    // peform the ctrl_axi read
    reg_write(gdev, gdev->control_offset, gdev->control_reg);
    rc = readl_poll_timeout (
        gdev->regs+gdev->control_offset, 
        reg, 
        !(reg & GCORE_CONTROL_READ_MASK), 
        100, // delay us
        200000 // timout us
    );

    // READ bit failed to de-assert
    if(rc){
        printk(KERN_ERR "%s ioctl: control read bit failed to de-assert\n", MODULE_NAME);
        ret = -ETIMEDOUT;
        goto err_unlock;
    }

    // read addr and data registers
    gdev->addr_reg = reg_read(gdev, gdev->addr_offset); 
    gdev->data_reg = reg_read(gdev, gdev->data_offset); 
    
    // set packet values
    ctrl_packet->rank_select = 0;
    ctrl_packet->addr = gdev->addr_reg;
    ctrl_packet->data = gdev->data_reg;
    
    printk(KERN_DEBUG "%s ioctl: read addr: 0x%08X data: 0x%08X\n", 
        MODULE_NAME, ctrl_packet->addr, ctrl_packet->data);
    
err_unlock:
    mutex_unlock(&gsys->sem);
    return ret;
}

/*
 * Asserts or de-asserts artix sync line, which goes to both artix units.
 * Pass a zero or one through ctrl_packet data. 
 *
 */
static inline u32 artix_sync(struct gcore_system *gsys, struct gcore_ctrl_packet *ctrl_packet)
{
    u32 rc = 0;
	u32 ret = 0;
    u32 reg = 0x00000000;
    struct gcore_dev *gdev = gsys->gdev;
    u32 good_val = 0x00000000;

    // lock
    ret = mutex_lock_interruptible(&gsys->sem);
    if (ret){
        goto err_unlock;
    }

    if((ctrl_packet->data & 0x00000001) == 1) {
        good_val = GCORE_STATUS_SYNCED_MASK;
    }else if((ctrl_packet->data & 0x00000001) == 0){
        good_val = 0x00000000;
    }else{
        printk(KERN_ERR "%s: invalid sync value given in data register.\n", MODULE_NAME);
        ret = -EINVAL;
        goto err_unlock;
    }

    
    // write to data reg since contro_sync look at data_reg[0]
    // for sync value
    gdev->data_reg = ctrl_packet->data;
    reg_write(gdev, gdev->data_offset, gdev->data_reg);

    // run control_sync
    gdev->control_reg = reg_read(gdev, gdev->control_offset); 
    gdev->control_reg |= GCORE_CONTROL_SYNC_MASK;
    reg_write(gdev, gdev->control_offset, gdev->control_reg);

    // check if control sync flag de-asserted
    reg = 0;
    rc = readl_poll_timeout (
        gdev->regs+gdev->control_offset, 
        reg, 
        !(reg & GCORE_CONTROL_SYNC_MASK), 
        0, // delay us
        100000 // timout us
    );

    // timeout!
    if(rc){
        printk(KERN_ERR "%s: control register artix sync flag failed to de-assert.\n", MODULE_NAME);
        ret = -ETIMEDOUT;
        goto err_unlock;
    }

    // wait until we see sync in status
    reg = 0;
    rc = readl_poll_timeout (
        gdev->regs+gdev->status_offset, 
        reg, 
        ((reg & GCORE_STATUS_SYNCED_MASK) == good_val), 
        1, // delay us
        10000 // timout us
    );

    // timeout!
    if(rc){
        printk(KERN_ERR "%s: failed to assert artix sync.\n", MODULE_NAME);
        ret = -ETIMEDOUT;
        goto err_unlock;
    }

err_unlock:
    mutex_unlock(&gsys->sem);
    return ret;
}

/*
 * =======================================================
 * File operation functions
 *
 */

static int gcore_open(struct inode *inode, struct file *file)
{
    int status;
    struct gcore_system *gsys;
    
    printk(KERN_DEBUG "%s file: open()\n", MODULE_NAME);
    gsys = container_of(inode->i_cdev, struct gcore_system, chrdev);

    status = mutex_lock_interruptible(&gsys->sem);
	if (status){
        goto error;
    }

	//if (gsys->is_busy) {
	//	status = -EBUSY;
    //    goto error;
	//}

	file->private_data = gsys;
    //gsys->is_busy = 1;
    gsys->endian_swap = 0;

error:
    mutex_unlock(&gsys->sem);
	return status;
}


/*
 * Close has been called on the file so set the is_busy to 0;
 *
 */
static int gcore_release(struct inode *inode, struct file *file)
{
    struct gcore_system *gsys = file->private_data;
    gsys->is_busy = 0;
    printk(KERN_DEBUG "%s file: close()\n", MODULE_NAME);
	return 0;
}

/*
 * Read doesn't do anything for now.
 *
 */
static ssize_t gcore_read(struct file *file, char __user * buf, size_t
    len, loff_t * off)
{
    u32 status = 0;
    struct gcore_system *gsys = file->private_data;
    printk(KERN_DEBUG "%s file: read()\n", MODULE_NAME);
    
    status = mutex_lock_interruptible(&gsys->sem);
    if(status){
        goto error;
    }
    
    // our buffer is to small!
    if(len > gsys->buffer_size){
        status = -ENOMEM;
        goto err_unlock;
    }

    // grab data from userspace and put it in the dma buffer
    if(copy_to_user(buf, gsys->buffer, len)){
        goto err_unlock;
    }
    
    // update offset since we read 'len' num of bytes
    *off += len;
    // we must return the number of bytes processed
    status = len;

err_unlock:
    mutex_unlock(&gsys->sem);
error:
    return status;
}

/*
 * The /dev/gcore file has been opened and now is awaiting data
 * to be written to it.
 *
 * subcore should be in a paused state and blocking on the
 * axis valid signal. Once data is writen, kernel will
 * issue a tlast which will trigger subore to go to wait
 * confit state.
 *
 * @file: pointer to the file structure
 * @buf: pointer to the bitstream or dut_data location
 * @len: number of bytes to be written
 * @off: pointer to the offset value
 *
 */
static ssize_t gcore_write(struct file *file, const char __user * buf,
	size_t len, loff_t * off)
{
    u32 status = 0;
    struct gcore_system *gsys = file->private_data;

    printk(KERN_DEBUG "%s file: write()\n", MODULE_NAME);
    
	status = mutex_lock_interruptible(&gsys->sem);
    if(status){
        goto error;
    }
    
    // our buffer is to small!
    if(len > gsys->buffer_size){
        status = -ENOMEM;
        goto err_unlock;
    }

    // grab data from userspace and put it in the dma buffer
    if(copy_from_user(gsys->buffer, buf, len)){
        goto err_unlock;
    }
    
    // update offset since we read 'len' num of bytes
    *off += len;
    // we must return the number of bytes processed
    status = len;

err_unlock:
    mutex_unlock(&gsys->sem);
error:
    return status;
}

/*
 * gcore_mmap
 *
 * Maps the dma buffer to userspace so it can be written to directly
 * before performing a dma operation. 
 *
 * Used for either writing bitstream and writing/reading test vector
 * information.
 *
 */
static int gcore_mmap(struct file *file, struct vm_area_struct *vma)
{
	int result;
	unsigned long requested_size;
    struct gcore_system *gsys = file->private_data;
	
    requested_size = vma->vm_end - vma->vm_start;

	printk(KERN_DEBUG "%s: file: mmap()\n", MODULE_NAME);
	printk(KERN_DEBUG "%s: file: dma buffer size: %d, mmap size requested: %lu\n",
	       MODULE_NAME, DMA_SIZE, requested_size);

	if (requested_size > DMA_SIZE) {
		printk(KERN_ERR "%s: %d reserved != %lu requested)\n",
		       MODULE_NAME, DMA_SIZE, requested_size);

		return -EAGAIN;
	}

    if (vma->vm_pgoff == 0) {
	    printk(KERN_DEBUG "%s: file: mmap using dma_mmap_coherent\n", 
            MODULE_NAME);
        result = dma_mmap_coherent(NULL, vma, gsys->buffer, 
            gsys->dma_handle, requested_size);
    } else {
	    printk(KERN_DEBUG "%s: file: mmap using remap_pfn_range\n", 
                MODULE_NAME);
        vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
        vma->vm_flags |= VM_IO;
        result = remap_pfn_range(vma, vma->vm_start, 
            PFN_DOWN(virt_to_phys(phys_to_virt(gsys->dma_handle))) + vma->vm_pgoff,
            requested_size, vma->vm_page_prot);
    }

	if (result < 0 ) {
		printk(KERN_ERR
		       "%s: error in calling remap_pfn_range: returned %d\n",
		       MODULE_NAME, result);

		return -EAGAIN;
	}

	return 0;
}

static long gcore_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    u32 rc = 0;
    u32 reg = 0;
    struct gcore_system *gsys = file->private_data;
    struct gcore_dev *gdev = gsys->gdev;
	long ret = 0;
	struct gcore_registers registers;
	struct gcore_userdev userdev;
	struct gcore_cfg gcfg;
	struct gcore_chan_cfg chan_cfg;
	struct gcore_transfer trans;
	struct gcore_ctrl_packet ctrl_packet;
    u32 chan;

	switch (cmd) {
	case GCORE_REGS_READ:
		printk(KERN_DEBUG "%s ioctl: GCORE_REGS_READ\n", MODULE_NAME);

		if (copy_from_user((void *)&registers, (const void __user *)arg, sizeof(struct gcore_registers)))
			return -EFAULT;

		gcore_get_reg_info(gsys, &registers);

		if (copy_to_user((struct gcore_registers *)arg, &registers, sizeof(struct gcore_registers)))
			return -EFAULT;

		break;
    case GCORE_USERDEVS_READ:
		printk(KERN_DEBUG "%s ioctl: GCORE_USERDEVS_READ\n", MODULE_NAME);

		if (copy_from_user((void *)&userdev, (const void __user *)arg, sizeof(struct gcore_userdev)))
			return -EFAULT;

		gcore_get_userdev_info(gsys, &userdev);

		if (copy_to_user((struct gcore_userdev *)arg, &userdev, sizeof(struct gcore_userdev)))
			return -EFAULT;

		break;
    case GCORE_SUBCORE_LOAD:
		printk(KERN_DEBUG "%s ioctl: GCORE_SUBCORE_LOAD\n", MODULE_NAME);

		if(copy_from_user((void *)&gcfg, (const void __user *)arg, sizeof(struct gcore_cfg)))
			return -EFAULT;
        
        gsys->subcore_state = gcfg.subcore_state;
        gsys->artix_select = gcfg.artix_select;
        
        ret = mutex_lock_interruptible(&gsys->sem);
        if (ret){
            goto err_unlock;
        }
        ret = subcore_load(gsys);
        mutex_unlock(&gsys->sem);
        break;
    case GCORE_SUBCORE_RUN:
		printk(KERN_DEBUG "%s ioctl: GCORE_SUBCORE_RUN\n", MODULE_NAME);
        ret = mutex_lock_interruptible(&gsys->sem);
        if (ret){
            goto err_unlock;
        }
        ret = subcore_run(gsys);
        mutex_unlock(&gsys->sem);
        break;
    case GCORE_SUBCORE_IDLE:
		printk(KERN_DEBUG "%s ioctl: GCORE_SUBCORE_IDLE\n", MODULE_NAME);
        
        ret = mutex_lock_interruptible(&gsys->sem);
        if (ret){
            goto err_unlock;
        }
        // wait for idle
        ret = subcore_idle(gsys);
        mutex_unlock(&gsys->sem);

		break;
    case GCORE_SUBCORE_STATE:
		printk(KERN_DEBUG "%s ioctl: GCORE_SUBCORE_STATE\n", MODULE_NAME);
        ret = mutex_lock_interruptible(&gsys->sem);
        if (ret){
            goto err_unlock;
        }
        gdev->control_reg = reg_read(gdev, gdev->control_offset); 
        gdev->control_reg |= GCORE_CONTROL_STATE_MASK;    
        reg_write(gdev, gdev->control_offset, gdev->control_reg);
        rc = readl_poll_timeout (
            gdev->regs+gdev->control_offset, 
            reg, 
            !(reg & GCORE_CONTROL_STATE_MASK), 
            100, // delay us
            200000 // timout us
        );
        if(rc){
		    printk(KERN_DEBUG "%s ioctl: control reg state bit failed to de-assert\n", MODULE_NAME);
            ret = -ETIMEDOUT;
            goto err_unlock;
        }
        mutex_unlock(&gsys->sem);       
        break;
    case GCORE_SUBCORE_RESET:
		printk(KERN_DEBUG "%s ioctl: GCORE_SUBCORE_RESET\n", MODULE_NAME);
        
        ret = mutex_lock_interruptible(&gsys->sem);
        if (ret){
            goto err_unlock;
        }
        // will block to wait for 
        ret = subcore_reset(gsys);
        mutex_unlock(&gsys->sem);

		break;
    case GCORE_ARTIX_SYNC:

        // grab packet from user space
        if (copy_from_user((void *)&ctrl_packet, (const void __user *)arg, sizeof(struct gcore_ctrl_packet)))
			return -EFAULT;

		printk(KERN_DEBUG "%s ioctl: GCORE_SYNC\n", MODULE_NAME);

        ret = artix_sync(gsys, &ctrl_packet);
        if (ret){
            goto err_unlock;
        }
        break;
    /*
     * Subcore must be in ctrl_write state. Write to 
     * rank_sel, addr and data. Call this ioctl to pass
     * data to subcore.
     * 
     */
    case GCORE_CTRL_WRITE:
        
        // grab packet from user space
        if (copy_from_user((void *)&ctrl_packet, (const void __user *)arg, sizeof(struct gcore_ctrl_packet)))
			return -EFAULT;
        
		printk(KERN_DEBUG "%s ioctl: GCORE_CTRL_WRITE: addr: 0x%08X data: 0x%08X\n", 
            MODULE_NAME, ctrl_packet.addr, ctrl_packet.data);
        
        // write packet to subcore
        ret = subcore_ctrl_write(gsys, &ctrl_packet);
        if (ret){
            goto err_unlock;
        }
        break;
    /*
     * Subcore must be in ctrl_read state. Call this ioctl
     * to read into rank_sel, addr and data.
     */
    case GCORE_CTRL_READ:
		printk(KERN_DEBUG "%s ioctl: GCORE_CTRL_READ\n", MODULE_NAME);
       
        // read packet from subcore
        ret = subcore_ctrl_read(gsys, &ctrl_packet);
        if (ret){
            goto err_unlock;
        }

        // copy ctrl packet back to user
        if (copy_to_user((struct gcore_ctrl_packet *)arg, &ctrl_packet, sizeof(struct gcore_ctrl_packet)))
            return -EFAULT;

        break;
    case GCORE_DMA_CONFIG:
		printk(KERN_DEBUG "%s ioctl: GCORE_DMA_CONFIG\n", MODULE_NAME);

		if(copy_from_user((void *)&chan_cfg, (const void __user *)arg, sizeof(struct gcore_chan_cfg)))
			return -EFAULT;

		gcore_chan_config(gsys, &chan_cfg);
		break;
	case GCORE_DMA_PREP:
		printk(KERN_DEBUG "%s ioctl: GCORE_DMA_PREP\n", MODULE_NAME);

		if (copy_from_user((void *)&chan_cfg, (const void __user *)arg, sizeof(struct gcore_chan_cfg)))
			return -EFAULT;

		ret = (long)gcore_chan_prep(gsys, &chan_cfg);

		if (copy_to_user((struct gcore_chan_cfg *)arg, &chan_cfg, sizeof(struct gcore_chan_cfg)))
			return -EFAULT;

		break;
	case GCORE_DMA_START:
		printk(KERN_DEBUG "%s ioctl: GCORE_DMA_START\n",
		       MODULE_NAME);

		if (copy_from_user((void *)&trans, (const void __user *)arg, sizeof(struct gcore_transfer)))
			return -EFAULT;

		ret = (long)gcore_dma_start(&trans, true);
		break;
	case GCORE_DMA_STOP:
		printk(KERN_DEBUG "%s ioctl: GCORE_DMA_STOP\n", MODULE_NAME);

		if (copy_from_user((void *)&chan,
				   (const void __user *)arg, sizeof(u32)))
			return -EFAULT;

		gcore_dma_stop((struct dma_chan *)chan);
		break;
	default:
		break;
	}
    
	return ret;

err_unlock:
    mutex_unlock(&gsys->sem);
    return ret;
}


/*
 * =======================================================
 * DMA functions
 *
 */


static void gcore_sync_callback(void *completion)
{
	complete(completion);
}

static void gcore_chan_config(struct gcore_system *gsys, struct gcore_chan_cfg *chan_cfg)
{
	struct dma_chan *chan = (struct dma_chan *)chan_cfg->chan;
	struct dma_slave_config config;
    
    switch (chan_cfg->dir) {
	case GCORE_MEM_TO_DEV:
		config.src_addr = gsys->dma_handle + chan_cfg->buf_offset;
        config.dst_addr = gsys->dma_handle + chan_cfg->buf_offset;
		break;
	case GCORE_DEV_TO_MEM:
		config.src_addr = gsys->dma_handle + chan_cfg->buf_offset;
        config.dst_addr = gsys->dma_handle + chan_cfg->buf_offset;
        break;
	default:
		config.src_addr = gsys->dma_handle + chan_cfg->buf_offset;
        config.dst_addr = gsys->dma_handle + chan_cfg->buf_offset;
        break;
	}
    
    config.src_addr_width = (u32)8; 
    config.dst_addr_width = (u32)8;
    config.src_maxburst = (u32)16;
    config.dst_maxburst = (u32)16;
    config.device_fc = 0;
    config.slave_id = 0;

	if (chan) {
        dmaengine_slave_config(chan, &config);
	}
}

/*
 * Prepares the buffer for a given channel and 
 * submits the task to the dma queue.
 *
 */
static int gcore_chan_prep(struct gcore_system *gsys,  struct gcore_chan_cfg *chan_cfg)
{
	int ret = 0;
	struct dma_chan *chan;
	dma_addr_t dma_buf;
	size_t dma_len;
	enum dma_transfer_direction dir;
	enum dma_ctrl_flags flags;
	struct dma_async_tx_descriptor *chan_desc;
	struct completion *cmp;
	dma_cookie_t cookie;

	chan = (struct dma_chan *)chan_cfg->chan;
	cmp = (struct completion *)chan_cfg->completion;
	dma_buf = gsys->dma_handle + chan_cfg->buf_offset;
	dma_len = chan_cfg->buf_size;
	dir = gcore_to_dma_direction(chan_cfg->dir);

	flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;

	chan_desc = dmaengine_prep_slave_single(chan, dma_buf, dma_len, dir, flags);

	if (!chan_desc) {
		printk(KERN_ERR "%s: dmaengine_prep_slave_single error\n", MODULE_NAME);
		ret = -1;
		chan_cfg->cookie = (u32)-EBUSY;
	} else {
		chan_desc->callback = gcore_sync_callback;
		chan_desc->callback_param = cmp;

		// set the prepared descriptor to be executed by the engine
		cookie = chan_desc->tx_submit(chan_desc);
		if (dma_submit_error(cookie)) {
			printk(KERN_ERR "%s: dma tx_submit error\n", MODULE_NAME);
			ret = -1;
		}

		chan_cfg->cookie = (u32)cookie;
	}

	return ret;
}

/*
 * Starts the DMA transfer
 *
 */
static int gcore_dma_start(struct gcore_transfer *trans, bool measure_time)
{
	int ret = 0;
	unsigned long tmo = msecs_to_jiffies(trans->wait_time_msecs);
	enum dma_status status;
	struct dma_chan *chan;
	struct completion *cmp;
	dma_cookie_t cookie;
	struct timeval ti, tf;

	chan = (struct dma_chan *)trans->chan;
	cmp = (struct completion *)trans->completion;
	cookie = (dma_cookie_t)trans->cookie;

	init_completion(cmp);
	
    if(measure_time){
        do_gettimeofday(&ti);
    }
	
    dma_async_issue_pending(chan);

	if (trans->wait) {
		tmo = wait_for_completion_timeout(cmp, tmo);
		status = dma_async_is_tx_complete(chan, cookie, NULL, NULL);
		if (0 == tmo) {
			printk(KERN_ERR "%s: dma timed out\n", MODULE_NAME);
			ret = -1;
		} else if (status != DMA_COMPLETE) {
			printk(KERN_DEBUG
			       "%s transfer: returned completion callback status of: \'%s\'\n",
			       MODULE_NAME,
			       status == DMA_ERROR ? "error" : "in progress");
			ret = -1;
		}
	}

    if(measure_time){
        do_gettimeofday(&tf);
        trans->duration_usecs = (uint32_t)(tf.tv_usec - ti.tv_usec);
        printk(KERN_DEBUG "%s: dma sent %d bytes in time [us]: %ld at speed MB/s: %ld\n", MODULE_NAME, 
                trans->buf_size, trans->duration_usecs, trans->buf_size / trans->duration_usecs);
    }

	return ret;
}

/*
 * Aborts all transfers on a channel.
 *
 */
static void gcore_dma_stop(struct dma_chan *chan)
{
	if (chan) {
        // this func is available kernel 4.5 and later
		//dmaengine_terminate_sync(chan);

        // for now we use this deprecated crap
        dmaengine_terminate_all(chan);
	}
}

/*
 * =======================================================
 * Driver setup
 *
 */

/*
 * Creates a new gcore device with the channels gotten using the names
 * in the devicetree.
 *
 */
static int gcore_create_device(struct platform_device *pdev, struct dma_chan *tx_chan, struct dma_chan *rx_chan)
{
    struct gcore_dev *gdev;
    struct resource *res;
	struct completion *tx_cmp, *rx_cmp;
    
    struct gcore_system *gsys = platform_get_drvdata(pdev);
    if(!gsys){
        return -1;        
    }

	tx_cmp = (struct completion *)kzalloc(sizeof(struct completion), GFP_KERNEL);
    if(!tx_cmp){
        return -ENOMEM;
    }

	rx_cmp = (struct completion *)kzalloc(sizeof(struct completion), GFP_KERNEL);
    if(!rx_cmp){
        return -ENOMEM;
    }
	
    gdev = (struct gcore_dev *)kzalloc(sizeof(struct gcore_dev), GFP_KERNEL);
    if(!gdev){
        return -ENOMEM;
    }

    // map the registers
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    gdev->regs = devm_ioremap_resource(&pdev->dev, res);
    if(IS_ERR(gdev->regs)){
        return PTR_ERR(gdev->regs);
    }
    
    gdev->res = res;
    gdev->tx_chan = tx_chan;
	gdev->tx_cmp = tx_cmp;
	gdev->rx_chan = rx_chan;
	gdev->rx_cmp = rx_cmp;
    gdev->control_offset = GCORE_CONTROL_REGISTER; 
    gdev->status_offset = GCORE_STATUS_REGISTER; 
    gdev->addr_offset = GCORE_ADDR_REGISTER; 
    gdev->data_offset = GCORE_DATA_REGISTER; 
    gdev->a1_status_offset = GCORE_A1_STATUS_REGISTER; 
    gdev->a2_status_offset = GCORE_A2_STATUS_REGISTER; 
    
    // read the initial register values from gemini_core
    gdev->control_reg = reg_read(gdev, gdev->control_offset); 
    gdev->status_reg = reg_read(gdev, gdev->status_offset); 
    gdev->addr_reg = reg_read(gdev, gdev->addr_offset); 
    gdev->data_reg = reg_read(gdev, gdev->data_offset);
    
    // save the gcore device to the system instance.
    gsys->gdev = gdev;
    
    
    return 0;
}

static int gcore_probe(struct platform_device *pdev);
static int gcore_remove(struct platform_device *pdev);

static const struct of_device_id gemini_core_of_ids[] = {
	{ .compatible = "xlnx,gemini-core-1.0",},
	{}
};

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = gcore_open,
	.release = gcore_release,
	.read = gcore_read,
	.write = gcore_write,
	.mmap = gcore_mmap,
	.unlocked_ioctl = gcore_ioctl,
};

static struct platform_driver gemini_core_driver = {
	.driver = {
		.name = "gcore",
		.owner = THIS_MODULE,
		.of_match_table = gemini_core_of_ids,
	},
    .probe = gcore_probe,
	.remove = gcore_remove,
};

static int gcore_probe(struct platform_device *pdev)
{
    struct gcore_system *gsys;
	dma_cap_mask_t mask;
	struct dma_chan *tx_chan, *rx_chan;
    int err;
    
    printk(KERN_DEBUG "%s: initializing...\n", MODULE_NAME);
    
    gsys = (struct gcore_system *)kzalloc(sizeof(struct gcore_system), GFP_KERNEL);
    if(!gsys){
        return -ENOMEM;
    }

    // gcore system init
    gsys->subcore_state = SUBCORE_IDLE; 
    gsys->artix_select = ARTIX_SELECT_NONE;
    gsys->is_busy = 0;
    spin_lock_init(&gsys->lock);
	mutex_init(&gsys->sem);
    
    // save gcore system in the kernel
	platform_set_drvdata(pdev, gsys);
    
    // create the chrdev
	if (alloc_chrdev_region(&gsys->chrdev_num, 0, 1, MODULE_NAME) < 0) {
		return -1;
	}
    
    // create the class
	if ((gsys->class = class_create(THIS_MODULE, MODULE_NAME)) == NULL) {
	    printk(KERN_ERR "%s: init failed to create class\n", MODULE_NAME);
		unregister_chrdev_region(gsys->chrdev_num, 1);
		return -1;
	}
    
    // create the device
	if ((gsys->dev = device_create(gsys->class, NULL, gsys->chrdev_num, NULL, MODULE_NAME)) == NULL) {
	    printk(KERN_ERR "%s: init failed to create device\n", MODULE_NAME);
		class_destroy(gsys->class);
		unregister_chrdev_region(gsys->chrdev_num, 1);
		return -1;
	}

    // init the chrdev
	cdev_init(&gsys->chrdev, &fops);
	if (cdev_add(&gsys->chrdev, gsys->chrdev_num, 1) == -1) {
	    printk(KERN_ERR "%s: init failed to cdev_add\n", MODULE_NAME);
		device_destroy(gsys->class, gsys->chrdev_num);
		class_destroy(gsys->class);
		unregister_chrdev_region(gsys->chrdev_num, 1);
		return -1;
	}
    
    // allocate our buffer for dma transfers 
	gsys->buffer = dma_zalloc_coherent(NULL, DMA_SIZE, &(gsys->dma_handle), GFP_KERNEL);
    gsys->buffer_size = DMA_SIZE;

	if (!(gsys->buffer)) {
		printk(KERN_ERR "%s: allocating dma memory failed\n", MODULE_NAME);
		return -ENOMEM;
	}


	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE | DMA_PRIVATE, mask);
	
    tx_chan = dma_request_slave_channel(&pdev->dev, "axidma0");
    if (!tx_chan) {
        printk(KERN_ERR "%s probe: no tx 'axidma0' channel found in dts.\n", MODULE_NAME);
        return -1;
    }

    rx_chan = dma_request_slave_channel(&pdev->dev, "axidma1");
    if (!rx_chan) {
        printk(KERN_ERR "%s probe: no rx 'axidma1' channel found in dts.\n", MODULE_NAME);
        goto free_tx;
    }

    err = gcore_create_device(pdev, tx_chan, rx_chan);
    if (err) {
        printk(KERN_ERR "%s probe: unable to create gcore device\n", MODULE_NAME);
        goto free_rx;
    }
	
    printk(KERN_DEBUG "%s: gemini core subsystem initialized\n", MODULE_NAME);
    
    return 0;

free_rx:
    dma_release_channel(rx_chan);
free_tx:
    dma_release_channel(tx_chan);
    
    if(gsys) {
        kfree((struct gemini_system *)gsys);
    }

    return err;
}

static int gcore_remove(struct platform_device *pdev)
{
    struct gcore_system *gsys = platform_get_drvdata(pdev);
    struct gcore_dev *gdev = gsys->gdev;
    
    /* device destructor */
	cdev_del(&gsys->chrdev);
	device_destroy(gsys->class, gsys->chrdev_num);
	class_destroy(gsys->class);
	unregister_chrdev_region(gsys->chrdev_num, 1);
	printk(KERN_DEBUG "%s exit: unregistered\n", MODULE_NAME);
    
    // gdev better exist
    if(gdev) {
        if(gdev->res){
            devm_ioremap_release(&pdev->dev, gdev->res);     
        }
        if (gdev->tx_chan){
            dma_release_channel((struct dma_chan *)gdev->tx_chan);
        }

        if (gdev->tx_cmp){
            kfree((struct completion *)gdev->tx_cmp);
        }

        if (gdev->rx_chan){
            dma_release_channel((struct dma_chan *)gdev->rx_chan);
        }

        if (gdev->rx_cmp){
            kfree((struct completion *)gdev->rx_cmp);
        }
    }
    
    /* free mmap area */
	if (gsys->buffer) {
		dma_free_coherent(NULL, DMA_SIZE, gsys->buffer, gsys->dma_handle);
	}
    
    // finally free the gcore system struct
    if(gsys) {
        kfree((struct gemini_system *)gsys);
    }

    return 0;
}

static int __init gcore_init(void)
{
	return platform_driver_register(&gemini_core_driver);
}
late_initcall(gcore_init);

static void __exit gcore_exit(void)
{
	platform_driver_unregister(&gemini_core_driver);
}
module_exit(gcore_exit);

MODULE_AUTHOR("Julian Lupu");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("GEMINI CORE subsystem module");


