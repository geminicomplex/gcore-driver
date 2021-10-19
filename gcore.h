/*
 * gcore.h
 *
 * Copyright (c) 2015-2021 Gemini Complex Corporation. All rights reserved.
 *
 */
#ifndef GCORE_H
#define GCORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/types.h>
#include <asm/ioctl.h>

#define MODULE_NAME	"gcore"

#define u32 uint32_t

// The max transfer length for the dma register is 
// (2^23)-1 or 8388607 bytes. Use a size that's divisible
// by the burst size of 128 bytes and 4 byte aligned.
// linux uses 4k pages so must be at least less than page
#define DMA_SIZE (8384512)

// ioctl methods
#define GCORE_IOCTL_BASE	    'W'
#define GCORE_REGS_READ	        _IO(GCORE_IOCTL_BASE, 0)
#define GCORE_USERDEVS_READ	    _IO(GCORE_IOCTL_BASE, 1)
#define GCORE_SUBCORE_LOAD      _IO(GCORE_IOCTL_BASE, 2)
#define GCORE_SUBCORE_RUN       _IO(GCORE_IOCTL_BASE, 3)
#define GCORE_SUBCORE_IDLE  	_IO(GCORE_IOCTL_BASE, 4)
#define GCORE_SUBCORE_STATE     _IO(GCORE_IOCTL_BASE, 5)
#define GCORE_SUBCORE_RESET	    _IO(GCORE_IOCTL_BASE, 6)
#define GCORE_ARTIX_SYNC        _IO(GCORE_IOCTL_BASE, 7)
#define GCORE_CTRL_WRITE        _IO(GCORE_IOCTL_BASE, 8)
#define GCORE_CTRL_READ     	_IO(GCORE_IOCTL_BASE, 9)
#define GCORE_DMA_CONFIG	    _IO(GCORE_IOCTL_BASE, 10)
#define GCORE_DMA_PREP		    _IO(GCORE_IOCTL_BASE, 11)
#define GCORE_DMA_START	        _IO(GCORE_IOCTL_BASE, 12)
#define GCORE_DMA_STOP	        _IO(GCORE_IOCTL_BASE, 13)

// control reg
#define GCORE_CONTROL_RUN_MASK          (1 << 0)
#define GCORE_CONTROL_RESET_MASK        (1 << 1) 
#define GCORE_CONTROL_LOAD_MASK         (1 << 2)
#define GCORE_CONTROL_READ_MASK         (1 << 3)
#define GCORE_CONTROL_WRITE_MASK        (1 << 4)
#define GCORE_CONTROL_STATE_MASK        (1 << 5)
#define GCORE_CONTROL_RANK_SELECT_MASK  (1 << 6)
#define GCORE_CONTROL_SYNC_MASK         (1 << 7)

// status reg
#define GCORE_STATUS_IDLE_MASK         (1 << 0)
#define GCORE_STATUS_PAUSED_MASK       (1 << 1)
#define GCORE_STATUS_RUNNING_MASK      (1 << 2)
#define GCORE_STATUS_SYNCED_MASK       (1 << 3)
#define GCORE_STATUS_RESET_MASK        (1 << 4)
#define GCORE_STATUS_CONFIG_ERROR_MASK (1 << 28)
#define GCORE_STATUS_DONE_ERROR_MASK   (1 << 29)
#define GCORE_STATUS_SETUP_ERROR_MASK  (1 << 30)
#define GCORE_STATUS_INIT_ERROR_MASK   (1 << 31)

// mask all except error
#define GCORE_STATUS_ERROR_CODE_MASK 0xF0000000

// a1/a2 agent status reg
#define GCORE_AGENT_MIG_MMCM_ERROR_MASK  (1 << 24)
#define GCORE_AGENT_MIG_CALIB_ERROR_MASK (1 << 23)
#define GCORE_AGENT_MIG_RESET_ERROR_MASK (1 << 22)
#define GCORE_AGENT_DCLK_PLL_ERROR_MASK  (1 << 21)
#define GCORE_AGENT_DDR3_TEMP_ERROR_MASK (1 << 20)
#define GCORE_AGENT_STARTUP_DONE_MASK    (1 << 19)
#define GCORE_AGENT_DONE_MASK            (1 << 18)
#define GCORE_AGENT_ERROR_MASK           (1 << 17)
#define GCORE_AGENT_GVPU_FAILED_MASK     (1 << 16)
#define GCORE_AGENT_GVPU_STAGE_MASK      (0x0000F000)
#define GCORE_AGENT_MEMCORE_STATE_MASK   (0x00000F00)
#define GCORE_AGENT_GVPU_STATE_MASK      (0x000000F0)
#define GCORE_AGENT_STATE_MASK           (0x0000000F)

/*
 * Subcore artix select.
 */
#define GCORE_ARTIX_SELECT_NONE 0x00000000
#define GCORE_ARTIX_SELECT_A1   0x40000000
#define GCORE_ARTIX_SELECT_A2   0x80000000

/*
 * Subcore FSM states.
 */
#define GCORE_SUBCORE_MODE_IDLE          0x00000000
#define GCORE_SUBCORE_MODE_PAUSED        0x00000001
#define GCORE_SUBCORE_MODE_CONFIG_SETUP  0x00000002
#define GCORE_SUBCORE_MODE_CONFIG_LOAD   0x00000003
#define GCORE_SUBCORE_MODE_CONFIG_WAIT   0x00000004
#define GCORE_SUBCORE_MODE_AGENT_STARTUP 0x00000005
#define GCORE_SUBCORE_MODE_SETUP_BURST   0x00000006
#define GCORE_SUBCORE_MODE_SETUP_WRITE   0x00000007
#define GCORE_SUBCORE_MODE_SETUP_READ    0x00000008
#define GCORE_SUBCORE_MODE_SETUP_CLEANUP 0x00000009
#define GCORE_SUBCORE_MODE_CTRL_WRITE    0x0000000A
#define GCORE_SUBCORE_MODE_CTRL_READ     0x0000000B
#define GCORE_SUBCORE_MODE_CTRL_RUN      0x0000000C
#define GCORE_SUBCORE_MODE_DMA_WRITE     0x0000000D
#define GCORE_SUBCORE_MODE_DMA_READ      0x0000000E
#define GCORE_SUBCORE_MODE_GPIO_DNA      0x0000000F

/*
 * Subcore gpio dna commands
 */
#define SUBCORE_GPIO_DNA_CMD_NONE          (0x00000000)
#define SUBCORE_GPIO_DNA_CMD_BOOT_DONE     (0x00000001)
#define SUBCORE_GPIO_DNA_CMD_SET_RED_LED   (0x00000002)
#define SUBCORE_GPIO_DNA_CMD_SET_GREEN_LED (0x00000003)
#define SUBCORE_GPIO_DNA_CMD_GET_DNA       (0x00000004)

/*
 * DMA direction
 *
 */
enum gcore_direction {
    GCORE_MEM_TO_DEV,
    GCORE_DEV_TO_MEM,
    GCORE_TRANS_NONE,
};

/*
 * gcore userdev structure
 * @tx_chan: transmit dma channel
 * @tx_cmp: tx channel completion struct
 * @rx_chan: receive dma channel
 * @rx_cmp: rx channel completion struct
 *
 */
struct gcore_userdev {
    u32 tx_chan;	
    u32 tx_cmp;	
    u32 rx_chan;	
    u32 rx_cmp;	
};

/*
 * subcore states
 *
 * Note: never place subcore directly into config load
 * or wait directly. Always call setup.
 *
 */
enum subcore_states {
    SUBCORE_IDLE    = 0x00000000,
    SUBCORE_PAUSED  = 0x00000001,
    CONFIG_SETUP    = 0x00000002,
    CONFIG_LOAD     = 0x00000003,
    CONFIG_WAIT     = 0x00000004,
    AGENT_STARTUP   = 0x00000005,
    SETUP_BURST     = 0x00000006,
    SETUP_WRITE     = 0x00000007,
    SETUP_READ      = 0x00000008,
    SETUP_CLEANUP   = 0x00000009,
    CTRL_WRITE      = 0x0000000A,
    CTRL_READ       = 0x0000000B,
    CTRL_RUN        = 0x0000000C,
    DMA_WRITE       = 0x0000000D,
    DMA_READ        = 0x0000000E,
    GPIO_DNA        = 0x0000000F
};

/*
 * artix_core agent states
 */
enum agent_states {
    AGENT_INIT   = 0x00000000,
    AGENT_IDLE   = 0x00000001,
    AGENT_PAUSED = 0x00000002,
    STATUS       = 0x00000003,
    BURST_LOAD   = 0x00000004,
    GVPU_LOAD    = 0x00000005,
    GVPU_RUN     = 0x00000006,
    GVPU_WRITE   = 0x00000007,
    GVPU_READ    = 0x00000008,
    GVPU_STATUS  = 0x00000009,
    GVPU_RESET   = 0x0000000A
};

/*
 * artix_core gvpu states
 */

enum gvpu_states {
    GVPU_IDLE       = 0x00000000,
    GVPU_PAUSED     = 0x00000001,
    MEM_BURST       = 0x00000002,
    MEM_LOAD        = 0x00000003,
    MEM_RUN         = 0x00000004,
    MEM_WRITE       = 0x00000005,
    MEM_READ        = 0x00000006,
    MEM_TEST        = 0x00000007,
    TEST_INIT       = 0x00000008,
    TEST_SETUP      = 0x00000009,
    TEST_RUN        = 0x0000000A,
    TEST_FAIL_PINS  = 0x0000000B,
    TEST_CLEANUP    = 0x0000000C
};

/*
 * artix_core memcore states
 */

enum memcore_states {
    MEMCORE_IDLE        = 0x00000000,
    MEMCORE_PAUSED      = 0x00000001,
    MEMCORE_SETUP_BURST = 0x00000002,
    MEMCORE_WRITE_BURST = 0x00000003,
    MEMCORE_READ_BURST  = 0x00000004
};

/*
 * subcore artix unit select
 *
 * NONE: initial state 
 * A1: select the top artix1 unit
 * A2: select the bot artix2 unit
 *
 */
enum artix_selects {
    ARTIX_SELECT_NONE,
    ARTIX_SELECT_A1,
    ARTIX_SELECT_A2,
    ARTIX_SELECT_BOTH
};

/*
 * gcore cfg structure
 *
 * Provides interface to userspace to configure this biznatch.
 *
 */
struct gcore_cfg {
    enum subcore_states subcore_state;
    enum artix_selects artix_select;
};

/*
 * gcore registers structure
 *
 * Used to hold values of the 4 registers.
 *
 */
struct gcore_registers {
    u32 control;	
    u32 status;	
    u32 addr;	
    u32 data;	
    u32 a1_status;
    u32 a2_status;
};

/*
 * Used to write and read to ctrl_axi.
 */
struct gcore_ctrl_packet {
    u32 rank_select;
    u32 addr;
    u32 data;
};

/*
 * gcore dma channel config structure
 *
 * Used to configure a dma channel.
 *
 */
struct gcore_chan_cfg {
    u32 chan;
    enum gcore_direction dir;
    u32 buf_offset;
    u32 buf_size;
    u32 completion;	
    u32 cookie;
};

struct gcore_transfer {
    u32 chan;
    u32 completion;	
    u32 cookie;
    u32 wait;	/* true/false */
    u32 wait_time_msecs;
    u32 buf_size;
    u32 duration_usecs;
};

#ifdef __cplusplus
}
#endif
#endif	/* GCORE_H */
