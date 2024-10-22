/******************************************************************************
  @file    mp-ctl.h
  @brief   Header file for communication and actions for PerfLock

  DESCRIPTION

  ---------------------------------------------------------------------------
  Copyright (c) 2011-2015,2017 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.
  ---------------------------------------------------------------------------
******************************************************************************/

#ifndef __MP_CTL_H__
#define __MP_CTL_H__

#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/*newer resources
  supported from v3.0 onwards*/
#define MAX_RESOURCES_PER_REQUEST 32
#define MAX_ARGS_PER_REQUEST (MAX_RESOURCES_PER_REQUEST*2)
#define MAX_MSG_APP_NAME_LEN 128
#define MIN_CLUSTERS 2

#define MPCTL_VERSION      3
#define OLD_MPCTL_VERSION  2
#define PROP_VAL_LENGTH     128

#define BOOT_CMPLT_PROP "vendor.post_boot.parsed"

/*All resources are categorized into one of the following types
  based their sysfs node manipulations.*/
enum {
    /*Resources which updates a single normal node can be categorized as SINGLE_NODE.*/
    SINGLE_NODE,
    /* All the resources whose sysfs node paths can go offline depending on the core
    status(offline/online) and also updating a single core automatically updates all the
    cores of that cluster, can be assigned as type INTERACTIVE_NODE.
    Example -  schedutil_hispeed_freq, we find the first online core in the requested cluster
    and update it with requested vlaue. As a result all the cores in that cluster automatically
    gets updated with the same value for schedutil_hispeed_freq.*/
    INTERACTIVE_NODE,
    /*The resources for which we manually update all the cores of both clusters with a same
    value basing on the requested value, can be categorized as UPDATE_ALL_CORES.
    Example - sched_prefer_idle, when we get a acquire request call for this resource, we
    update the given Sysfs node for all the cores of both clusters with a same value.*/
    UPDATE_ALL_CORES,
    /*A special type of UPDATE_ALL_CORES, where we manually update cores of both clusters with
    different values basing on the requested value, can be assigned as UPDATE_CORES_PER_CLUSTER.
    Example - sched_mostly_idle_freq, For an acquire request call we update the given Sysfs node for
    all the cores but with different values for different clusters.*/
    UPDATE_CORES_PER_CLUSTER,
    /*All the resources which provide an option to select a particular core to get updated
    can be assigned as type SELECT_CORE_TO_UPDATE
    Example - sched_static_cpu_pwr_cost, for this resource in the Opcode we can provide the
    exact core number which needs to be updated.*/
    SELECT_CORE_TO_UPDATE,
    /*All other resources which update multiple nodes or needs special treament.*/
    SPECIAL_NODE,
    /*Adding memlat as a node type for handling memlat resource request*/
    MEM_LAT_NODE,
};

enum {
    MPCTL_CMD_PERFLOCKACQ = 2,
    MPCTL_CMD_PERFLOCKREL = 3,
    MPCTL_CMD_PERFLOCKPOLL = 4,
    MPCTL_CMD_PERFLOCKRESET = 5,
    MPCTL_CMD_PERFLOCK_PROFILE = 6,
    MPCTL_CMD_PERFLOCK_RESTORE_GOV_PARAMS = 7,
    MPCTL_CMD_PERFLOCKHINTACQ = 8,
};

enum {
    VENDOR_HINT_START = 0x00001000,
    //activity trigger hints
    VENDOR_ACT_TRIGGER_HINT_BEGIN = 0x00001001,
    VENDOR_HINT_ACTIVITY_START = 0x00001001,
    VENDOR_HINT_ACTIVITY_STOP = 0x00001002,
    VENDOR_HINT_ACTIVITY_RESUME = 0x00001003,
    VENDOR_HINT_ACTIVITY_PAUSE = 0x00001004,
    VENDOR_HINT_PROCESS_START = 0x00001005,
    VENDOR_HINT_NET_OPS = 0x00001006,
    VENDOR_ACT_TRIGGER_HINT_END = 0x0000103F,

    //perf events
    VENDOR_PERF_EVENT_BEGIN = 0x00001040,
    VENDOR_HINT_DISPLAY_OFF = 0x00001040,
    VENDOR_HINT_DISPLAY_ON = 0x00001041,
    VENDOR_HINT_FIRST_DRAW = 0x00001042,
    VENDOR_HINT_TAP_EVENT = 0x00001043,
    VENDOR_HINT_APP_WORKLOAD_TYPE = 0x00001044,
    VENDOR_HINT_WORKLOAD_HEAVY = 0x00001045,
    VENDOR_HINT_HEADROOM_REGULATOR = 0x00001046,
    VENDOR_HINT_SENSOR_DATA = 0x00001047,
    VENDOR_HINT_THERMAL_UPDATE = 0x00001048,
    VENDOR_PERF_EVENT_END = 0x0000107F,

    //perf hints
    VENDOR_PERF_HINT_BEGIN = 0x00001080,
    VENDOR_HINT_SCROLL_BOOST = 0x00001080,
    VENDOR_HINT_FIRST_LAUNCH_BOOST = 0x00001081,
    VENDOR_HINT_SUBSEQ_LAUNCH_BOOST = 0x00001082,
    VENDOR_HINT_ANIM_BOOST = 0x00001083,
    VENDOR_HINT_ACTIVITY_BOOST = 0x00001084,
    VENDOR_HINT_TOUCH_BOOST = 0x00001085,
    VENDOR_HINT_MTP_BOOST = 0x00001086,
    VENDOR_HINT_DRAG_BOOST = 0x00001087,
    VENDOR_HINT_PACKAGE_INSTALL_BOOST = 0x00001088,
    VENDOR_HINT_ROTATION_LATENCY_BOOST = 0x00001089,
    VENDOR_HINT_ROTATION_ANIM_BOOST = 0x00001090,
    VENDOR_HINT_PERFORMANCE_MODE = 0x00001091,
    VENDOR_HINT_APP_UPDATE = 0x00001092,
    VENDOR_HINT_KILL = 0x00001093,

    VENDOR_PERF_HINT_END = 0x000011FF,

    //reserved for power hints
    VENDOR_POWER_HINT_BEGIN = 0x00001200,
    VENDOR_POWER_HINT_END = 0x000015FF,

    VENDOR_FEEDBACK_INPUT_HINT_BEGIN = 0x00001600,
    VENDOR_FEEDBACK_WORKLOAD_TYPE = 0x00001601,
    VENDOR_FEEDBACK_LAUNCH_END_POINT = 0x00001602,
    VENDOR_FEEDBACK_INPUT_HINT_END = 0x00001632,
    VENDOR_HINT_END = 0x00002000,
};

enum {
    LAUNCH_BOOST_V1 = 1,
    LAUNCH_BOOST_V2 = 2,
    LAUNCH_BOOST_V3 = 3,
    LAUNCH_BOOST_GAME = 4,
    LAUNCH_RESERVED_1 = 5,
    LAUNCH_RESERVED_2 = 6,
    LAUNCH_TYPE_SERVICE_START = 100,
    LAUNCH_TYPE_START_PROC = 101,
    LAUNCH_TYPE_APP_START_FROM_BG = 102,
    LAUNCH_TYPE_ATTACH_APPLICATION = 103,
};

typedef struct mpctl_msg_t {
    uint16_t data;
    int32_t pl_handle;
    uint8_t req_type;
    int32_t profile;
    int32_t pl_time;
    int32_t pl_args[MAX_ARGS_PER_REQUEST];
    pid_t client_pid;
    pid_t client_tid;
    uint32_t hint_id;
    int32_t hint_type;
    void *userdata;
    char usrdata_str[MAX_MSG_APP_NAME_LEN];
} mpctl_msg_t;


struct profile_handle {
    int profile;
    int priority;
    int handle;
    struct profile_handle *next;
};

enum {
    MPCTLV3_TOGGLE_POWER_COLLAPSE                      = 0x40400000,
    MPCTLV3_L2_POWER_COLLAPSE_PERF                     = 0x40404000,
    MPCTLV3_LPM_BIAS_HYST                              = 0x40408000,
    MPCTLV3_LPM_LEVELS_REF_STDDEV                      = 0x4040C000,
    MPCTLV3_LPM_LEVELS_TMR_ADD                         = 0x40410000,
    MPCTLV3_LPM_IPI_PREDICTION                         = 0x40414000,

    MPCTLV3_MIN_FREQ_CLUSTER_BIG_CORE_0                 = 0x40800000,
    MPCTLV3_MIN_FREQ_CLUSTER_BIG_CORE_1                 = 0x40800010,
    MPCTLV3_MIN_FREQ_CLUSTER_BIG_CORE_2                 = 0x40800020,
    MPCTLV3_MIN_FREQ_CLUSTER_BIG_CORE_3                 = 0x40800030,
    MPCTLV3_MIN_FREQ_CLUSTER_LITTLE_CORE_0                 = 0x40800100,
    MPCTLV3_MIN_FREQ_CLUSTER_LITTLE_CORE_1                 = 0x40800110,
    MPCTLV3_MIN_FREQ_CLUSTER_LITTLE_CORE_2                 = 0x40800120,
    MPCTLV3_MIN_FREQ_CLUSTER_LITTLE_CORE_3                 = 0x40800130,
    MPCTLV3_MIN_FREQ_CLUSTER_PRIME_CORE_0                 = 0x40800200,

    MPCTLV3_MAX_FREQ_CLUSTER_BIG_CORE_0                 = 0x40804000,
    MPCTLV3_MAX_FREQ_CLUSTER_BIG_CORE_1                 = 0x40804010,
    MPCTLV3_MAX_FREQ_CLUSTER_BIG_CORE_2                 = 0x40804020,
    MPCTLV3_MAX_FREQ_CLUSTER_BIG_CORE_3                 = 0x40804030,
    MPCTLV3_MAX_FREQ_CLUSTER_LITTLE_CORE_0                 = 0x40804100,
    MPCTLV3_MAX_FREQ_CLUSTER_LITTLE_CORE_1                 = 0x40804110,
    MPCTLV3_MAX_FREQ_CLUSTER_LITTLE_CORE_2                 = 0x40804120,
    MPCTLV3_MAX_FREQ_CLUSTER_LITTLE_CORE_3                 = 0x40804130,
    MPCTLV3_MAX_FREQ_CLUSTER_PRIME_CORE_0                 = 0x40804200,

    MPCTLV3_SCHED_BOOST                               = 0x40C00000,
    MPCTLV3_SCHED_PREFER_IDLE                         = 0x40C04000,
    MPCTLV3_SCHED_MIGRATE_COST                        = 0x40C08000,
    MPCTLV3_SCHED_SMALL_TASK                          = 0x40C0C000,
    MPCTLV3_SCHED_MOSTLY_IDLE_LOAD                    = 0x40C10000,
    MPCTLV3_SCHED_MOSTLY_IDLE_NR_RUN                  = 0x40C14000,
    MPCTLV3_SCHED_INIT_TASK_LOAD                      = 0x40C18000,
    MPCTLV3_SCHED_UPMIGRATE                           = 0x40C1C000,
    MPCTLV3_SCHED_DOWNMIGRATE                         = 0x40C20000,
    MPCTLV3_SCHED_MOSTLY_IDLE_FREQ                    = 0x40C24000,
    MPCTLV3_SCHED_GROUP                               = 0x40C28000,
    MPCTLV3_SCHED_SPILL_NR_RUN                        = 0x40C2C000,
    MPCTLV3_SCHED_STATIC_CPU_PWR_COST                 = 0x40C30000,
    MPCTLV3_SCHED_RESTRICT_CLUSTER_SPILL              = 0x40C34000,
    MPCTLV3_SCHED_FREQ_AGGR_GROUP                     = 0x40C38000,
    MPCTLV3_SCHED_CPUSET_TOP_APP                      = 0x40C3C000,
    MPCTLV3_SCHED_CPUSET_FOREGROUND                   = 0x40C40000,
    MPCTLV3_SCHED_CPUSET_SYSTEM_BACKGROUND            = 0x40C44000,
    MPCTLV3_SCHED_CPUSET_BACKGROUND                   = 0x40C48000,
    MPCTLV3_SCHED_SET_FREQ_AGGR                       = 0x40C4C000,
    MPCTLV3_SCHED_ENABLE_THREAD_GROUPING              = 0x40C50000,
    MPCTLV3_SCHED_GROUP_UPMIGRATE                     = 0x40C54000,
    MPCTLV3_SCHED_GROUP_DOWNMIGRATE                   = 0x40C58000,
    MPCTLV3_SCHED_FREQ_AGGR_THRESHOLD                 = 0x40C5C000,
    MPCTLV3_SCHEDTUNE_PREFER_IDLE                     = 0x40C60000,
    MPCTLV3_SCHED_INITIAL_TASK_UTIL                   = 0x40C64000,
    MPCTLV3_SCHED_LOAD_BOOST                          = 0x40C68000,
    MPCTLV3_SCHED_LITTLE_CLUSTER_COLOC_FMIN_KHZ       = 0x40C6C000,
    MPCTLV3_SCHEDTUNE_BOOST                           = 0x40C70000,
    MPCTLV3_SCHED_BUSY_HYSTERSIS_CPU_MASK             = 0x40C74000,
    MPCTLV3_SCHED_MIN_TASK_UTIL_FOR_COLOCATION        = 0x40C78000,
    MPCTLV3_SCHED_MIN_TASK_UTIL_FOR_BOOST             = 0x40C7C000,
    MPCTLV3_SCHED_TASK_BOOST                          = 0x40C80000,
    MPCTLV3_SCHED_USER_HINT                           = 0x40C84000,
    MPCTLV3_SCHED_TASK_UNFILTER_NR_WINDOWS            = 0x40C88000,
    MPCTLV3_SCHED_COLOC_DOWNMIGRATE_NS                = 0x40C8C000,
    MPCTLV3_SCHED_BUSY_HYSTERESIS_ENABLE_COLOC_CPUS   = 0x40C90000,
    MPCTLV3_SCHED_COLOC_BIAS_HYST                     = 0x40C94000,
    MPCTLV3_SCHED_WINDOW_STATS_POLICY                 = 0x40C98000,
    MPCTLV3_SCHED_MANY_WAKEUP_THRESHOLD               = 0x40C9C000,
    MPCTLV3_SCHED_SYNC_HINT_ENABLE                    = 0x40CA0000,
    MPCTLV3_SCHED_WINDOW_TICKS_UPDATE                 = 0x40CA4000,

    MPCTLV3_MIN_ONLINE_CPU_CLUSTER_BIG                  = 0x41000000,
    MPCTLV3_MIN_ONLINE_CPU_CLUSTER_LITTLE               = 0x41000100,
    MPCTLV3_MIN_ONLINE_CPU_CLUSTER_PRIME                = 0x41000200,
    MPCTLV3_MAX_ONLINE_CPU_CLUSTER_BIG                  = 0x41004000,
    MPCTLV3_MAX_ONLINE_CPU_CLUSTER_LITTLE               = 0x41004100,

    MPCTLV3_ABOVE_HISPEED_DELAY_INTERACTIVE_CLUSTER_BIG = 0x41400000,
    MPCTLV3_BOOST_INTERACTIVE_CLUSTER_BIG               = 0x41404000,
    MPCTLV3_BOOSTPULSE_INTERACTIVE_CLUSTER_BIG          = 0x41408000,
    MPCTLV3_BOOSTPULSE_DURATION_INTERACTIVE_CLUSTER_BIG = 0x4140C000,
    MPCTLV3_GO_HISPEED_LOAD_INTERACTIVE_CLUSTER_BIG     = 0x41410000,
    MPCTLV3_HISPEED_FREQ_INTERACTIVE_CLUSTER_BIG        = 0x41414000,
    MPCTLV3_IO_IS_BUSY_INTERACTIVE_CLUSTER_BIG          = 0x41418000,
    MPCTLV3_MIN_SAMPLE_TIME_INTERACTIVE_CLUSTER_BIG     = 0x4141C000,
    MPCTLV3_TARGET_LOADS_INTERACTIVE_CLUSTER_BIG        = 0x41420000,
    MPCTLV3_TIMER_RATE_INTERACTIVE_CLUSTER_BIG          = 0x41424000,
    MPCTLV3_TIMER_SLACK_INTERACTIVE_CLUSTER_BIG         = 0x41428000,
    MPCTLV3_MAX_FREQ_HYSTERESIS_INTERACTIVE_CLUSTER_BIG = 0x4142C000,
    MPCTLV3_USE_SCHED_LOAD_INTERACTIVE_CLUSTER_BIG      = 0x41430000,
    MPCTLV3_USE_MIGRATION_NOTIF_CLUSTER_BIG             = 0x41434000,
    MPCTLV3_IGNORE_HISPEED_NOTIF_CLUSTER_BIG            = 0x41438000,
    MPCTLV3_SCHEDUTIL_HISPEED_FREQ_CLUSTER_BIG          = 0x4143C000,
    MPCTLV3_SCHEDUTIL_HISPEED_LOAD_CLUSTER_BIG          = 0x41440000,
    MPCTLV3_SCHEDUTIL_PREDICTIVE_LOAD_CLUSTER_BIG       = 0x41444000,
    MPCTLV3_SCHEDUTIL_DOWN_RATE_LIMIT_CLUSTER_BIG       = 0x41448000,
    MPCTLV3_SCHEDUTIL_RTG_BOOST_FREQ_CLUSTER_BIG        = 0x4144C000,

    MPCTLV3_ABOVE_HISPEED_DELAY_INTERACTIVE_CLUSTER_LITTLE = 0x41400100,
    MPCTLV3_BOOST_INTERACTIVE_CLUSTER_LITTLE               = 0x41404100,
    MPCTLV3_BOOSTPULSE_INTERACTIVE_CLUSTER_LITTLE          = 0x41408100,
    MPCTLV3_BOOSTPULSE_DURATION_INTERACTIVE_CLUSTER_LITTLE = 0x4140C100,
    MPCTLV3_GO_HISPEED_LOAD_INTERACTIVE_CLUSTER_LITTLE     = 0x41410100,
    MPCTLV3_HISPEED_FREQ_INTERACTIVE_CLUSTER_LITTLE        = 0x41414100,
    MPCTLV3_IO_IS_BUSY_INTERACTIVE_CLUSTER_LITTLE          = 0x41418100,
    MPCTLV3_MIN_SAMPLE_TIME_INTERACTIVE_CLUSTER_LITTLE     = 0x4141C100,
    MPCTLV3_TARGET_LOADS_INTERACTIVE_CLUSTER_LITTLE        = 0x41420100,
    MPCTLV3_TIMER_RATE_INTERACTIVE_CLUSTER_LITTLE          = 0x41424100,
    MPCTLV3_TIMER_SLACK_INTERACTIVE_CLUSTER_LITTLE         = 0x41428100,
    MPCTLV3_MAX_FREQ_HYSTERESIS_INTERACTIVE_CLUSTER_LITTLE = 0x4142C100,
    MPCTLV3_USE_SCHED_LOAD_INTERACTIVE_CLUSTER_LITTLE      = 0x41430100,
    MPCTLV3_USE_MIGRATION_NOTIF_CLUSTER_LITTLE             = 0x41434100,
    MPCTLV3_IGNORE_HISPEED_NOTIF_CLUSTER_LITTLE            = 0x41438100,
    MPCTLV3_SCHEDUTIL_HISPEED_FREQ_CLUSTER_LITTLE          = 0x4143C100,
    MPCTLV3_SCHEDUTIL_HISPEED_LOAD_CLUSTER_LITTLE          = 0x41440100,
    MPCTLV3_SCHEDUTIL_PREDICTIVE_LOAD_CLUSTER_LITTLE       = 0x41444100,
    MPCTLV3_SCHEDUTIL_DOWN_RATE_LIMIT_CLUSTER_LITTLE       = 0x41448100,
    MPCTLV3_SCHEDUTIL_RTG_BOOST_FREQ_CLUSTER_LITTLE        = 0x4144C100,

    MPCTLV3_CPUBW_HWMON_MIN_FREQ                      = 0x41800000,
    MPCTLV3_CPUBW_HWMON_DECAY_RATE                    = 0x41804000,
    MPCTLV3_CPUBW_HWMON_IO_PERCENT                    = 0x41808000,
    MPCTLV3_CPUBW_HWMON_HYST_OPT                      = 0x4180C000,
    MPCTLV3_CPUBW_HWMON_LOW_POWER_CEIL_MBPS           = 0x41810000,
    MPCTLV3_CPUBW_HWMON_LOW_POWER_IO_PERCENT          = 0x41814000,
    MPCTLV3_CPUBW_HWMON_MAX_FREQ                      = 0x41818000,
    MPCTLV3_CPUBW_HWMON_POLLING_INTERVAL              = 0x4181C000,
    MPCTLV3_CPUBW_HWMON_SAMPLE_MS                     = 0x41820000,
    MPCTLV3_CPUBW_HWMON_IDLE_MBPS                     = 0x41824000,
    MPCTLV3_CPU_LLCC_BW_MIN_FREQ                      = 0x41828000,
    MPCTLV3_CPU_LLCC_BW_UP_SCALE                      = 0x4182C000,
    MPCTLV3_CPU_LLCC_BW_USE_AB                        = 0x41830000,

    MPCTLV3_VIDEO_ENCODE_PB_HINT                      = 0x41C00000,
    MPCTLV3_VIDEO_DECODE_PB_HINT                      = 0x41C04000,
    MPCTLV3_VIDEO_DISPLAY_PB_HINT                     = 0x41C08000,

    MPCTLV3_KSM_RUN_STATUS                            = 0x42000000,
    MPCTLV3_KSM_PARAMS                                = 0x42004000,

    MPCTLV3_SAMPLING_RATE_ONDEMAND                    = 0x42400000,
    MPCTLV3_IO_IS_BUSY_ONDEMAND                       = 0x42404000,
    MPCTLV3_SAMPLING_DOWN_FACTOR_ONDEMAND             = 0x42408000,
    MPCTLV3_SYNC_FREQ_ONDEMAND                        = 0x4240C000,
    MPCTLV3_OPTIMAL_FREQ_ONDEMAND                     = 0x42410000,
    MPCTLV3_ENABLE_STEP_UP_ONDEMAND                   = 0x42414000,
    MPCTLV3_MAX_INTERMEDIATE_STEPS_ONDEMAND           = 0x42418000,
    MPCTLV3_NOTIFY_ON_MIGRATE                         = 0x4241C000,

    MPCTLV3_GPU_POWER_LEVEL                           = 0X42800000,
    MPCTLV3_GPU_MIN_POWER_LEVEL                       = 0X42804000,
    MPCTLV3_GPU_MAX_POWER_LEVEL                       = 0X42808000,
    MPCTLV3_GPU_MIN_FREQ                              = 0X4280C000,
    MPCTLV3_GPU_MAX_FREQ                              = 0X42810000,
    MPCTLV3_GPU_BUS_MIN_FREQ                          = 0X42814000,
    MPCTLV3_GPU_BUS_MAX_FREQ                          = 0X42818000,
    MPCTLV3_GPU_DISABLE_GPU_NAP                       = 0X4281C000,

    MPCTLV3_IRQ_BALANCER                              = 0X42C04000,
    MPCTLV3_UNSUPPORTED                               = 0X42C00000,
    MPCTLV3_INPUT_BOOST_RESET                         = 0x42C08000,
    MPCTLV3_SWAP_RATIO                                = 0x42C0C000,
    MPCTLV3_STORAGE_CLK_SCALING_DISABLE               = 0x42C10000,
    MPCTLV3_KEEP_ALIVE                                = 0x42C14000,
    MPCTLV3_DISABLE_PPR                               = 0x42C18000,

    MPCTLV3_LLCBW_HWMON_MIN_FREQ                      = 0x43000000,
    MPCTLV3_LLCBW_HWMON_IO_PERCENT                    = 0x43004000,
    MPCTLV3_LLCBW_HWMON_HYST_OPT                      = 0x43008000,
    MPCTLV3_LLCBW_HWMON_SAMPLE_MS                     = 0x4300C000,
    MPCTLV3_LLCC_DDR_BW_MIN_FREQ                      = 0x43010000,
    MPCTLV3_LLCC_DDR_BW_UP_SCALE                      = 0x43014000,
    MPCTLV3_LLCC_DDR_BW_MAX_FREQ                      = 0x43018000,

    MPCTLV3_L3_MEMLAT_MIN_FREQ                        = 0x43400000,
    MPCTLV3_L2_MEMLAT_RATIO_CEIL_0                    = 0x43404000,
    MPCTLV3_L2_MEMLAT_RATIO_CEIL_1                    = 0x43408000,
    MPCTLV3_L3_MEMLAT_STALL_FLOOR_0                   = 0X4340C000,
    MPCTLV3_L3_MEMLAT_STALL_FLOOR_1                   = 0X43410000,
    MPCTLV3_MEMLAT_MIN_FREQ_0                         = 0x43414000,
    MPCTLV3_MEMLAT_MIN_FREQ_1                         = 0x43418000,
    MPCTLV3_LLCC_MEMLAT_MIN_FREQ                      = 0x4341C000,
    MPCTLV3_LLCC_MEMLAT_RATIO_CELL_0                  = 0x43420000,
    MPCTLV3_LLCC_MEMLAT_RATIO_CELL_1                  = 0x43424000,
    MPCTLV3_LLCC_MEMLAT_STALL_FLOOR_0                 = 0x43428000,
    MPCTLV3_LLCC_MEMLAT_STALL_FLOOR_1                 = 0x4342C000,
    MPCTLV3_LLCC_DDR_LAT_MIN_FREQ_0                   = 0x43430000,
    MPCTLV3_LLCC_DDR_LAT_MIN_FREQ_1                   = 0x43430100,
    MPCTLV3_LLCC_DDR_LAT_RATIO_CELL_0                 = 0x43434000,
    MPCTLV3_LLCC_DDR_LAT_RATIO_CELL_1                 = 0x43438000,
    MPCTLV3_LLCC_DDR_LAT_STALL_FLOOR_0                = 0x4343C000,
    MPCTLV3_LLCC_DDR_LAT_STALL_FLOOR_1                = 0x43440000,

};

enum major_resource_opcode {
    DISPLAY_OFF_MAJOR_OPCODE = 0,  /* 0x0 */
    POWER_COLLAPSE_MAJOR_OPCODE,   /* 0x1 */
    CPUFREQ_MAJOR_OPCODE,          /* 0x2 */
    SCHED_MAJOR_OPCODE,            /* 0x3 */
    CORE_HOTPLUG_MAJOR_OPCODE,     /* 0x4 */
    INTERACTIVE_MAJOR_OPCODE,      /* 0x5 */
    CPUBW_HWMON_MAJOR_OPCODE,      /* 0x6 */
    VIDEO_MAJOR_OPCODE,            /* 0x7 */
    KSM_MAJOR_OPCODE,              /* 0x8 */
    ONDEMAND_MAJOR_OPCODE,         /* 0x9 */
    GPU_MAJOR_OPCODE,              /* 0xA */
    MISC_MAJOR_OPCODE,             /* 0xB */
    LLCBW_HWMON_MAJOR_OPCODE,      /* 0xC */
    MEMLAT_MAJOR_OPCODE,        /* 0xD */
    MAX_MAJOR_RESOURCES
};

//do not change order of "resource minor groups"
enum minor_resource_opcode {
    DISPLAY_START_INDEX = 0,
    DISPLAY_OFF_OPCODE = 0,
    MAX_DISPLAY_MINOR_OPCODE,

    POWER_COLLAPSE_START_INDEX = DISPLAY_START_INDEX + MAX_DISPLAY_MINOR_OPCODE,
    POWER_COLLAPSE_OPCODE = 0,     /*0x0*/
    L2_POWER_COLLAPSE_PERF_OPCODE, /*0x1*/
    LPM_BIAS_HYST_OPCODE,          /*0x2*/
    LPM_LEVELS_REF_STDDEV,
    LPM_LEVELS_TMR_ADD,
    LPM_IPI_PREDICTION,
    MAX_PC_MINOR_OPCODE,

    CPUFREQ_START_INDEX = POWER_COLLAPSE_START_INDEX + MAX_PC_MINOR_OPCODE,
    CPUFREQ_MIN_FREQ_OPCODE = 0, /* 0x0 */
    CPUFREQ_MAX_FREQ_OPCODE,     /* 0x1 */
    MAX_CPUFREQ_MINOR_OPCODE,

    SCHED_START_INDEX = CPUFREQ_START_INDEX + MAX_CPUFREQ_MINOR_OPCODE,
    SCHED_BOOST_OPCODE = 0,          /* 0x0 */
    SCHED_PREFER_IDLE_OPCODE,        /* 0x1 */
    SCHED_MIGRATE_COST_OPCODE,       /* 0x2 */
    SCHED_SMALL_TASK_OPCODE,         /* 0x3 */
    SCHED_MOSTLY_IDLE_LOAD_OPCODE,   /* 0x4 */
    SCHED_MOSTLY_IDLE_NR_RUN_OPCODE, /* 0x5 */
    SCHED_INIT_TASK_LOAD_OPCODE,     /* 0x6 */
    SCHED_UPMIGRATE_OPCODE,          /* 0x7 */
    SCHED_DOWNMIGRATE_OPCODE,        /* 0x8 */
    SCHED_MOSTLY_IDLE_FREQ_OPCODE,   /* 0x9 */
    SCHED_GROUP_OPCODE,              /* 0xA */
    SCHED_SPILL_NR_RUN_OPCODE,       /* 0xB */
    SCHED_STATIC_CPU_PWR_COST_OPCODE,/* 0xC */
    SCHED_RESTRICT_CLUSTER_SPILL_OPCODE,/* 0xD */
    SCHED_FREQ_AGGR_GROUP_OPCODE,       /* 0xE */
    SCHED_CPUSET_TOP_APP_OPCODE,     /* 0xF */
    SCHED_CPUSET_FOREGROUND_OPCODE,  /* 0x10 */
    SCHED_CPUSET_SYSTEM_BACKGROUND_OPCODE, /* 0x11 */
    SCHED_CPUSET_BACKGROUND_OPCODE,  /* 0x12 */
    SCHED_SET_FREQ_AGGR_OPCODE,      /* 0x13 */
    SCHED_ENABLE_THREAD_GROUPING_OPCODE,   /* 0x14 */
    SCHED_GROUP_UPMIGRATE_OPCODE,   /* 0x15 */
    SCHED_GROUP_DOWNMIGRATE_OPCODE,   /* 0x16 */
    SCHED_FREQ_AGGR_THRESHOLD_OPCODE,   /* 0x17 */
    SCHEDTUNE_PREFER_IDLE_OPCODE,      /*0x18*/
    SCHED_INITIAL_TASK_UTIL_OPCODE,      /*0x19*/
    SCHED_LOAD_BOOST_OPCODE,            /*0x1A*/
    SCHED_SCHED_LITTLE_CLUSTER_COLOC_FMIN_KHZ, /*0x1B*/
    SCHEDTUNE_BOOST_OPCODE,          /* 0x1C */
    SCHED_BUSY_HYSTERSIS_CPU_MASK_OPCODE,   /* 0x1D */
    SCHED_MIN_TASK_UTIL_FOR_COLOCATION,        /*0x1E*/
    SCHED_MIN_TASK_UTIL_FOR_BOOST,             /*0x1F*/
    SCHED_TASK_BOOST, /*0x20*/
    SCHED_USER_HINT, /*0x21*/
    SCHED_TASK_UNFILTER_NR_WINDOWS, /*0x22*/
    SCHED_COLOC_DOWNMIGRATE_NS, /*0x23*/
    SCHED_BUSY_HYSTERESIS_ENABLE_COLOC_CPUS, /*0x24*/
    SCHED_COLOC_BIAS_HYST, /*0x25*/
    SCHED_WINDOW_STATS_POLICY, /*0x26*/
    SCHED_MANY_WAKEUP_THRESHOLD, /*0x27*/
    SCHED_SYNC_HINT_ENABLE, /*0x28*/
    SCHED_WINDOW_TICKS_UPDATE, /*0x29*/
    MAX_SCHED_MINOR_OPCODE,

    CORE_HOTPLUG_START_INDEX = SCHED_START_INDEX + MAX_SCHED_MINOR_OPCODE,
    CORE_HOTPLUG_MIN_CORE_ONLINE_OPCODE = 0,
    CORE_HOTPLUG_MAX_CORE_ONLINE_OPCODE,
    MAX_CORE_HOTPLUG_MINOR_OPCODE,

    INTERACTIVE_START_INDEX = CORE_HOTPLUG_START_INDEX + MAX_CORE_HOTPLUG_MINOR_OPCODE,
    INTERACTIVE_ABOVE_HISPEED_DELAY_OPCODE = 0,  /* 0x0 */
    INTERACTIVE_BOOST_OPCODE,                    /* 0x1 */
    INTERACTIVE_BOOSTPULSE_OPCODE,               /* 0x2 */
    INTERACTIVE_BOOSTPULSE_DURATION_OPCODE,      /* 0x3 */
    INTERACTIVE_GO_HISPEED_LOAD_OPCODE,          /* 0x4 */
    INTERACTIVE_HISPEED_FREQ_OPCODE,             /* 0x5 */
    INTERACTIVE_IO_IS_BUSY_OPCODE,               /* 0x6 */
    INTERACTIVE_MIN_SAMPLE_TIME_OPCODE,          /* 0x7 */
    INTERACTIVE_TARGET_LOADS_OPCODE,             /* 0x8 */
    INTERACTIVE_TIMER_RATE_OPCODE,               /* 0x9 */
    INTERACTIVE_TIMER_SLACK_OPCODE,              /* 0xA */
    INTERACTIVE_MAX_FREQ_HYSTERESIS_OPCODE,      /* 0xB */
    INTERACTIVE_USE_SCHED_LOAD_OPCODE,           /* 0xC */
    INTERACTIVE_USE_MIGRATION_NOTIF_OPCODE,      /* 0xD */
    INTERACTIVE_IGNORE_HISPEED_NOTIF_OPCODE,     /* 0xE */
    SCHEDUTIL_HISPEED_FREQ_OPCODE,               /* 0xF */
    SCHEDUTIL_HISPEED_LOAD_OPCODE,               /* 0x10 */
    SCHEDUTIL_PREDICTIVE_LOAD_OPCODE,            /* 0x11 */
    SCHEDUTIL_DOWN_RATE_LIMIT_OPCODE,            /* 0x12 */
    SCHEDUTIL_RTG_BOOST_FREQ_OPCODE,             /* 0x13 */
    MAX_INTERACTIVE_MINOR_OPCODE,

    CPUBW_HWMON_START_INDEX = INTERACTIVE_START_INDEX + MAX_INTERACTIVE_MINOR_OPCODE,
    CPUBW_HWMON_MINFREQ_OPCODE = 0,
    CPUBW_HWMON_DECAY_RATE_OPCODE,
    CPUBW_HWMON_IO_PERCENT_OPCODE,
    CPUBW_HWMON_HYST_OPT_OPCODE,
    CPUBW_HWMON_LOW_POWER_CEIL_MBPS,
    CPUBW_HWMON_LOW_POWER_IO_PERCENT,
    CPUBW_HWMON_MAXFREQ_OPCODE,
    CPUBW_HWMON_POLLING_INTERVAL_OPCODE,
    CPUBW_HWMON_SAMPLE_MS,
    CPUBW_HWMON_IDLE_MBPS,
    CPU_LLCC_BW_MIN_FREQ,
    CPU_LLCC_BW_UP_SCALE,
    CPU_LLCC_BW_USE_AB,
    MAX_CPUBW_HWMON_MINOR_OPCODE,

    VIDEO_START_INDEX = CPUBW_HWMON_START_INDEX + MAX_CPUBW_HWMON_MINOR_OPCODE,
    VIDEO_ENCODE_PB_HINT = 0,
    VIDEO_DECODE_PB_HINT,
    VIDEO_DISPLAY_PB_HINT,
    MAX_VIDEO_MINOR_OPCODE,

    KSM_START_INDEX = VIDEO_START_INDEX + MAX_VIDEO_MINOR_OPCODE,
    KSM_ENABLE_DISABLE_OPCODE = 0,
    KSM_SET_RESET_OPCODE,
    MAX_KSM_MINOR_OPCODE,

    ONDEMAND_START_INDEX = KSM_START_INDEX + MAX_KSM_MINOR_OPCODE,
    OND_SAMPLING_RATE_OPCODE = 0,
    OND_IO_IS_BUSY_OPCODE,
    OND_SAMPLING_DOWN_FACTOR_OPCODE,
    OND_SYNC_FREQ_OPCODE,
    OND_OPIMAL_FREQ_OPCODE,
    OND_ENABLE_STEP_UP,
    OND_MAX_INTERMEDIATE_STEPS,
    NOTIFY_ON_MIGRATE,
    MAX_OND_MINOR_OPCODE,

    GPU_START_INDEX = ONDEMAND_START_INDEX + MAX_OND_MINOR_OPCODE,
    GPU_POWER_LEVEL = 0,
    GPU_MIN_POWER_LEVEL,
    GPU_MAX_POWER_LEVEL,
    GPU_MIN_FREQ_OPCODE,
    GPU_MAX_FREQ_OPCODE,
    GPU_BUS_MIN_FREQ_OPCODE,
    GPU_BUS_MAX_FREQ_OPCODE,
    GPU_DISABLE_GPU_NAP_OPCODE,
    MAX_GPU_MINOR_OPCODE,

    MISC_START_INDEX = GPU_START_INDEX + MAX_GPU_MINOR_OPCODE,
    UNSUPPORTED_OPCODE = 0,
    IRQ_BAL_OPCODE,
    INPUT_BOOST_RESET_OPCODE,
    SWAP_RATIO_OPCODE,
    STORAGE_CLK_SCALING_DISABLE_OPCODE,
    NET_KEEP_ALIVE_OPCODE,
    DISABLE_PPR,
    MAX_MISC_MINOR_OPCODE,

    LLCBW_HWMON_START_INDEX = MISC_START_INDEX + MAX_MISC_MINOR_OPCODE,
    LLCBW_HWMON_MINFREQ_OPCODE = 0,
    LLCBW_HWMON_IO_PERCENT_OPCODE,
    LLCBW_HWMON_HYST_OPT_OPCODE,
    LLCBW_HWMON_SAMPLE_MS,
    LLCC_DDR_BW_MIN_FREQ,
    LLCC_DDR_BW_UP_SCALE,
    LLCC_DDR_BW_MAX_FREQ,
    MAX_LLCBW_HWMON_MINOR_OPCODE,

    MEMLAT_START_INDEX = LLCBW_HWMON_START_INDEX + MAX_LLCBW_HWMON_MINOR_OPCODE,
    L3_MEMLAT_MINFREQ_OPCODE = 0,
    L2_MEMLAT_RATIO_CEIL_0_OPCODE,
    L2_MEMLAT_RATIO_CEIL_1_OPCODE,
    L3_MEMLAT_STALL_FLOOR_0_OPCODE,
    L3_MEMLAT_STALL_FLOOR_1_OPCODE,
    MEMLAT_MIN_FREQ_0_OPCODE,
    MEMLAT_MIN_FREQ_1_OPCODE,
    LLCC_MEMLAT_MIN_FREQ_OPCODE,
    LLCC_MEMLAT_RATIO_CELL_0_OPCODE,
    LLCC_MEMLAT_RATIO_CELL_1_OPCODE,
    LLCC_MEMLAT_STALL_FLOOR_0_OPCODE,
    LLCC_MEMLAT_STALL_FLOOR_1_OPCODE,
    LLCC_DDR_LAT_MIN_FREQ_OPCODE,
    LLCC_DDR_LAT_RATIO_CELL_0_OPCODE,
    LLCC_DDR_LAT_RATIO_CELL_1_OPCODE,
    LLCC_DDR_LAT_STALL_FLOOR_0_OPCODE,
    LLCC_DDR_LAT_STALL_FLOOR_1_OPCODE,
    MAX_MEMLAT_MINOR_OPCODE,

    MAX_MINOR_RESOURCES = MAX_DISPLAY_MINOR_OPCODE + MAX_PC_MINOR_OPCODE +
                          MAX_CPUFREQ_MINOR_OPCODE + MAX_SCHED_MINOR_OPCODE +
                          MAX_CORE_HOTPLUG_MINOR_OPCODE + MAX_INTERACTIVE_MINOR_OPCODE +
                          MAX_CPUBW_HWMON_MINOR_OPCODE + MAX_VIDEO_MINOR_OPCODE +
                          MAX_KSM_MINOR_OPCODE + MAX_OND_MINOR_OPCODE + MAX_GPU_MINOR_OPCODE +
                          MAX_MISC_MINOR_OPCODE + MAX_LLCBW_HWMON_MINOR_OPCODE +
                          MAX_MEMLAT_MINOR_OPCODE
};

enum map_type {
    NO_VALUE_MAP = 0,
    VALUE_MAPPED = 1
};

/* Enum for pre-defined cluster map.
 * */
enum predefine_cluster_map {
    START_PRE_DEFINE_CLUSTER = 8,
    LAUNCH_CLUSTER = 8,
    SCROLL_CLUSTER = 9,
    ANIMATION_CLUSTER = 10,
    PREDEFINED_CLUSTER_11 = 11,
    PREDEFINED_CLUSTER_12 = 12,
    PREDEFINED_CLUSTER_13 = 13,
    PREDEFINED_CLUSTER_14 = 14,
    PREDEFINED_CLUSTER_15 = 15,
    MAX_PRE_DEFINE_CLUSTER
};

/* Enum for value mapping for frequency.
 * A target needs to define the mapped
 * values for these.
 * */
enum freq_map {
    LOWEST_FREQ = 0,
    LEVEL1_FREQ = 1,
    LEVEL2_FREQ = 2,
    LEVEL3_FREQ = 3,
    HIGHEST_FREQ = 4,
    FREQ_MAP_MAX
};

/* Enum for workload type and action to be taken for workload type. */
enum workload_type {
    NOT_KNOWN = 0,
    APP,
    GAME,
    BROWSER,
    PREPROAPP
};

enum workload_db_action {
    DELETE,
    INSERT
};


/*older resources
 *   supported till v2.0*/

enum resources {
    DISPLAY = 0,
    POWER_COLLAPSE,
    CPU0_MIN_FREQ,
    CPU1_MIN_FREQ,
    CPU2_MIN_FREQ,
    CPU3_MIN_FREQ,
    UNSUPPORTED_0,
    CLUSTR_0_CPUS_ON,
    CLUSTR_0_MAX_CORES,
    UNSUPPORTED_2,
    UNSUPPORTED_3,
    SAMPLING_RATE,
    ONDEMAND_IO_IS_BUSY,
    ONDEMAND_SAMPLING_DOWN_FACTOR,
    INTERACTIVE_TIMER_RATE,
    INTERACTIVE_HISPEED_FREQ,
    INTERACTIVE_HISPEED_LOAD,
    SYNC_FREQ,
    OPTIMAL_FREQ,
    SCREEN_PWR_CLPS,
    THREAD_MIGRATION,
    CPU0_MAX_FREQ,
    CPU1_MAX_FREQ,
    CPU2_MAX_FREQ,
    CPU3_MAX_FREQ,
    ONDEMAND_ENABLE_STEP_UP,
    ONDEMAND_MAX_INTERMEDIATE_STEPS,
    INTERACTIVE_IO_BUSY,
    KSM_RUN_STATUS,
    KSM_PARAMS,
    SCHED_BOOST,
    CPU4_MIN_FREQ,
    CPU5_MIN_FREQ,
    CPU6_MIN_FREQ,
    CPU7_MIN_FREQ,
    CPU4_MAX_FREQ,
    CPU5_MAX_FREQ,
    CPU6_MAX_FREQ,
    CPU7_MAX_FREQ,
    CPU0_INTERACTIVE_ABOVE_HISPEED_DELAY,
    CPU0_INTERACTIVE_BOOST,
    CPU0_INTERACTIVE_BOOSTPULSE,
    CPU0_INTERACTIVE_BOOSTPULSE_DURATION,
    CPU0_INTERACTIVE_GO_HISPEED_LOAD,
    CPU0_INTERACTIVE_HISPEED_FREQ,
    CPU0_INTERACTIVE_IO_IS_BUSY,
    CPU0_INTERACTIVE_MIN_SAMPLE_TIME,
    CPU0_INTERACTIVE_TARGET_LOADS,
    CPU0_INTERACTIVE_TIMER_RATE,
    CPU0_INTERACTIVE_TIMER_SLACK,
    CPU4_INTERACTIVE_ABOVE_HISPEED_DELAY,
    CPU4_INTERACTIVE_BOOST,
    CPU4_INTERACTIVE_BOOSTPULSE,
    CPU4_INTERACTIVE_BOOSTPULSE_DURATION,
    CPU4_INTERACTIVE_GO_HISPEED_LOAD,
    CPU4_INTERACTIVE_HISPEED_FREQ,
    CPU4_INTERACTIVE_IO_IS_BUSY,
    CPU4_INTERACTIVE_MIN_SAMPLE_TIME,
    CPU4_INTERACTIVE_TARGET_LOADS,
    CPU4_INTERACTIVE_TIMER_RATE,
    CPU4_INTERACTIVE_TIMER_SLACK,
    CLUSTR_1_MAX_CORES,
    SCHED_PREFER_IDLE,
    SCHED_MIGRATE_COST,
    SCHED_SMALL_TASK,
    SCHED_MOSTLY_IDLE_LOAD,
    SCHED_MOSTLY_IDLE_NR_RUN,
    SCHED_INIT_TASK_LOAD,
    VIDEO_DECODE_PLAYBACK_HINT,
    DISPLAY_LAYER_HINT,
    VIDEO_ENCODE_PLAYBACK_HINT,
    CPUBW_HWMON_MIN_FREQ,
    CPUBW_HWMON_DECAY_RATE,
    CPUBW_HWMON_IO_PERCENT,
    CPU0_INTERACTIVE_MAX_FREQ_HYSTERESIS,
    CPU4_INTERACTIVE_MAX_FREQ_HYSTERESIS,
    GPU_DEFAULT_PWRLVL,
    CLUSTR_1_CPUS_ON,
    SCHED_UPMIGRATE,
    SCHED_DOWNMIGRATE,
    SCHED_MOSTLY_IDLE_FREQ,
    IRQ_BALANCER,
    INTERACTIVE_USE_SCHED_LOAD,
    INTERACTIVE_USE_MIGRATION_NOTIF,
    INPUT_BOOST_RESET,
    /* represents the maximum number of
     * optimizations allowed per
     * request and should always be
     * the last element
     */
    OPTIMIZATIONS_MAX
};

int perfmodule_init(void);
void perfmodule_exit(void);
int perfmodule_submit_request(mpctl_msg_t *msg);

#ifdef __cplusplus
}
#endif

#endif /* __MP_CTL_H__ */
