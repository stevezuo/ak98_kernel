#ifndef _AK98_FREQ_POLICY_H_
#define _AK98_FREQ_POLICY_H_

#define MAX_NAME_LEN	100

#define FREQ_IOC_MAGIC 'f'

#define REQUEST_MODE			_IOW(FREQ_IOC_MAGIC, 32, __u8)
#define RELEASE_MODE			_IOW(FREQ_IOC_MAGIC, 33, __u8)

#ifdef __KERNEL__

#include <linux/anyka_cpufreq.h>

/* micros definitions  */
#define HIGHEST_MODE 	FREQ_MODE_MAX
#define LOWEST_MODE		FREQ_MODE_MIN

#define HIGH_IX (HIGHEST_MODE>LOWEST_MODE ? HIGHEST_MODE : LOWEST_MODE)
#define LOW_IX  (HIGHEST_MODE<LOWEST_MODE ? HIGHEST_MODE : LOWEST_MODE)

#define NUM_OF_MODE  	(LOWEST_MODE-HIGHEST_MODE>0 ? (LOWEST_MODE-HIGHEST_MODE+1):(HIGHEST_MODE-LOWEST_MODE+1))

#define DEFAULT_MODE 		LOWEST_MODE
#define NORMAL_DELTA_MIN 	NORMAL_MODE_CLOCK_3
#define BK_MUSIC_MODE  		NORMAL_MODE_CLOCK_5
#define RELEASE_DELAY 		5000  /* unit: microsecond  */
#define REQUEST_DELAY		2000 
#define PLL_CHANGE_DELAY    2000

typedef T_OPERATION_MODE T_MODE_TYPE;
/**************************************************************/

struct t_app_to_mode {
	char appName[MAX_NAME_LEN];	
	T_OPERATION_MODE mode;
};

struct mode_table_node {
	char modeName[MAX_NAME_LEN];
	T_OPERATION_MODE mode;
};

struct rqst_info
{
	char appName[MAX_NAME_LEN];
	pid_t pid;
	int tag;
};

struct rqst_node {
	struct rqst_info app_info;
	struct list_head list;
};


/*
	request the system to hold the mode until the ak98_release_hold_mode is called
*/
typedef void (*AK98_RQSTMODE_CALLBACK)(void *data);
int ak98_request_hold_mode(T_MODE_TYPE mode, AK98_RQSTMODE_CALLBACK fn, 
	void *data);
int ak98_release_hold_mode(T_MODE_TYPE mode);


#endif

#endif
