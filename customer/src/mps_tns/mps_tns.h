/******************************************************************************

  @file    mps_tns.h
  @brief   TNS (Time Network Synchronization) - SIB9 NR5G Sync Pulse App

  ******************************************************************************/

#ifndef __MPS_TNS_H__
#define __MPS_TNS_H__

#include <syslog.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <asm-generic/signal-defs.h>

#include "qmi_idl_lib.h"
#include "qmi_csi.h"
#include "qmi_sap.h"
#include "network_access_service_v01.h"
#include "qmi_client.h"

/*===========================================================================
                              LOGGING MACROS
===========================================================================*/

static void log_to_syslog(int priority, const char *file, int line, const char *func, const char *fmt, ...)
{
 	va_list args;
 	struct timespec ts;
 	struct tm tm_info;
 	char time_buf[32];

 	clock_gettime(CLOCK_REALTIME, &ts);
 	localtime_r(&ts.tv_sec, &tm_info);
 	snprintf(time_buf, sizeof(time_buf), "%04d-%02d-%02d %02d:%02d:%02d.%03ld",
 	         tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday,
 	         tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec,
 	         ts.tv_nsec / 1000000);

 	va_start(args, fmt);

 	char buffer[512];
 	snprintf(buffer, sizeof(buffer), "[%s][%s:%d] %s() ", time_buf, file, line, func);
 	vsnprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), fmt, args);
 	syslog(priority, "%s", buffer);
 	printf("%s\n", buffer);

 	va_end(args);
}

#define LOGE(fmt, ...) log_to_syslog(LOG_ERR, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)
#define LOGI(fmt, ...) log_to_syslog(LOG_INFO, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)
#define LOGD(fmt, ...) log_to_syslog(LOG_DEBUG, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)

/*===========================================================================
                              TYPE DEFINITIONS
===========================================================================*/

typedef unsigned char boolean;
#if !defined(true)
#define true    1
#endif
#if !defined(false)
#define false   0
#endif

/*===========================================================================
                              CONSTANTS
===========================================================================*/

#define TNS_SEND_TIMEOUT        50000
#define TNS_CLIENT_CB_DATA      0xBEEF

/*===========================================================================
                       SYNC PULSE CONFIG STRUCTURE
===========================================================================*/

typedef struct {
	uint32_t pulse_period;          /* 0-128, multiple of 10ms. 0 = stop */
	uint32_t start_sfn;             /* 0-1024. 1024 = next available SFN */
	uint32_t report_period;         /* 0-128, multiple of 10ms. 0 = disabled */
	uint8_t  pulse_align_type;      /* 0 = NR5G frame boundary, 1 = UTC second boundary */
	uint8_t  pulse_trigger_action;  /* 0 = Trigger pulse, 1 = Skip pulse */
	uint8_t  pulse_get_cxo_count;   /* 0 = Do not get CXO count, 1 = Get CXO count */
} tns_sync_pulse_config_t;

/*===========================================================================
                              FUNCTION DECLARATIONS
===========================================================================*/

/* Config operations */
void tns_config_set_defaults(tns_sync_pulse_config_t *config);

#endif /* __MPS_TNS_H__ */
