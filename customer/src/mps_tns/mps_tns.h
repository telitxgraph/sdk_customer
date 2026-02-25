/******************************************************************************

  @file    mps_tns.h
  @brief   TNS (Time Network Synchronization) - SIB9 NR5G Sync Pulse App

  ******************************************************************************/

#ifndef __MPS_TNS_H__
#define __MPS_TNS_H__

#include <syslog.h>
#include <stdarg.h>
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
	va_start(args, fmt);

	char buffer[512];
	snprintf(buffer, sizeof(buffer), "[TNS][%s:%d] %s() ", file, line, func);
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
#define TNS_CONFIG_FILE_PATH    "/etc/tns/tns_config.conf"
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

/* Config file operations */
int  tns_config_load(const char *path, tns_sync_pulse_config_t *config);
void tns_config_set_defaults(tns_sync_pulse_config_t *config);

#endif /* __MPS_TNS_H__ */
