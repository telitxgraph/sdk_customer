/******************************************************************************

  @file	 mps_qmi_test.h

 ******************************************************************************/

#ifndef __QMI_NAS_TEST_H__
#define __QMI_NAS_TEST_H__

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
#include "general_modem_service_v01.h"
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


#define strlcpy g_strlcpy
#define strlcat g_strlcat
typedef unsigned char boolean;
#if !defined(true)
#define true	1
#endif
#if !defined(false)
#define false	0
#endif

#endif
