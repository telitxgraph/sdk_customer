/******************************************************************************
 *
 *  @file    nas_nr5g_indications.h
 *  @brief   TNS (Time Network Synchronization) - SIB9 NR5G Sync Pulse App
 *
 ******************************************************************************/

#ifndef __NAS_NR5G_INDICATIONS_H__
#define __NAS_NR5G_INDICATIONS_H__

/*
 * Define FEATURE_ENABLE_LOGGING_TO_SYSLOG to enable syslog-based logging
 * with timestamps and source location information.
 * When undefined, log output is directed to stdout via printf.
 */

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

#ifdef FEATURE_ENABLE_LOGGING_TO_SYSLOG

/**
 * @brief  Log a message to syslog with timestamp and source location.
 * @param  priority  syslog priority (LOG_ERR, LOG_INFO, LOG_DEBUG)
 * @param  file      source file name (__FILE__)
 * @param  line      source line number (__LINE__)
 * @param  func      function name (__FUNCTION__)
 * @param  fmt       printf-style format string
 * @param  ...       variable arguments for format string
 * @return None
 */
static void log_to_syslog( int priority, const char *file, int line,
                            const char *func, const char *fmt, ... )
{
  va_list args;
  struct timespec ts;
  struct tm tm_info;
  char time_buf[32];
  char buffer[512];

  clock_gettime( CLOCK_REALTIME, &ts );
  localtime_r( &ts.tv_sec, &tm_info );
  snprintf( time_buf, sizeof( time_buf ),
            "%04d-%02d-%02d %02d:%02d:%02d.%03ld",
            tm_info.tm_year + 1900, tm_info.tm_mon + 1,
            tm_info.tm_mday, tm_info.tm_hour, tm_info.tm_min,
            tm_info.tm_sec, ts.tv_nsec / 1000000 );

  va_start( args, fmt );

  snprintf( buffer, sizeof( buffer ), "[%s][%s:%d] %s() ",
            time_buf, file, line, func );
  vsnprintf( buffer + strlen( buffer ),
             sizeof( buffer ) - strlen( buffer ), fmt, args );
  syslog( priority, "%s", buffer );
  printf( "%s\n", buffer );

  va_end( args );
}

#define LOGE( fmt, ... ) \
  log_to_syslog( LOG_ERR, __FILE__, __LINE__, __FUNCTION__, \
                 fmt, ##__VA_ARGS__ )
#define LOGI( fmt, ... ) \
  log_to_syslog( LOG_INFO, __FILE__, __LINE__, __FUNCTION__, \
                 fmt, ##__VA_ARGS__ )
#define LOGD( fmt, ... ) \
  log_to_syslog( LOG_DEBUG, __FILE__, __LINE__, __FUNCTION__, \
                 fmt, ##__VA_ARGS__ )

#else /* !FEATURE_ENABLE_LOGGING_TO_SYSLOG */

#define LOGE( fmt, ... ) printf( "[ERROR] " fmt "\n", ##__VA_ARGS__ )
#define LOGI( fmt, ... ) printf( "[INFO ] " fmt "\n", ##__VA_ARGS__ )
#define LOGD( fmt, ... ) printf( "[DEBUG] " fmt "\n", ##__VA_ARGS__ )

#endif /* FEATURE_ENABLE_LOGGING_TO_SYSLOG */

/*===========================================================================
                              TYPE DEFINITIONS
===========================================================================*/

typedef unsigned char boolean;
#define TRUE  1
#define FALSE 0

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
  uint8_t  pulse_align_type;      /* 0 = NR5G frame, 1 = UTC second */
  uint8_t  pulse_trigger_action;  /* 0 = Trigger pulse, 1 = Skip pulse */
  uint8_t  pulse_get_cxo_count;   /* 0 = No CXO count, 1 = Get CXO count */
} tns_sync_pulse_config_t;

/*===========================================================================
                              FUNCTION DECLARATIONS
===========================================================================*/

/* Configuration operations */
void tns_config_set_defaults( tns_sync_pulse_config_t *config );

#endif /* __NAS_NR5G_INDICATIONS_H__ */
