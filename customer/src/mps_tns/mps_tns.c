/******************************************************************************

  @file    mps_tns.c
  @brief   TNS (Time Network Synchronization) Application
           Monitors NR5G SIB9 time synchronization via QMI NAS messages.

           Uses:
           - QMI_NAS_INDICATION_REGISTER_REQ_MSG_V01
           - QMI_NAS_SET_NR5G_SYNC_PULSE_GEN_REQ_MSG_V01
           - QMI_NAS_NR5G_TIME_SYNC_PULSE_REPORT_IND_MSG_V01

  ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <signal.h>

#include "comdef.h"
#include "mps_tns.h"

/*===========================================================================
                              GLOBAL VARIABLES
===========================================================================*/

/* NAS QMI client */
static qmi_client_type         tns_nas_client_handle = NULL;
static qmi_client_os_params    tns_nas_os_params;
static int                     tns_nas_cb_data = TNS_CLIENT_CB_DATA;

/* NR5G Sync Pulse QMI client */
static qmi_client_type         tns_sync_pulse_client_handle = NULL;
static qmi_client_os_params    tns_sync_pulse_os_params;
static int                     tns_sync_pulse_cb_data = TNS_CLIENT_CB_DATA;

static volatile int            g_running = 1;

/*===========================================================================
                     INDICATION CALLBACK - NR5G TIME SYNC PULSE
===========================================================================*/

/**
  * @brief Decode and print the NR5G Time Sync Pulse Report indication.
  *        This is called when QMI_NAS_NR5G_TIME_SYNC_PULSE_REPORT_IND_MSG_V01
  *        is received from the modem.
  */
static void tns_decode_nr5g_time_sync_pulse_ind(
	qmi_client_type user_handle,
	unsigned int    msg_id,
	void           *ind_buf,
	unsigned int    ind_buf_len)
{
	qmi_client_error_type qmi_err;
	nas_nr5g_time_sync_pulse_report_ind_msg_v01 pulse_ind;

	memset(&pulse_ind, 0, sizeof(pulse_ind));

	qmi_err = qmi_client_message_decode(user_handle,
	                                     QMI_IDL_INDICATION,
	                                     msg_id,
	                                     ind_buf,
	                                     ind_buf_len,
	                                     &pulse_ind,
	                                     sizeof(pulse_ind));
	if ( QMI_NO_ERR != qmi_err )
	{
		LOGE("Failed to decode NR5G_TIME_SYNC_PULSE_REPORT_IND: err=%d", qmi_err);
		return;
	}

	LOGI("=== NR5G Time Sync Pulse Report ===");

	if ( pulse_ind.sfn_valid )
	{
		LOGI("  SFN            : %u", pulse_ind.sfn);
	}

	if ( pulse_ind.utc_time_valid )
	{
		LOGI("  UTC Time       : %llu ms", (unsigned long long)pulse_ind.utc_time);

		/* Convert to human-readable time */
		time_t sec = (time_t)(pulse_ind.utc_time / 1000);
		struct tm *tm_info = gmtime(&sec);
		if ( tm_info )
		{
			char time_str[64];
			strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", tm_info);
			LOGI("  UTC Time (str) : %s.%03llu UTC",
			     time_str, (unsigned long long)(pulse_ind.utc_time % 1000));
		}
	}

	if ( pulse_ind.gps_time_valid )
	{
		LOGI("  GPS Time       : %llu ms", (unsigned long long)pulse_ind.gps_time);
	}

	if ( pulse_ind.nta_valid )
	{
		LOGI("  NTA            : %d Ts", pulse_ind.nta);
	}

	if ( pulse_ind.nta_offset_valid )
	{
		LOGI("  NTA Offset     : %u Tc", pulse_ind.nta_offset);
	}

	if ( pulse_ind.leapseconds_valid )
	{
		LOGI("  Leap Seconds   : %u", pulse_ind.leapseconds);
	}

	if ( pulse_ind.is_cxo_count_present_valid && pulse_ind.is_cxo_count_present )
	{
		if ( pulse_ind.get_cxo_count_valid )
		{
			LOGI("  CXO Count      : %llu", (unsigned long long)pulse_ind.get_cxo_count);
		}
	}

	LOGI("===================================");
}

/*===========================================================================
                     INDICATION CALLBACK - NAS SERVING SYSTEM
===========================================================================*/

/**
  * @brief Decode NAS Serving System indication (registration state monitoring).
  */
static void tns_decode_serving_system_ind(
	qmi_client_type user_handle,
	unsigned int    msg_id,
	void           *ind_buf,
	unsigned int    ind_buf_len)
{
	qmi_client_error_type qmi_err;
	nas_serving_system_ind_msg_v01 ss_ind;

	memset(&ss_ind, 0, sizeof(ss_ind));

	qmi_err = qmi_client_message_decode(user_handle,
	                                     QMI_IDL_INDICATION,
	                                     msg_id,
	                                     ind_buf,
	                                     ind_buf_len,
	                                     &ss_ind,
	                                     sizeof(ss_ind));
	if ( QMI_NO_ERR != qmi_err )
	{
		LOGE("Failed to decode SERVING_SYSTEM_IND: err=%d", qmi_err);
		return;
	}

	LOGI("Serving System - Registration State: %d",
	     ss_ind.serving_system.registration_state);

	if ( ss_ind.serving_system.registration_state == 0 )
	{
		LOGE("NOT_REGISTERED");
	}
}

/*===========================================================================
                     INDICATION CALLBACK - NR5G LOST FRAME SYNC
===========================================================================*/

/**
  * @brief Decode NR5G Lost Frame Sync indication.
  */
static void tns_decode_nr5g_lost_frame_sync_ind(
  qmi_client_type user_handle,
  unsigned int    msg_id,
  void           *ind_buf,
  unsigned int    ind_buf_len)
{
 	qmi_client_error_type qmi_err;
 	nas_nr5g_lost_frame_sync_ind_msg_v01 lost_sync_ind;

 	memset(&lost_sync_ind, 0, sizeof(lost_sync_ind));

 	qmi_err = qmi_client_message_decode(user_handle,
 	                                     QMI_IDL_INDICATION,
 	                                     msg_id,
 	                                     ind_buf,
 	                                     ind_buf_len,
 	                                     &lost_sync_ind,
 	                                     sizeof(lost_sync_ind));
 	if ( QMI_NO_ERR != qmi_err )
 	{
  		LOGE("Failed to decode NR5G_LOST_FRAME_SYNC_IND: err=%d", qmi_err);
  		return;
 	}

 	if ( lost_sync_ind.nr5g_sync_lost_reason_valid )
 	{
  		const char *reason_str = "UNKNOWN";
  		switch ( lost_sync_ind.nr5g_sync_lost_reason )
  		{
   			case NAS_NR5G_LOST_FRAME_SYNC_RLF_V01:
    				reason_str = "RLF";
    				break;
   			case NAS_NR5G_LOST_FRAME_SYNC_HANDOVER_V01:
    				reason_str = "HANDOVER";
    				break;
   			case NAS_NR5G_LOST_FRAME_SYNC_RESELECTION_V01:
    				reason_str = "RESELECTION";
    				break;
   			case NAS_NR5G_LOST_FRAME_SYNC_OOS_V01:
    				reason_str = "OOS";
    				break;
   			case NAS_NR5G_LOST_FRAME_SYNC_STALE_SIB9_V01:
    				reason_str = "STALE_SIB9";
    				break;
   			case NAS_NR5G_LOST_FRAME_SYNC_NO_SIB9_V01:
    				reason_str = "NO_SIB9";
    				break;
   			default:
   				 break;
  		}
  		LOGE("NR5G Lost Frame Sync: reason=%s (%d)",
  		     reason_str, lost_sync_ind.nr5g_sync_lost_reason_valid);
 	}
}

/*===========================================================================
                     QMI CLIENT INDICATION CALLBACK - NAS
===========================================================================*/

/**
  * @brief QMI NAS indication callback.
  *        Routes NAS indications to the appropriate decoder.
  */
static void tns_nas_client_ind_cb(
	qmi_client_type   user_handle,
	unsigned int      msg_id,
	void             *ind_buf,
	unsigned int      ind_buf_len,
	void             *ind_cb_data)
{
 qmi_client_error_type           qmi_error;
 nas_sig_info_ind_msg_v01        nas_sig_ind;

	(void)ind_cb_data;

	LOGI("NAS Indication received: msg_id=0x%04X, len=%u", msg_id, ind_buf_len);

	switch ( msg_id )
	{
		case QMI_NAS_SERVING_SYSTEM_IND_MSG_V01:
			tns_decode_serving_system_ind(user_handle, msg_id, ind_buf, ind_buf_len);
			break;

		case QMI_NAS_SYS_INFO_IND_MSG_V01:
			LOGI("QMI_NAS_SYS_INFO_IND_MSG_V01");
			break;

		case QMI_NAS_SIG_INFO_IND_MSG_V01:
   LOGI("QMI_NAS_SIG_INFO_IND_MSG_V01");
   qmi_error = qmi_client_message_decode(user_handle,
                                       QMI_IDL_INDICATION,
                                       msg_id,
                                       (void*)ind_buf,
                                       ind_buf_len,
                                       (void*)&nas_sig_ind, 
                                       sizeof(nas_sig_info_ind_msg_v01));
   if(QMI_NO_ERR != qmi_error)
   {
     return;
   }
   
   if(nas_sig_ind.lte_sig_info_valid)
   {
     LOGI("[LTE] RSSI : %hd", nas_sig_ind.lte_sig_info.rssi);
     LOGI("[LTE] RSRQ : %hd", nas_sig_ind.lte_sig_info.rsrq);
     LOGI("[LTE] RSRP : %hd", nas_sig_ind.lte_sig_info.rsrp);
     LOGI("[LTE] SNR  : %hd", nas_sig_ind.lte_sig_info.snr);
   }
   break;

		default:
			LOGI("Unhandled NAS Indication: msg_id=0x%04X", msg_id);
			break;
	}
}

/*===========================================================================
                     QMI CLIENT INDICATION CALLBACK - NR5G SYNC PULSE
===========================================================================*/

/**
  * @brief QMI NR5G Sync Pulse indication callback.
  *        Routes sync pulse indications to the appropriate decoder.
  */
static void tns_sync_pulse_client_ind_cb(
	qmi_client_type   user_handle,
	unsigned int      msg_id,
	void             *ind_buf,
	unsigned int      ind_buf_len,
	void             *ind_cb_data)
{
	(void)ind_cb_data;

	LOGI("Sync Pulse Indication received: msg_id=0x%04X, len=%u", msg_id, ind_buf_len);

	switch ( msg_id )
	{
		case QMI_NAS_NR5G_TIME_SYNC_PULSE_REPORT_IND_MSG_V01:
			tns_decode_nr5g_time_sync_pulse_ind(user_handle, msg_id, ind_buf, ind_buf_len);
			break;

		case QMI_NAS_NR5G_LOST_FRAME_SYNC_IND_MSG_V01:
			tns_decode_nr5g_lost_frame_sync_ind(user_handle, msg_id, ind_buf, ind_buf_len);
			break;

		default:
			LOGI("Unhandled Sync Pulse Indication: msg_id=0x%04X", msg_id);
			break;
	}
}

/*===========================================================================
                     QMI CLIENT ERROR CALLBACK - NAS
===========================================================================*/

/**
  * @brief QMI NAS error callback handler.
  */
static void tns_nas_client_error_cb(
	qmi_client_type       user_handle,
	qmi_client_error_type error,
	void                 *err_cb_data)
{
	(void)user_handle;

	if ( err_cb_data == NULL )
	{
		LOGE("NAS Error callback: null pointer");
		return;
	}

	if ( (int)(*((int *)err_cb_data)) != TNS_CLIENT_CB_DATA )
	{
		LOGE("NAS Error callback: invalid callback data");
		return;
	}

	switch ( error )
	{
		case QMI_SERVICE_ERR:
			LOGE("NAS service is down, releasing client");
			if ( tns_nas_client_handle != NULL )
			{
				if ( QMI_NO_ERR != qmi_client_release(tns_nas_client_handle) )
				{
					LOGE("NAS client release failed");
				}
				tns_nas_client_handle = NULL;
			}
			break;

		default:
			LOGE("NAS client error: %d", error);
			break;
	}
}

/*===========================================================================
                     QMI CLIENT ERROR CALLBACK - NR5G SYNC PULSE
===========================================================================*/

/**
  * @brief QMI NR5G Sync Pulse error callback handler.
  */
static void tns_sync_pulse_client_error_cb(
	qmi_client_type       user_handle,
	qmi_client_error_type error,
	void                 *err_cb_data)
{
	(void)user_handle;

	if ( err_cb_data == NULL )
	{
		LOGE("Sync Pulse Error callback: null pointer");
		return;
	}

	if ( (int)(*((int *)err_cb_data)) != TNS_CLIENT_CB_DATA )
	{
		LOGE("Sync Pulse Error callback: invalid callback data");
		return;
	}

	switch ( error )
	{
		case QMI_SERVICE_ERR:
			LOGE("Sync Pulse service is down, releasing client");
			if ( tns_sync_pulse_client_handle != NULL )
			{
				if ( QMI_NO_ERR != qmi_client_release(tns_sync_pulse_client_handle) )
				{
					LOGE("Sync Pulse client release failed");
				}
				tns_sync_pulse_client_handle = NULL;
			}
			break;

		default:
			LOGE("Sync Pulse client error: %d", error);
			break;
	}
}

/*===========================================================================
              REGISTER FOR NAS INDICATIONS
===========================================================================*/

/**
  * @brief Register for NAS indications (sys_info, sig_info, serving_system).
  * @return 0 on success, -1 on failure.
  */
static int tns_register_nas_indications(qmi_client_type client_handle)
{
	qmi_client_error_type qmi_err;
	nas_indication_register_req_msg_v01  req_msg;
	nas_indication_register_resp_msg_v01 resp_msg;

	LOGI("Registering for NAS indications...");

	memset(&req_msg, 0, sizeof(req_msg));
	memset(&resp_msg, 0, sizeof(resp_msg));

	/* Register for system info indication */
	req_msg.sys_info_valid = 1;
	req_msg.sys_info = 0x01;

	/* Register for signal info indication */
	req_msg.sig_info_valid = 1;
	req_msg.sig_info = 0x01;

	qmi_err = qmi_client_send_msg_sync(client_handle,
	                                    QMI_NAS_INDICATION_REGISTER_REQ_MSG_V01,
	                                    (void *)&req_msg,
	                                    sizeof(req_msg),
	                                    (void *)&resp_msg,
	                                    sizeof(resp_msg),
	                                    TNS_SEND_TIMEOUT);

	if ( qmi_err != QMI_NO_ERR )
	{
		LOGE("NAS indication register failed: err=%d", qmi_err);
		return -1;
	}

	if ( resp_msg.resp.result != QMI_RESULT_SUCCESS_V01 )
	{
		LOGE("NAS indication register response error: result=%d, error=0x%x",
		     resp_msg.resp.result, resp_msg.resp.error);
		return -1;
	}

	LOGI("NAS indication registration successful");
	return 0;
}

/*===========================================================================
              SET NR5G SYNC PULSE GENERATION
===========================================================================*/

/**
  * @brief Configure NR5G sync pulse generation on the modem.
  *        Reads settings from the config file.
  * @return 0 on success, -1 on failure.
  */
static int tns_set_nr5g_sync_pulse(qmi_client_type client_handle, const tns_sync_pulse_config_t *config)
{
	qmi_client_error_type qmi_err;
	nas_set_nr5g_sync_pulse_gen_req_msg_v01  req_msg;
	nas_set_nr5g_sync_pulse_gen_resp_msg_v01 resp_msg;

	LOGI("Setting NR5G sync pulse generation...");

	memset(&req_msg, 0, sizeof(req_msg));
	memset(&resp_msg, 0, sizeof(resp_msg));

	/* Mandatory: pulse period */
	req_msg.pulse_period = config->pulse_period;
	LOGI("  pulse_period        = %u (x10ms)", req_msg.pulse_period);

	/* Optional: start SFN */
	req_msg.start_sfn_valid = 1;
	req_msg.start_sfn = config->start_sfn;
	LOGI("  start_sfn           = %u", req_msg.start_sfn);

	/* Optional: report period */
	req_msg.report_period_valid = 1;
	req_msg.report_period = config->report_period;
	LOGI("  report_period       = %u (x10ms)", req_msg.report_period);

	/* Optional: pulse alignment type */
	req_msg.pulse_align_type_valid = 1;
	req_msg.pulse_align_type = config->pulse_align_type;
	LOGI("  pulse_align_type    = %u (%s)", req_msg.pulse_align_type,
	     req_msg.pulse_align_type == 0 ? "NR5G frame" : "UTC second");

	/* Optional: pulse trigger action */
	req_msg.pulse_trigger_action_valid = 1;
	req_msg.pulse_trigger_action = config->pulse_trigger_action;
	LOGI("  pulse_trigger_action= %u (%s)", req_msg.pulse_trigger_action,
	     req_msg.pulse_trigger_action == 0 ? "Trigger" : "Skip");

	/* Optional: CXO count */
	req_msg.pulse_get_cxo_count_valid = 1;
	req_msg.pulse_get_cxo_count = config->pulse_get_cxo_count;
	LOGI("  pulse_get_cxo_count = %u", req_msg.pulse_get_cxo_count);

	qmi_err = qmi_client_send_msg_sync(client_handle,
	                                    QMI_NAS_SET_NR5G_SYNC_PULSE_GEN_REQ_MSG_V01,
	                                    (void *)&req_msg,
	                                    sizeof(req_msg),
	                                    (void *)&resp_msg,
	                                    sizeof(resp_msg),
	                                    TNS_SEND_TIMEOUT);

	if ( qmi_err != QMI_NO_ERR )
	{
		LOGE("SET_NR5G_SYNC_PULSE_GEN failed: err=%d", qmi_err);
		return -1;
	}

	if ( resp_msg.resp.result != QMI_RESULT_SUCCESS_V01 )
	{
		LOGE("SET_NR5G_SYNC_PULSE_GEN response error: result=%d, error=0x%x",
		     resp_msg.resp.result, resp_msg.resp.error);
		return -1;
	}

	LOGI("NR5G sync pulse generation configured successfully");
	return 0;
}

/*===========================================================================
              NAS QMI INITIALIZATION
===========================================================================*/

/**
  * @brief Initialize QMI NAS client and register for NAS indications.
  *        Handles serving system, sys_info, sig_info events.
  */
static void *tns_nas_qmi_start(void *arg)
{
	qmi_client_error_type rc;
	qmi_idl_service_object_type nas_service_object;

	(void)arg;

	LOGI("TNS NAS QMI initialization starting...");

	/* Get the NAS service object */
	nas_service_object = nas_get_service_object_v01();
	if ( NULL == nas_service_object )
	{
		LOGE("NAS service object not available");
		return NULL;
	}
	LOGI("NAS service object acquired");

	/* Initialize QMI NAS client */
	memset(&tns_nas_os_params, 0, sizeof(tns_nas_os_params));

	rc = qmi_client_init_instance(nas_service_object,
	                               QMI_CLIENT_INSTANCE_ANY,
	                               tns_nas_client_ind_cb,
	                               NULL,
	                               &tns_nas_os_params,
	                               TNS_SEND_TIMEOUT,
	                               &tns_nas_client_handle);

	if ( rc != QMI_NO_ERR )
	{
		LOGE("QMI NAS client init failed: err=%d", rc);
		return NULL;
	}
	LOGI("QMI NAS client initialized");

	/* Register error callback */
	rc = qmi_client_register_error_cb(tns_nas_client_handle,
	                                   tns_nas_client_error_cb,
	                                   &tns_nas_cb_data);
	if ( rc != QMI_NO_ERR )
	{
		LOGE("NAS error callback registration failed: err=%d", rc);
	}

	/* Register for NAS indications (sys_info, sig_info, serving_system) */
	if ( tns_register_nas_indications(tns_nas_client_handle) != 0 )
	{
		LOGE("Failed to register NAS indications");
		return NULL;
	}

	/* Keep thread alive to receive NAS indication callbacks */
	LOGI("NAS indication thread running...");
	while ( g_running )
	{
		sleep(1);
	}

	LOGI("NAS indication thread exited");
	return NULL;
}

/*===========================================================================
              NR5G SYNC PULSE QMI INITIALIZATION
===========================================================================*/

/**
  * @brief Initialize QMI NR5G Sync Pulse client.
  *        Configures pulse generation and waits for indication callbacks.
  */
static void *tns_sync_pulse_qmi_start(void *arg)
{
	qmi_client_error_type rc;
	qmi_idl_service_object_type nas_service_object;
	tns_sync_pulse_config_t config;

	(void)arg;

	LOGI("TNS NR5G Sync Pulse QMI initialization starting...");

	/* Get the NAS service object */
	nas_service_object = nas_get_service_object_v01();
	if ( NULL == nas_service_object )
	{
		LOGE("NAS service object not available (sync pulse)");
		return NULL;
	}
	LOGI("NAS service object acquired (sync pulse)");

	/* Initialize QMI client for sync pulse */
	memset(&tns_sync_pulse_os_params, 0, sizeof(tns_sync_pulse_os_params));

	rc = qmi_client_init_instance(nas_service_object,
	                               QMI_CLIENT_INSTANCE_ANY,
	                               tns_sync_pulse_client_ind_cb,
	                               NULL,
	                               &tns_sync_pulse_os_params,
	                               TNS_SEND_TIMEOUT,
	                               &tns_sync_pulse_client_handle);

	if ( rc != QMI_NO_ERR )
	{
		LOGE("QMI Sync Pulse client init failed: err=%d", rc);
		return NULL;
	}
	LOGI("QMI Sync Pulse client initialized");

	/* Register error callback */
	rc = qmi_client_register_error_cb(tns_sync_pulse_client_handle,
	                                   tns_sync_pulse_client_error_cb,
	                                   &tns_sync_pulse_cb_data);
	if ( rc != QMI_NO_ERR )
	{
		LOGE("Sync Pulse error callback registration failed: err=%d", rc);
	}

	/* Load config and set NR5G sync pulse generation */
	if ( tns_config_load(TNS_CONFIG_FILE_PATH, &config) != 0 )
	{
		LOGI("Using default sync pulse configuration");
	}

	if ( tns_set_nr5g_sync_pulse(tns_sync_pulse_client_handle, &config) != 0 )
	{
		LOGE("Failed to set NR5G sync pulse generation");
		/* Continue running - indications may still come */
	}

	/* Keep thread alive to receive Sync Pulse indication callbacks */
	LOGI("Sync Pulse indication thread running...");
	while ( g_running )
	{
		sleep(1);
	}

	LOGI("Sync Pulse indication thread exited");
	return NULL;
}

/*===========================================================================
              CLEANUP
===========================================================================*/

/**
  * @brief Release both QMI NAS and Sync Pulse clients.
  */
static void tns_qmi_release(void)
{
	int rc;

	/* Stop sync pulse generation before exiting */
	if ( tns_sync_pulse_client_handle != NULL )
	{
		nas_set_nr5g_sync_pulse_gen_req_msg_v01  stop_req;
		nas_set_nr5g_sync_pulse_gen_resp_msg_v01 stop_resp;

		memset(&stop_req, 0, sizeof(stop_req));
		memset(&stop_resp, 0, sizeof(stop_resp));
		stop_req.pulse_period = 0; /* 0 = stop pulse generation */

		LOGI("Stopping NR5G sync pulse generation...");
		qmi_client_send_msg_sync(tns_sync_pulse_client_handle,
		                          QMI_NAS_SET_NR5G_SYNC_PULSE_GEN_REQ_MSG_V01,
		                          (void *)&stop_req, sizeof(stop_req),
		                          (void *)&stop_resp, sizeof(stop_resp),
		                          TNS_SEND_TIMEOUT);
	}

	/* Release Sync Pulse client */
	if ( tns_sync_pulse_client_handle != NULL )
	{
		rc = qmi_client_release(tns_sync_pulse_client_handle);
		if ( rc < 0 )
		{
			LOGE("QMI Sync Pulse client release failed");
		}
		else
		{
			LOGI("QMI Sync Pulse client released");
		}
		tns_sync_pulse_client_handle = NULL;
	}

	/* Release NAS client */
	if ( tns_nas_client_handle != NULL )
	{
		rc = qmi_client_release(tns_nas_client_handle);
		if ( rc < 0 )
		{
			LOGE("QMI NAS client release failed");
		}
		else
		{
			LOGI("QMI NAS client released");
		}
		tns_nas_client_handle = NULL;
	}
}

/*===========================================================================
              SIGNAL HANDLER
===========================================================================*/

static void tns_signal_handler(int sig)
{
	LOGI("Signal %d received, shutting down...", sig);
	g_running = 0;
}

/*===========================================================================
              MAIN
===========================================================================*/

int main(void)
{
	int rc;
	pthread_t nas_thread;
	pthread_t sync_pulse_thread;

	LOGI("=== TNS (Time Network Synchronization) Application ===");
	LOGI("Monitors NR5G SIB9 time sync via QMI NAS");
	LOGI("Config file: %s", TNS_CONFIG_FILE_PATH);

	/* Install signal handlers for graceful shutdown */
	signal(SIGINT, tns_signal_handler);
	signal(SIGTERM, tns_signal_handler);

	while ( g_running )
	{
		/* Start NAS QMI thread */
		rc = pthread_create(&nas_thread, NULL, tns_nas_qmi_start, NULL);
		if ( rc != 0 )
		{
			LOGE("NAS pthread_create failed: %d", rc);
			sleep(1);
			continue;
		}

		/* Start NR5G Sync Pulse QMI thread */
		rc = pthread_create(&sync_pulse_thread, NULL, tns_sync_pulse_qmi_start, NULL);
		if ( rc != 0 )
		{
			LOGE("Sync Pulse pthread_create failed: %d", rc);
			g_running = 0;
			pthread_join(nas_thread, NULL);
			break;
		}

		/* Wait for both threads to complete */
		rc = pthread_join(nas_thread, NULL);
		if ( rc != 0 )
		{
			LOGE("NAS pthread_join failed: %d", rc);
		}

		rc = pthread_join(sync_pulse_thread, NULL);
		if ( rc != 0 )
		{
			LOGE("Sync Pulse pthread_join failed: %d", rc);
		}

		if ( g_running )
		{
			LOGI("QMI threads exited, restarting in 5 seconds...");
			sleep(5);
		}
	}

	/* Cleanup */
	tns_qmi_release();

	LOGI("TNS application terminated");
	return 0;
}
