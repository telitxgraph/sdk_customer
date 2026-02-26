/******************************************************************************

  @file    nas_nr5g_indications.c
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

#include <ctype.h>

#include "comdef.h"
#include "nas_nr5g_indications.h"

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

/* NR5G service ready synchronization */
static volatile int            g_nr5g_ready = 0;
static pthread_mutex_t         g_nr5g_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t          g_nr5g_cond  = PTHREAD_COND_INITIALIZER;

/* Global sync pulse configuration (set by CLI input) */
static tns_sync_pulse_config_t g_sync_pulse_config;

/*===========================================================================
                     INDICATION CALLBACK - NAS SERVING SYSTEM
===========================================================================*/

/**
  * @brief Decode NAS Serving System indication (registration state monitoring).
  */
static void tns_decode_serving_system_ind
(
  qmi_client_type user_handle,
  unsigned int    msg_id,
  void           *ind_buf,
  unsigned int    ind_buf_len
)
{
  qmi_client_error_type qmi_err;
  nas_serving_system_ind_msg_v01 ss_ind;
  uint32_t i;

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

  LOGI("=== Serving System Indication ===");

  /* Registration State */
  {
    const char *reg_str = "UNKNOWN";
    switch (ss_ind.serving_system.registration_state)
    {
      case 0: reg_str = "NOT_REGISTERED"; break;
      case 1: reg_str = "REGISTERED"; break;
      case 2: reg_str = "NOT_REGISTERED_SEARCHING"; break;
      case 3: reg_str = "REGISTRATION_DENIED"; break;
      case 4: reg_str = "REGISTRATION_UNKNOWN"; break;
      default: break;
    }
    LOGI("  Registration State : %d (%s)", ss_ind.serving_system.registration_state, reg_str);
  }

  /* CS/PS Attach State */
  LOGI("  CS Attach State    : %d (0=Unknown,1=Attached,2=Detached)",
       ss_ind.serving_system.cs_attach_state);
  LOGI("  PS Attach State    : %d (0=Unknown,1=Attached,2=Detached)",
       ss_ind.serving_system.ps_attach_state);

  /* Selected Network */
  LOGI("  Selected Network   : %d (0=Unknown,1=3GPP2,2=3GPP)",
       ss_ind.serving_system.selected_network);

  /* Radio IF list */
  for (i = 0; i < ss_ind.serving_system.radio_if_len && i < NAS_RADIO_IF_LIST_MAX_V01; i++)
  {
    const char *radio_str = "Unknown";
    switch (ss_ind.serving_system.radio_if[i])
    {
      case 0x00: radio_str = "NO_SVC"; break;
      case 0x01: radio_str = "CDMA_1X"; break;
      case 0x02: radio_str = "CDMA_1xEVDO"; break;
      case 0x04: radio_str = "GSM"; break;
      case 0x05: radio_str = "UMTS"; break;
      case 0x08: radio_str = "LTE"; break;
      case 0x09: radio_str = "TDSCDMA"; break;
      case 0x0C: radio_str = "NR5G"; break;
      default: break;
    }
    LOGI("  Radio IF [%u]       : 0x%02X (%s)", i, ss_ind.serving_system.radio_if[i], radio_str);
  }

  /* Roaming Indicator */
  if (ss_ind.roaming_indicator_valid)
  {
    LOGI("  Roaming Indicator  : %d (0=On/Roaming,1=Off/Home)", ss_ind.roaming_indicator);
  }

  /* Current PLMN */
  if (ss_ind.current_plmn_valid)
  {
    LOGI("  PLMN MCC           : %u", ss_ind.current_plmn.mobile_country_code);
    LOGI("  PLMN MNC           : %u", ss_ind.current_plmn.mobile_network_code);
    LOGI("  Network Desc       : %s", ss_ind.current_plmn.network_description);
  }

  /* Data Capabilities */
  if (ss_ind.data_capabilities_valid)
  {
    for (i = 0; i < ss_ind.data_capabilities_len; i++)
    {
      const char *cap_str = "Unknown";
      switch (ss_ind.data_capabilities[i])
      {
        case 0x01: cap_str = "GPRS"; break;
        case 0x02: cap_str = "EDGE"; break;
        case 0x03: cap_str = "HSDPA"; break;
        case 0x04: cap_str = "HSUPA"; break;
        case 0x05: cap_str = "WCDMA"; break;
        case 0x06: cap_str = "CDMA"; break;
        case 0x07: cap_str = "EVDO_REV_0"; break;
        case 0x08: cap_str = "EVDO_REV_A"; break;
        case 0x09: cap_str = "GSM"; break;
        case 0x0A: cap_str = "EVDO_REV_B"; break;
        case 0x0B: cap_str = "LTE"; break;
        case 0x0C: cap_str = "HSDPA+"; break;
        case 0x0D: cap_str = "DC_HSDPA+"; break;
        default: break;
      }
      LOGI("  Data Cap [%u]       : 0x%02X (%s)", i, ss_ind.data_capabilities[i], cap_str);
    }
  }

  /* LAC */
  if (ss_ind.lac_valid)
  {
    LOGI("  LAC                : %u", ss_ind.lac);
  }

  /* Cell ID */
  if (ss_ind.cell_id_valid)
  {
    LOGI("  Cell ID            : %u (0x%X)", ss_ind.cell_id, ss_ind.cell_id);
  }

  /* TAC (LTE) */
  if (ss_ind.tac_valid)
  {
    LOGI("  TAC (LTE)          : %u", ss_ind.tac);
  }

  /* Time Zone */
  if (ss_ind.time_zone_valid)
  {
    LOGI("  Time Zone          : %d (x15 min)", ss_ind.time_zone);
  }

  /* Network Name Source */
  if (ss_ind.nas_3gpp_nw_name_source_valid)
  {
    const char *src_str = "Unknown";
    switch (ss_ind.nas_3gpp_nw_name_source)
    {
      case 0: src_str = "UNKNOWN"; break;
      case 1: src_str = "OPL_PNN"; break;
      case 2: src_str = "CPHS_ONS"; break;
      case 3: src_str = "NITZ"; break;
      case 4: src_str = "SE13"; break;
      case 5: src_str = "MCC_MNC"; break;
      case 6: src_str = "SPN"; break;
      default: break;
    }
    LOGI("  NW Name Source     : %d (%s)", ss_ind.nas_3gpp_nw_name_source, src_str);
  }

  LOGI("=================================");
}

/*===========================================================================
                     INDICATION CALLBACK - NR5G TIME SYNC PULSE
===========================================================================*/

/**
  * @brief Decode and print the NR5G Time Sync Pulse Report indication.
  *        This is called when QMI_NAS_NR5G_TIME_SYNC_PULSE_REPORT_IND_MSG_V01
  *        is received from the modem.
  */
static void tns_decode_nr5g_time_sync_pulse_ind
(
  qmi_client_type user_handle,
  unsigned int    msg_id,
  void           *ind_buf,
  unsigned int    ind_buf_len
)
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
  LOGI("INFO: sfn = %u", pulse_ind.sfn);
 }

 if ( pulse_ind.nta_valid )
 {
  LOGI("INFO: nta = %d", pulse_ind.nta);
 }

 if ( pulse_ind.nta_offset_valid )
 {
  LOGI("INFO: nta_offset = %u", pulse_ind.nta_offset);
 }

 if ( pulse_ind.leapseconds_valid )
 {
  LOGI("INFO: leapseconds = %u", pulse_ind.leapseconds);
 }

 if ( pulse_ind.utc_time_valid )
 {
  LOGI("INFO: utc_time = %llu", (unsigned long long)pulse_ind.utc_time);
 }

 if ( pulse_ind.gps_time_valid )
 {
  LOGI("INFO: gps_time = %llu", (unsigned long long)pulse_ind.gps_time);
 }

 if ( pulse_ind.is_cxo_count_present_valid && pulse_ind.is_cxo_count_present )
 {
  if ( pulse_ind.get_cxo_count_valid )
  {
   LOGI("INFO: cxo_count = %llu", (unsigned long long)pulse_ind.get_cxo_count);
  }
 }

 LOGI("===================================");
}

/*===========================================================================
                     INDICATION CALLBACK - NR5G LOST FRAME SYNC
===========================================================================*/

/**
  * @brief Decode NR5G Lost Frame Sync indication.
  */
static void tns_decode_nr5g_lost_frame_sync_ind
(
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
static void tns_nas_client_ind_cb
(
  qmi_client_type   user_handle,
  unsigned int      msg_id,
  void             *ind_buf,
  unsigned int      ind_buf_len,
  void             *ind_cb_data
)
{
  (void)ind_cb_data;

  LOGI("NAS Indication received: msg_id=0x%04X, len=%u", msg_id, ind_buf_len);

  switch ( msg_id )
  {
    case QMI_NAS_SYS_INFO_IND_MSG_V01:
    {
      qmi_client_error_type qmi_err;
      nas_sys_info_ind_msg_v01 sys_ind;

      memset(&sys_ind, 0, sizeof(sys_ind));
      qmi_err = qmi_client_message_decode(user_handle,
                                           QMI_IDL_INDICATION,
                                           msg_id,
                                           ind_buf, ind_buf_len,
                                           &sys_ind, sizeof(sys_ind));
      if ( QMI_NO_ERR != qmi_err )
      {
        LOGE("Failed to decode SYS_INFO_IND: err=%d", qmi_err);
        break;
      }

      /* Check NR5G service status */
      if ( sys_ind.nr5g_srv_status_info_valid )
      {
        LOGI("[NR5G] Service Status: %d (0=NoSrv,1=Limited,2=Srv)",
             sys_ind.nr5g_srv_status_info.srv_status);

        if ( sys_ind.nr5g_srv_status_info.srv_status == 0x02 ) /* SRV */
        {
          pthread_mutex_lock(&g_nr5g_mutex);
          if ( !g_nr5g_ready )
          {
            g_nr5g_ready = 1;
            LOGI("NR5G service is available, signaling sync pulse thread");
            pthread_cond_signal(&g_nr5g_cond);
          }
          pthread_mutex_unlock(&g_nr5g_mutex);
        }
        else
        {
          pthread_mutex_lock(&g_nr5g_mutex);
          if ( g_nr5g_ready )
          {
            g_nr5g_ready = 0;
            LOGI("NR5G service lost");
          }
          pthread_mutex_unlock(&g_nr5g_mutex);
        }
      }
      break;
    }

    case QMI_NAS_SERVING_SYSTEM_IND_MSG_V01:
      tns_decode_serving_system_ind(user_handle, msg_id, ind_buf, ind_buf_len);
      break;

    default:
      LOGD("Unhandled NAS indication: msg_id=0x%04X", msg_id);
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
   /* NAS indications (e.g. 0x0024, 0x003A, 0x004E) are also delivered here
    * because both clients share the same NAS service. Silently ignore them. */
   LOGD("Ignoring non-sync-pulse indication: msg_id=0x%04X", msg_id);
   break;
 }
}

/*===========================================================================
                     QMI CLIENT ERROR CALLBACK - NAS
===========================================================================*/

/**
  * @brief QMI NAS error callback handler.
  */
static void tns_nas_client_error_cb
(
  qmi_client_type       user_handle,
  qmi_client_error_type error,
  void                 *err_cb_data
)
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
static void tns_sync_pulse_client_error_cb
(
  qmi_client_type       user_handle,
  qmi_client_error_type error,
  void                 *err_cb_data
)
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

  /* Register for serving system indication */
  req_msg.req_serving_system_valid = 1;
  req_msg.req_serving_system = 0x01;

  /* Register for operator name data indication */
  req_msg.reg_operator_name_data_valid = 1;
  req_msg.reg_operator_name_data = 0x01;

  /*  NR5G Time Sync Pulse Report Indication */
  req_msg.reg_nr5g_time_sync_pulse_report_ind_valid = 1;
  req_msg.reg_nr5g_time_sync_pulse_report_ind = 0x01;

  /*  NR5G Lost Sync Frame Indication */
  req_msg.reg_nr5g_lost_sync_frame_ind_valid = 1;
  req_msg.reg_nr5g_lost_sync_frame_ind = 0x01;

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
static int tns_set_nr5g_sync_pulse
(
  qmi_client_type client_handle,
  const tns_sync_pulse_config_t *config
)
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

  /* Register error callback */
  rc = qmi_client_register_error_cb(tns_nas_client_handle,
                                    tns_nas_client_error_cb,
                                    &tns_nas_cb_data);
  if ( rc != QMI_NO_ERR )
  {
    LOGE("NAS error callback registration failed: err=%d", rc);
  }

  /* Register for NAS indications (sys_info, sig_info, serving_system) */
  if ( tns_register_nas_indications( tns_nas_client_handle ) != 0 )
  {
    LOGE("Failed to register NAS indications");
    return NULL;
  }

  /* Keep thread alive to receive NAS indication callbacks */
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

  /* Wait for NR5G service to be ready before configuring sync pulse */
  LOGI("Waiting for NR5G service to become available...");
  {
    pthread_mutex_lock(&g_nr5g_mutex);
    while ( !g_nr5g_ready && g_running )
    {
      struct timespec ts;
      clock_gettime(CLOCK_REALTIME, &ts);
      ts.tv_sec += 5; /* check every 5 seconds */
      pthread_cond_timedwait(&g_nr5g_cond, &g_nr5g_mutex, &ts);
      if ( !g_nr5g_ready && g_running )
      {
        LOGI("Still waiting for NR5G service...");
      }
    }
    pthread_mutex_unlock(&g_nr5g_mutex);
  }

  if ( !g_running )
  {
    LOGI("Shutdown requested before NR5G became available");
    return NULL;
  }

  LOGI("NR5G service ready, configuring sync pulse generation");

  /* Set NR5G sync pulse generation with retry */
  {
    int retry;
    int max_retries = 3;
    for ( retry = 0; retry < max_retries && g_running; retry++ )
    {
      if ( tns_set_nr5g_sync_pulse(tns_sync_pulse_client_handle, &g_sync_pulse_config) == 0 )
      {
        break;
      }
      LOGE("Sync pulse config attempt %d/%d failed, retrying in 3s...", retry + 1, max_retries);
      sleep(3);
    }
  }

  /* Keep thread alive to receive Sync Pulse indication callbacks */
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

/**
  * @brief Read an unsigned integer value from stdin with prompt and range validation.
  * @return The validated input value.
  */
static uint32_t tns_cli_read_uint(const char *prompt, uint32_t min_val, uint32_t max_val)
{
 char buf[64];
 uint32_t val;

 while (1)
 {
  printf("%s", prompt);
  fflush(stdout);

  if (fgets(buf, sizeof(buf), stdin) == NULL)
  {
   LOGE("Failed to read input");
   continue;
  }

  /* Remove trailing newline */
  buf[strcspn(buf, "\n")] = '\0';

  char *endptr = NULL;
  errno = 0;
  unsigned long parsed = strtoul(buf, &endptr, 10);

  if (errno != 0 || endptr == buf || *endptr != '\0')
  {
   printf("Invalid input. Please enter a number between %u and %u.\n", min_val, max_val);
   continue;
  }

  val = (uint32_t)parsed;
  if (val < min_val || val > max_val)
  {
   printf("Out of range. Please enter a value between %u and %u.\n", min_val, max_val);
   continue;
  }

  return val;
 }
}

int main(void)
{
  int rc;
  pthread_t nas_thread;
  pthread_t sync_pulse_thread;

  LOGI("=== TNS (Time Network Synchronization) Application ===");
  LOGI("Monitors NR5G SIB9 time sync via QMI NAS");

  /* Install signal handlers for graceful shutdown */
  signal(SIGINT, tns_signal_handler);
  signal(SIGTERM, tns_signal_handler);

  /* Set defaults for fields not prompted via CLI */
  tns_config_set_defaults(&g_sync_pulse_config);

  /* Interactive CLI input for 3 parameters */
  printf("\n");
  g_sync_pulse_config.pulse_period = tns_cli_read_uint(
    "Enter pulse period (range: 0 - 128, in multiple of 10 milliseconds): ", 0, 128);

  g_sync_pulse_config.start_sfn = tns_cli_read_uint(
    "Enter system frame number (range: 0 - 1024, 1024 = next available sfn): ", 0, 1024);

  g_sync_pulse_config.report_period = tns_cli_read_uint(
    "Enter pulse generation indication periodicity (range: 0 - 128, in multiple of 10 milliseconds, 0 = disabled): ", 0, 128);

  printf("\n");
  LOGI("Configuration: pulse_period=%u, start_sfn=%u, report_period=%u",
       g_sync_pulse_config.pulse_period,
       g_sync_pulse_config.start_sfn,
       g_sync_pulse_config.report_period);

  /* Start NAS QMI thread */
  rc = pthread_create(&nas_thread, NULL, tns_nas_qmi_start, NULL);
  if ( rc != 0 )
  {
    LOGE("NAS pthread_create failed: %d", rc);
    return -1;
  }

  /* Start NR5G Sync Pulse QMI thread */
  rc = pthread_create(&sync_pulse_thread, NULL, tns_sync_pulse_qmi_start, NULL);
  if ( rc != 0 )
  {
    LOGE("Sync Pulse pthread_create failed: %d", rc);
    g_running = 0;
    pthread_join(nas_thread, NULL);
    return -1;
  }

  /* Wait for ENTER key to stop */
  printf("\n(After having set the input, press ENTER to stop)\n\n");
  fflush(stdout);
  getchar();

  /* Signal threads to stop */
  LOGI("ENTER pressed, stopping...");
  g_running = 0;

  /* Wait for both threads to complete */
  pthread_join(nas_thread, NULL);
  pthread_join(sync_pulse_thread, NULL);

  /* Cleanup */
  tns_qmi_release();

  LOGI("TNS application terminated");
  return 0;
}
