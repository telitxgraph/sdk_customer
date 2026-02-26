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
#include <sys/ipc.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/msg.h>
#include <linux/netlink.h>
#include <sys/ioctl.h>
#include <asm/ioctls.h>
#include "comdef.h"

#include "qmi_idl_lib.h"
#include "qmi_csi.h"
#include "qmi_sap.h"
#include "qmi_client.h"
#include "network_access_service_v01.h"

#include "mps_qmi_test.h"
#include "gms.h"

#include <ctype.h>

#define min(a,b)  ((a) < (b) ? (a):(b))
#define SEND_TIMEOUT  50000

qmi_client_type user_handle_nas=NULL;

/*qmi message library handle*/

/*===========================================================================
                              FUNCTION DECLARATIONS
===========================================================================*/

#define NAS_TEST_CLIENT_CB_DATA 0xDEAD
static int client_cb_data = NAS_TEST_CLIENT_CB_DATA;

/*---------------------------------------------------------------------------
   Notifier OS param
---------------------------------------------------------------------------*/
static qmi_client_os_params          notifier_os_params;

/*---------------------------------------------------------------------------
   Client OS param
---------------------------------------------------------------------------*/
static qmi_client_os_params          client_os_params;


/*===========================================================================
                              VARIABLE DECLARATIONS
===========================================================================*/

/*qmi message library handle*/
qmi_client_type               client_handle = NULL;
qmi_idl_service_object_type   nas_service_object;

qmi_client_os_params   *os_params;
qmi_client_type        *user_handle;
void                   *ind_cb_data;
uint32_t                timeout;

char* log_file = "syslog_mps_qmi_test";

static qmi_client_os_params    nas_os_params_main;
static qmi_client_type         nas_notifier_main;

/*---------------------------------------------------------------------------
   QMI Notifier handler
---------------------------------------------------------------------------*/
static qmi_client_type notifier_handle = NULL;

/*===========================================================================
                              FUNCTION DEFINITIONS
===========================================================================*/

void nas_qmi_test_client_error_cb
(
  qmi_client_type user_handle,
  qmi_client_error_type error,
  void *err_cb_data
);

static void  qmi_nas_client_test_ind_cb
(
  qmi_client_type   user_handle,
  unsigned int      msg_id,
  void             *ind_buf,
  unsigned int      ind_buf_len,
  void             *ind_cb_data
);

int qmi_nas_client_test_init( void );
void* mps_qmi_test_start_func( void* arg );


/*===========================================================================
                             HELPER FUNCTIONS
===========================================================================*/

/**
 * @brief Format PLMN MCC-MNC string safely.
 *        MNC can be 2 or 3 digits; the 3rd byte may be garbage when 2-digit.
 *
 * @param[out] buf     Output buffer (min 8 bytes: "XXX-XXX\0")
 * @param[in]  mcc     MCC char array (3 bytes, not null-terminated)
 * @param[in]  mnc     MNC char array (3 bytes, not null-terminated)
 */
static void nas_format_plmn(char *buf, const char *mcc, const char *mnc)
{
  int mnc_len = (mnc[2] >= '0' && mnc[2] <= '9') ? 3 : 2;
  snprintf(buf, 8, "%.3s-%.*s", mcc, mnc_len, mnc);
}

/**
 * @brief Convert network name bytes to printable ASCII string.
 *        Handles GSM 7-bit packed alphabet and UCS2 coding schemes.
 *
 * @param[out] out            Output buffer
 * @param[in]  out_size       Size of output buffer
 * @param[in]  data           Raw name bytes
 * @param[in]  data_len       Length of raw name data in bytes
 * @param[in]  coding_scheme  0=GSM 7-bit packed, 1=UCS2
 * @param[in]  spare_bits     Number of spare bits in last octet (nas_spare_bits_enum_v01)
 */
static void nas_convert_nw_name(char *out, size_t out_size,
                                const uint8_t *data, uint32_t data_len,
                                nas_coding_scheme_enum_v01 coding_scheme,
                                nas_spare_bits_enum_v01 spare_bits)
{
  size_t pos = 0;
  uint32_t i;

  if (out_size == 0)
    return;

  if (coding_scheme == NAS_CODING_SCHEME_UCS2_V01)
  {
    /* UCS2: 2 bytes per char (big-endian). Extract ASCII-range chars,
     * replace non-ASCII with '?' */
    for (i = 0; i + 1 < data_len && pos + 1 < out_size; i += 2)
    {
      uint16_t ucs2_char = ((uint16_t)data[i] << 8) | data[i + 1];
      if (ucs2_char == 0)
        break;
      if (ucs2_char >= 0x20 && ucs2_char <= 0x7E)
        out[pos++] = (char)ucs2_char;
      else
        out[pos++] = '?';
    }
  }
  else
  {
    /* GSM 7-bit packed (CBS default alphabet per 3GPP TS 23.038).
     * Characters are packed 7 bits each into octets.
     * spare_bits indicates unused bits in the last byte. */
    uint32_t total_bits;
    uint32_t num_chars;
    uint32_t sb = (uint32_t)spare_bits; /* 0=unknown, 1-7 = # of spare bits */

    if (sb > 7) sb = 0;
    total_bits = data_len * 8;
    if (sb > 0 && sb <= total_bits)
      total_bits -= sb;
    num_chars = total_bits / 7;

    for (i = 0; i < num_chars && pos + 1 < out_size; i++)
    {
      uint32_t bit_offset = i * 7;
      uint32_t byte_idx = bit_offset / 8;
      uint32_t bit_idx  = bit_offset % 8;
      uint8_t ch;

      ch = (data[byte_idx] >> bit_idx) & 0x7F;
      if (bit_idx > 1 && (byte_idx + 1) < data_len)
        ch |= (data[byte_idx + 1] << (8 - bit_idx)) & 0x7F;

      /* GSM 7-bit alphabet: most printable ASCII chars map 1:1.
       * Map CR (0x0D) to space, skip non-printable. */
      if (ch == 0x0D)
        out[pos++] = ' ';
      else if (ch >= 0x20 && ch <= 0x7E)
        out[pos++] = (char)ch;
      else if (ch == 0x00)
        break; /* padding / end */
      else
        out[pos++] = '.';
    }
   }
   out[pos] = '\0';
}

/*===========================================================================
                     INDICATION CALLBACK - NAS SERVING SYSTEM
===========================================================================*/

/**
  * @brief Decode NAS Serving System indication (registration state monitoring).
  */
static void nas_decode_serving_system_ind
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

static void event_nas_ind (qmi_client_type user_handle,
                               unsigned int    msg_id,
                               void            *ind_buf,
                               unsigned int    ind_buf_len,
                               void            *ind_cb_data
                              )

{
  qmi_client_error_type                 qmi_err;
  nas_serving_system_ind_msg_v01        system_indication;
  (void) ind_cb_data;

  LOGI("ind msg_id: %u", msg_id);

  switch(msg_id)
  {
    case QMI_NAS_SERVING_SYSTEM_IND_MSG_V01:
      LOGI("QMI_NAS_SERVING_SYSTEM_IND_MSG_V01");
      qmi_err = qmi_client_message_decode(user_handle,
                                          QMI_IDL_INDICATION,
                                          msg_id,
                                          ind_buf,
                                          ind_buf_len,
                                          &system_indication,
                                          sizeof(system_indication));
      if(QMI_NO_ERR != qmi_err)
      {
        LOGE("Invalid filter mode ind msg error %d", qmi_err);
      }

      LOGI("registration STATUS:%d", system_indication.serving_system.registration_state);
    break;

    default:
      LOGI("NOT PROCESSED MSG_ID: %u", msg_id);
      break;
  }
}

static void cb_nas_indication(qmi_client_type       wms_user_handle,
                                  unsigned int          msg_id,
                                  void                  *resp_c_struct,
                                  unsigned int          resp_c_struct_len,
                                  void                  *resp_cb_data,
                                  qmi_client_error_type transp_err
                                 )
{
  if(transp_err != QMI_NO_ERR)
  {
    /* no mandatory field */
    LOGE("NAS_INDICATION_REQUEST_CALLBACK WITH ERROR %d", transp_err);
  }
  else
  {
    LOGE("cb_nas_indication %d", transp_err);
  }
}
static void register_for_nas_indication(qmi_client_type client_handle)
{
  qmi_client_error_type                qmi_err;
  qmi_txn_handle                       txn_handle;

  LOGI("register_for_nas_indication");

  // register
  nas_indication_register_req_msg_v01  nas_ind_reg_req_msg;
  nas_indication_register_resp_msg_v01 nas_ind_reg_resp_msg;
  memset(&nas_ind_reg_req_msg, 0, sizeof(nas_indication_register_req_msg_v01));
  memset(&nas_ind_reg_resp_msg, 0, sizeof(nas_indication_register_resp_msg_v01));

  // sig_info
  nas_config_sig_info2_req_msg_v01 *config_sig_info_req = NULL;
  nas_config_sig_info2_resp_msg_v01 *config_sig_info_resp = NULL;

  config_sig_info_req = (nas_config_sig_info2_req_msg_v01 *)malloc(sizeof(nas_config_sig_info2_req_msg_v01));
  if(config_sig_info_req == NULL)
  {
    LOGE("Failed to allocate memory for nas_config_sig_info2_req_msg_v01");
    return;
  }

  config_sig_info_resp = (nas_config_sig_info2_resp_msg_v01 *)malloc(sizeof(nas_config_sig_info2_resp_msg_v01));
  if(config_sig_info_resp == NULL)
  {
    free(config_sig_info_req);
    config_sig_info_req = NULL;
    LOGE("Failed to allocate memory for nas_config_sig_info2_resp_msg_v01");
    return;
  }

  memset(config_sig_info_req, 0, sizeof(nas_config_sig_info2_req_msg_v01));
  memset(config_sig_info_resp, 0, sizeof(nas_config_sig_info2_resp_msg_v01));

  // register
  nas_ind_reg_req_msg.sys_info_valid = 1;
  nas_ind_reg_req_msg.sys_info = 0x01;
  nas_ind_reg_req_msg.sig_info_valid  = 1;
  nas_ind_reg_req_msg.sig_info = 0x01;

  qmi_err = qmi_client_send_msg_sync(client_handle, QMI_NAS_INDICATION_REGISTER_REQ_MSG_V01,
                                    (void *)&nas_ind_reg_req_msg, sizeof(nas_indication_register_req_msg_v01),
                                    (void *)&nas_ind_reg_resp_msg, sizeof(nas_indication_register_resp_msg_v01),
                                    SEND_TIMEOUT);
 
  if(qmi_err != QMI_NO_ERR)
  {
    LOGE("Indication register request failed :%d", qmi_err);
    goto end_of_iotapp_qmi_nas_init;
  }
  LOGI("Indication register request is success.");

  // sig_info
  config_sig_info_req->lte_rsrq_delta_valid = 1;
  config_sig_info_req->lte_rsrq_delta = 0x0a;
  qmi_err = qmi_client_send_msg_sync(client_handle, QMI_NAS_CONFIG_SIG_INFO2_REQ_MSG_V01,
                                    (void *)config_sig_info_req, sizeof(nas_config_sig_info2_req_msg_v01),
                                    (void *)config_sig_info_resp, sizeof(nas_config_sig_info2_resp_msg_v01),
                                    SEND_TIMEOUT);
 
  if(qmi_err != QMI_NO_ERR)
  {
    LOGE("Config_Sig_Info2 failed :%d",qmi_err);
    goto end_of_iotapp_qmi_nas_init;
  }
  LOGI("Config_Sig_Info2 is success.");

end_of_iotapp_qmi_nas_init:
  free(config_sig_info_req);
  free(config_sig_info_resp);
  config_sig_info_req = NULL;
  config_sig_info_resp = NULL;
  
  return ; 
}


/*===========================================================================
FUNCTION       ECALL_QMI_LOC_CLIENT_ERROR_CB

DESCRIPTION 
  QMI_LOC error callback handler. This callback is called by QCCI
  to notify error.

DEPENDENCIES 
  FEATURE_ECALL_HAS_QMI_LOC 

RETURN VALUE 
  None 

SIDE EFFECTS 
  None 
===========================================================================*/
void nas_qmi_test_client_error_cb
(
  qmi_client_type user_handle,
  qmi_client_error_type error,
  void *err_cb_data
)
{
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

  /*---------------------------------------------------------------------
    Check if err cb data pointer is valid
  ---------------------------------------------------------------------*/
  if(err_cb_data == NULL)
  {
    LOGE("Null pointer passed");
    return;
  }

  /*---------------------------------------------------------------------
    Check if calback data is valid
  ---------------------------------------------------------------------*/
  if((int)(*((int*)err_cb_data)) != NAS_TEST_CLIENT_CB_DATA)
  {
    LOGE("Invalid callback data");
    return;
  }

  switch(error)
  {
    /*-----------------------------------------------------------------------
      In case of service error, client will be released. No attempt to recover
      the QMI connection will be made.
    ------------------------------------------------------------------------*/
    case QMI_SERVICE_ERR:
      LOGE("service is down");

      /*-----------------------------------------------------------------------
        Release the client.
      ------------------------------------------------------------------------*/
      if(QMI_NO_ERR != qmi_client_release(user_handle))
      {
        LOGE("Client release failed");
        return;
      }

      user_handle = NULL;
      break;

    default:
      LOGE("default");
      break;
  }

  return;
}

/*===========================================================================
                     QMI CLIENT INDICATION CALLBACK - NAS
===========================================================================*/

/**
  * @brief QMI NAS indication callback.
  *        Routes NAS indications to the appropriate decoder.
  */
static void qmi_nas_client_test_ind_cb(
  qmi_client_type   user_handle,
  unsigned int      msg_id,
  void             *ind_buf,
  unsigned int      ind_buf_len,
  void             *ind_cb_data)
{
  qmi_client_error_type           qmi_error;
  nas_sig_info_ind_msg_v01        nas_sig_ind;
  nas_sys_info_ind_msg_v01        nas_sys_ind;

  (void)ind_cb_data;

  LOGI("NAS Indication received: msg_id=0x%04X, len=%u", msg_id, ind_buf_len);

  switch ( msg_id )
  {
    case QMI_NAS_SERVING_SYSTEM_IND_MSG_V01:
      nas_decode_serving_system_ind(user_handle, msg_id, ind_buf, ind_buf_len);
      break;

    case QMI_NAS_SYS_INFO_IND_MSG_V01:
      LOGI("QMI_NAS_SYS_INFO_IND_MSG_V01");
      memset(&nas_sys_ind, 0, sizeof(nas_sys_ind));
      qmi_error = qmi_client_message_decode(user_handle,
                                             QMI_IDL_INDICATION,
                                             msg_id,
                                             (void*)ind_buf,
                                             ind_buf_len,
                                             (void*)&nas_sys_ind,
                                             sizeof(nas_sys_info_ind_msg_v01));
     if (QMI_NO_ERR != qmi_error)
     {
       LOGE("Failed to decode SYS_INFO_IND: err=%d", qmi_error);
       return;
     }

     LOGI("=== System Info Indication ===");

    /* --- LTE Service Status --- */
    if (nas_sys_ind.lte_srv_status_info_valid)
    {
     LOGI("[LTE] Service Status   : %d (0=NoSrv,1=Limited,2=Srv,3=LimitedRegional,4=PwrSave)",
          nas_sys_ind.lte_srv_status_info.srv_status);
     LOGI("[LTE] True Srv Status  : %d", nas_sys_ind.lte_srv_status_info.true_srv_status);
    }

    /* --- LTE System Info --- */
    if (nas_sys_ind.lte_sys_info_valid)
    {
     if (nas_sys_ind.lte_sys_info.common_sys_info.srv_domain_valid)
     {
      LOGI("[LTE] Service Domain   : %d (0=NoSrv,1=CS,2=PS,3=CS_PS,4=Camped)",
           nas_sys_ind.lte_sys_info.common_sys_info.srv_domain);
     }
     if (nas_sys_ind.lte_sys_info.common_sys_info.roam_status_valid)
     {
      LOGI("[LTE] Roaming Status   : %d (0=Off,1=On)",
           nas_sys_ind.lte_sys_info.common_sys_info.roam_status);
     }
     if (nas_sys_ind.lte_sys_info.threegpp_specific_sys_info.network_id_valid)
     {
      char plmn_buf[8];
      nas_format_plmn(plmn_buf,
                      nas_sys_ind.lte_sys_info.threegpp_specific_sys_info.network_id.mcc,
                      nas_sys_ind.lte_sys_info.threegpp_specific_sys_info.network_id.mnc);
      LOGI("[LTE] PLMN (MCC-MNC)   : %s", plmn_buf);
     }
     if (nas_sys_ind.lte_sys_info.lte_specific_sys_info.tac_valid)
     {
      LOGI("[LTE] TAC              : %u", nas_sys_ind.lte_sys_info.lte_specific_sys_info.tac);
     }
    }

    /* --- NR5G Service Status --- */
    if (nas_sys_ind.nr5g_srv_status_info_valid)
    {
     LOGI("[NR5G] Service Status  : %d (0=NoSrv,1=Limited,2=Srv,3=LimitedRegional,4=PwrSave)",
          nas_sys_ind.nr5g_srv_status_info.srv_status);
     LOGI("[NR5G] True Srv Status : %d", nas_sys_ind.nr5g_srv_status_info.true_srv_status);
    }

    /* --- NR5G System Info --- */
    if (nas_sys_ind.nr5g_sys_info_valid)
    {
     if (nas_sys_ind.nr5g_sys_info.common_sys_info.srv_domain_valid)
     {
      LOGI("[NR5G] Service Domain  : %d (0=NoSrv,1=CS,2=PS,3=CS_PS,4=Camped)",
           nas_sys_ind.nr5g_sys_info.common_sys_info.srv_domain);
     }
     if (nas_sys_ind.nr5g_sys_info.common_sys_info.srv_capability_valid)
     {
      LOGI("[NR5G] Srv Capability  : %d", nas_sys_ind.nr5g_sys_info.common_sys_info.srv_capability);
     }
     if (nas_sys_ind.nr5g_sys_info.common_sys_info.roam_status_valid)
     {
      LOGI("[NR5G] Roaming Status  : %d (0=Off,1=On)",
           nas_sys_ind.nr5g_sys_info.common_sys_info.roam_status);
     }
     if (nas_sys_ind.nr5g_sys_info.threegpp_specific_sys_info.network_id_valid)
     {
      char plmn_buf[8];
      nas_format_plmn(plmn_buf,
                      nas_sys_ind.nr5g_sys_info.threegpp_specific_sys_info.network_id.mcc,
                      nas_sys_ind.nr5g_sys_info.threegpp_specific_sys_info.network_id.mnc);
      LOGI("[NR5G] PLMN (MCC-MNC)  : %s", plmn_buf);
     }
    }

    /* --- NR5G Cell Access Status --- */
    if (nas_sys_ind.nr5g_cell_status_valid)
    {
     LOGI("[NR5G] Cell Status     : %d (0=NormalOnly,1=EmergOnly,2=NoCalls,3=AllCalls)",
          nas_sys_ind.nr5g_cell_status);
    }

    /* --- NR5G TAC --- */
    if (nas_sys_ind.nr5g_tac_info_valid)
    {
     uint32_t tac_val = ((uint32_t)nas_sys_ind.nr5g_tac_info.tac[0] << 16) |
                        ((uint32_t)nas_sys_ind.nr5g_tac_info.tac[1] << 8) |
                        ((uint32_t)nas_sys_ind.nr5g_tac_info.tac[2]);
     LOGI("[NR5G] TAC             : %u (0x%06X)", tac_val, tac_val);
    }

    /* --- NR5G PCI --- */
    if (nas_sys_ind.nr5g_pci_valid)
    {
     LOGI("[NR5G] PCI             : %u", nas_sys_ind.nr5g_pci);
    }

    /* --- NR5G Cell ID --- */
    if (nas_sys_ind.nr5g_cell_id_valid)
    {
     LOGI("[NR5G] Cell ID         : %llu", (unsigned long long)nas_sys_ind.nr5g_cell_id);
    }

    /* --- NR5G ARFCN --- */
    if (nas_sys_ind.nr5g_arfcn_valid)
    {
     LOGI("[NR5G] ARFCN           : %u", nas_sys_ind.nr5g_arfcn);
    }

    /* --- NR5G Frequency Type --- */
    if (nas_sys_ind.nr5g_freq_type_valid)
    {
     LOGI("[NR5G] Freq Type       : %d (%s)",
          nas_sys_ind.nr5g_freq_type,
          nas_sys_ind.nr5g_freq_type == 0 ? "Sub6" : "mmWave");
    }

    /* --- NR5G Subcarrier Spacing --- */
    if (nas_sys_ind.nr5g_subcarrier_spacing_valid)
    {
     const char *scs_str = "Unknown";
     switch (nas_sys_ind.nr5g_subcarrier_spacing)
     {
      case 0: scs_str = "15 KHz"; break;
      case 1: scs_str = "30 KHz"; break;
      case 2: scs_str = "60 KHz"; break;
      case 3: scs_str = "120 KHz"; break;
      case 4: scs_str = "240 KHz"; break;
      default: break;
     }
     LOGI("[NR5G] SCS             : %s", scs_str);
    }

    /* --- NR5G Voice Domain --- */
    if (nas_sys_ind.nr5g_voice_domain_valid)
    {
     LOGI("[NR5G] Voice Domain    : %d (0=NoVoice,1=IMS)",
          nas_sys_ind.nr5g_voice_domain);
    }

    /* --- NR-DC Info --- */
    if (nas_sys_ind.nrdc_pci_valid)
    {
     LOGI("[NR-DC] PCI            : %u", nas_sys_ind.nrdc_pci);
    }
    if (nas_sys_ind.nrdc_arfcn_valid)
    {
     LOGI("[NR-DC] ARFCN          : %u", nas_sys_ind.nrdc_arfcn);
    }
    if (nas_sys_ind.nrdc_freq_type_valid)
    {
     LOGI("[NR-DC] Freq Type      : %d (%s)",
          nas_sys_ind.nrdc_freq_type,
          nas_sys_ind.nrdc_freq_type == 0 ? "Sub6" : "mmWave");
    }

    LOGI("==============================");
    break;

   case QMI_NAS_OPERATOR_NAME_DATA_IND_MSG_V01:
   {
    nas_operator_name_data_ind_msg_v01 op_ind;

    LOGI("QMI_NAS_OPERATOR_NAME_DATA_IND_MSG_V01");
    memset(&op_ind, 0, sizeof(op_ind));
    qmi_error = qmi_client_message_decode(user_handle,
                                           QMI_IDL_INDICATION,
                                           msg_id,
                                           (void*)ind_buf,
                                           ind_buf_len,
                                           (void*)&op_ind,
                                           sizeof(nas_operator_name_data_ind_msg_v01));
    if (QMI_NO_ERR != qmi_error)
    {
     LOGE("Failed to decode OPERATOR_NAME_DATA_IND: err=%d", qmi_error);
     return;
    }

    LOGI("=== Operator Name Data Indication ===");

    /* Service Provider Name */
    if (op_ind.service_provider_name_valid)
    {
     LOGI("  SPN Display Cond   : 0x%02X", op_ind.service_provider_name.display_cond);
     if (op_ind.service_provider_name.spn_len > 0)
     {
      char spn_buf[NAS_SERVICE_PROVIDER_NAME_MAX_V01 + 1];
      uint32_t len = op_ind.service_provider_name.spn_len;
      if (len > NAS_SERVICE_PROVIDER_NAME_MAX_V01)
       len = NAS_SERVICE_PROVIDER_NAME_MAX_V01;
      memcpy(spn_buf, op_ind.service_provider_name.spn, len);
      spn_buf[len] = '\0';
      LOGI("  SPN                : %s", spn_buf);
     }
    }

    /* PLMN Name (CPHS Operator Name String) */
    if (op_ind.plmn_name_valid)
    {
     LOGI("  PLMN Name          : %s", op_ind.plmn_name);
    }

    /* NITZ Information */
    if (op_ind.nitz_information_valid)
    {
     LOGI("  NITZ Coding Scheme : %d (%s)",
          op_ind.nitz_information.coding_scheme,
          op_ind.nitz_information.coding_scheme == NAS_CODING_SCHEME_UCS2_V01 ? "UCS2" : "GSM");
     if (op_ind.nitz_information.long_name_len > 0)
     {
      char nitz_long[NAS_LONG_NAME_MAX_V01 + 1];
      nas_convert_nw_name(nitz_long, sizeof(nitz_long),
                          op_ind.nitz_information.long_name,
                          op_ind.nitz_information.long_name_len,
                          op_ind.nitz_information.coding_scheme);
      LOGI("  NITZ Long Name     : %s", nitz_long);
     }
     if (op_ind.nitz_information.short_name_len > 0)
     {
      char nitz_short[NAS_SHORT_NAME_MAX_V01 + 1];
      nas_convert_nw_name(nitz_short, sizeof(nitz_short),
                          op_ind.nitz_information.short_name,
                          op_ind.nitz_information.short_name_len,
                          op_ind.nitz_information.coding_scheme);
      LOGI("  NITZ Short Name    : %s", nitz_short);
     }
    }

    /* PLMN Network Name list */
    if (op_ind.plmn_network_name_valid && op_ind.plmn_network_name_len > 0)
    {
     uint32_t idx;
     LOGI("  PLMN Network Names : %u entries", op_ind.plmn_network_name_len);
     for (idx = 0; idx < op_ind.plmn_network_name_len && idx < 3; idx++)
     {
      LOGI("    [%u] Coding      : %d (%s)", idx,
           op_ind.plmn_network_name[idx].coding_scheme,
           op_ind.plmn_network_name[idx].coding_scheme == NAS_CODING_SCHEME_UCS2_V01 ? "UCS2" : "GSM");
      if (op_ind.plmn_network_name[idx].long_name_len > 0)
      {
       char pnn_long[NAS_LONG_NAME_MAX_V01 + 1];
       nas_convert_nw_name(pnn_long, sizeof(pnn_long),
                           op_ind.plmn_network_name[idx].long_name,
                           op_ind.plmn_network_name[idx].long_name_len,
                           op_ind.plmn_network_name[idx].coding_scheme);
       LOGI("    [%u] Long Name   : %s", idx, pnn_long);
      }
      if (op_ind.plmn_network_name[idx].short_name_len > 0)
      {
       char pnn_short[NAS_SHORT_NAME_MAX_V01 + 1];
       nas_convert_nw_name(pnn_short, sizeof(pnn_short),
                           op_ind.plmn_network_name[idx].short_name,
                           op_ind.plmn_network_name[idx].short_name_len,
                           op_ind.plmn_network_name[idx].coding_scheme);
       LOGI("    [%u] Short Name  : %s", idx, pnn_short);
      }
     }
    }

    LOGI("=====================================");
    break;
   }

   case QMI_NAS_SIG_INFO_IND_MSG_V01:
    LOGI("QMI_NAS_SIG_INFO_IND_MSG_V01");
    memset(&nas_sig_ind, 0, sizeof(nas_sig_ind));
    qmi_error = qmi_client_message_decode(user_handle,
                                           QMI_IDL_INDICATION,
                                           msg_id,
                                           (void*)ind_buf,
                                           ind_buf_len,
                                           (void*)&nas_sig_ind,
                                           sizeof(nas_sig_info_ind_msg_v01));
    if (QMI_NO_ERR != qmi_error)
    {
     LOGE("Failed to decode SIG_INFO_IND: err=%d", qmi_error);
     return;
    }

    if (nas_sig_ind.lte_sig_info_valid)
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

static void qmi_release_func( void )
{
  int rc = -1;

  if ( user_handle_nas != NULL )
  {
    rc = qmi_client_release(user_handle_nas);
    if (rc < 0 )
    {
      LOGE("Qmi nas client release not successful");
    }
    else
    {
      LOGI("Qmi nas client release successful");
    }
  }

  return;
}

int qmi_nas_client_test_init (void)
{
  qmi_client_error_type qmi_error = QMI_NO_ERR;

  /* Local variables Declaration */
  qmi_idl_service_object_type   nas_service_object;
  qmi_client_os_params          nas_os_params;

  /* Intilize all the variables */
  memset(&nas_service_object, 0, sizeof(qmi_idl_service_object_type));
  memset(&nas_os_params, 0, sizeof(qmi_client_os_params));

  /* Get the NAS service object */
  nas_service_object = nas_get_service_object_v01();

  /* Initialize a connection to first QMI control port */
  if ( nas_service_object == NULL )
  {
    LOGE("service object not available");
    return -1;
  }
  LOGI("service object available");

  /* Initlize client */
  qmi_error = qmi_client_init_instance(nas_service_object, 
                                       QMI_CLIENT_INSTANCE_ANY, 
                                       qmi_nas_client_test_ind_cb, 
                                       NULL,
                                       &nas_os_params, 
                                       SEND_TIMEOUT, 
                                       &user_handle_nas);

  if( qmi_error != QMI_NO_ERR )
  {
    LOGE("qmi_client_init_instance - qmi_error : %d", qmi_error);
    return -1;
  }

  register_for_nas_indication(user_handle_nas);
  return 0;
}

void* mps_qmi_test_start_func( void* arg )
{
  int rc = 0;

  qmi_gms_test();
  
  rc = qmi_nas_client_test_init();
  if ( rc == -1)
  {
    return (void*)1;
  }

  return (void*)0;
}

int main(void)
{
  int rc = -1;
  int qmi_init_thread = -1;
  int thread_created = 0;
  pthread_t qmi_init_thread_handler = 0;

  /* run thread to qmi messages */
  while (1)
  {
    if (!thread_created)
    {
      qmi_init_thread = pthread_create(&qmi_init_thread_handler,
                                       NULL, 
                                       mps_qmi_test_start_func, 
                                       NULL);

      if( qmi_init_thread == 0 )
      {
        thread_created = 1;

        void* thread_ret = NULL;
        rc = pthread_join( qmi_init_thread_handler, &thread_ret );
        if (rc != 0)
        {
          LOGE( "pthread_join failed" );
          exit( -1 );
        }
        
        if (thread_ret != 0)
        {
          LOGE("mps_qmi_test_start_func reported error (ret=%p). Exiting loop.", thread_ret);
          break;
        }
        else
        {
          LOGI("pthread_join success (no error)");
        }
      }
      else
      {
        LOGE( "pthread_create failed" );
        break;
      }
    }

    sleep(1);
  }

  /* Release the NAS client in case the NAS thread was created*/
  qmi_release_func();

  LOGI( "QMI Release done" );
  return 0;
}
