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

#define min(a,b)  ((a) < (b) ? (a):(b))
#define SEND_TIMEOUT 	50000

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
qmi_client_type 			client_handle = NULL;
qmi_idl_service_object_type nas_service_object;

qmi_client_os_params 		*os_params;
qmi_client_type 			*user_handle;
void 						*ind_cb_data;
uint32_t 					timeout;

char* log_file = "syslog_mps_qmi_test";

static qmi_client_os_params    nas_os_params_main;
static qmi_client_type         nas_notifier_main;

/*===========================================================================
                              FUNCTION DEFINITIONS
===========================================================================*/


void nas_qmi_test_client_error_cb
(
  qmi_client_type user_handle,
  qmi_client_error_type error,
  void *err_cb_data
);

void  qmi_nas_client_test_ind_cb
(
  qmi_client_type                user_handle,
  unsigned int                   msg_id,
  void                          *ind_buf_ptr,
  unsigned int                   ind_buf_len,
  void                          *ind_cb_data
);

int qmi_nas_client_test_init( void );
void* mps_qmi_test_start_func( void* arg );


/*---------------------------------------------------------------------------
   QMI Notifier handler
---------------------------------------------------------------------------*/
static qmi_client_type notifier_handle = NULL;

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

void qmi_nas_client_test_ind_cb
(
  qmi_client_type				user_handle,
  unsigned int	   			msg_id,
  void			        			*ind_buf,
  unsigned int	   			ind_buf_len,
  void				        		*ind_cb_data
)
{
  qmi_client_error_type			       	qmi_error;
  nas_serving_system_ind_msg_v01		serving_system_ind;
  nas_event_report_ind_msg_v01	  	event_report_ind;
  nas_sys_info_ind_msg_v01			     nas_sys_ind;
  nas_sig_info_ind_msg_v01			     nas_sig_ind;
  (void) ind_cb_data;

//  LOGI("ind msg_id: %04X ", msg_id);

  memset(&nas_sys_ind, 0, sizeof(nas_sys_info_ind_msg_v01));
  memset(&nas_sig_ind, 0, sizeof(nas_sig_info_ind_msg_v01));

  switch(msg_id)
  {
    // 0x0002
    case QMI_NAS_EVENT_REPORT_IND_MSG_V01:
      memset(&event_report_ind, 0, sizeof(event_report_ind));

      /* This is the API used to decode the the unsolicited events from the modem */
      qmi_error = qmi_client_message_decode(user_handle,
                                      						QMI_IDL_INDICATION,
                                      						msg_id,
                                      						(void*)ind_buf,
                                      						ind_buf_len,
                                      						(void *)&event_report_ind,
                                      						sizeof(event_report_ind));

      if(QMI_NO_ERR != qmi_error)
      {
        LOGI("Decode of NAS Indication message returned error: %d", qmi_error);
        return;
      }
      LOGI("NAS Indication message decode OK ");		

      if (event_report_ind.signal_strength_valid == 1)
      {
        LOGI("radio_if : %04X", event_report_ind.signal_strength.radio_if);
        switch (event_report_ind.signal_strength.radio_if)
        {
          case 0x00:
            LOGI("RADIO_IF_NO_SVC");
            break;
          case 0x04:
            LOGI("RADIO_IF_GSM");
            break;
          case 0x05:
            LOGI("RADIO_IF_UMTS");
            break;
          case 0x08:
            LOGI("RADIO_IF_LTE");
            break;
          case 0x0C:
            LOGI("RADIO_IF_NR5G");
            break;
          default:
            LOGI("Invalid RF Mode!!!");
            break;
        }
      }
      else
      {
        LOGE("Expected RSSI info to be valid but didn't find it so!");
      }
      break;

    // 0x0024			
    case QMI_NAS_SERVING_SYSTEM_IND_MSG_V01:
      LOGI("QMI_NAS_SERVING_SYSTEM_IND_MSG_V01");
      qmi_error = qmi_client_message_decode(user_handle,
                                          QMI_IDL_INDICATION,
                                          msg_id,
                                          (void*)ind_buf,
                                          ind_buf_len,
                                          (void*)&serving_system_ind,
                                          sizeof(serving_system_ind));
  
      if( QMI_NO_ERR != qmi_error )
      {
        LOGE("Invalid filter mode ind msg error %d", qmi_error );
      }
      LOGI("registration STATUS:%d", serving_system_ind.serving_system.registration_state);

      if (serving_system_ind.serving_system.registration_state == 0)
      {
        LOGE("NOT_REGISTERED");
      }
      break;

    // 0x003A
    case QMI_NAS_OPERATOR_NAME_DATA_IND_MSG_V01:
      LOGI("QMI_NAS_OPERATOR_NAME_DATA_IND_MSG_V01");
      break;

    // 0x004E
    case QMI_NAS_SYS_INFO_IND_MSG_V01:
      LOGI("QMI_NAS_SYS_INFO_IND_MSG_V01");
      break;

    // 0x0051
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
        LOGI("[LTE] SNR  : %hd\n", nas_sig_ind.lte_sig_info.snr);
      }
      break;

    default:
      LOGI("NOT PROCESSED MSG_ID: %04X", msg_id );
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
