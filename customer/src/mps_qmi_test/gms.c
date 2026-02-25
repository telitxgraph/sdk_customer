#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/stat.h>

#include "qmi_idl_lib.h"
#include "qmi_csi.h"
#include "qmi_sap.h"
//nclude "qmi/network_access_service_v01.h"
#include "qmi_client.h"
#include "general_modem_service_v01.h"

#include "gms.h"

void qmi_gms_test(void)
{
  printf("QMI gms Test\n");
  int fail = 1;
  qmi_client_error_type qerr;
  qmi_client_os_params os_params;
  int msg_id, req_len, resp_len;
  int timeout = 1000;

  qmi_client_type gms_chndl = NULL;
  qmi_idl_service_object_type gms_srvc_obj = gms_get_service_object_v01();
  /* Initialize a connection to first QMI control port */
  if ( gms_srvc_obj == NULL )
  {
    printf("service object not available\n");
//  return -1;
  }
  
  qerr = qmi_client_init_instance(gms_srvc_obj, QMI_CLIENT_INSTANCE_ANY, NULL, NULL, &os_params, 4, &gms_chndl);
  if( qerr )
  {
    printf("ERROR: error while trying to initialize client\n");
    goto done;
  }

  printf( "client gms handle: %p\n", gms_chndl );

  gms_nas_get_ca_info_req_msg_v01 ca_info_req;
  gms_nas_get_ca_info_resp_msg_v01 ca_info_resp;

  msg_id = QMI_GMS_NAS_GET_CA_INFO_REQ_V01;
  req_len = sizeof( gms_nas_get_ca_info_req_msg_v01 );
  resp_len = sizeof( gms_nas_get_ca_info_resp_msg_v01 );
  memset( ( void * )&ca_info_req, 0, req_len );
  memset( (void * )&ca_info_resp, 0, resp_len );

  printf( "send_msg_sync msgid=0x%04x,req=%p,req_len=%d,resp=%p,resp_len=%d,timeout=%d\n", \
            msg_id, &ca_info_req, req_len, &ca_info_resp, resp_len, timeout);

  qerr = qmi_client_send_msg_sync( gms_chndl, msg_id, &ca_info_req, req_len, &ca_info_resp, resp_len, timeout );

  if( qerr )
  {
    printf( "ERROR: qmi_client_send_msg_sync, qerr=%d\n", qerr );
    goto done;
  }

  printf( "qmi_client_send_msg_sync RETURNs success\n" );
  printf( "result: %d\n", ca_info_resp.resp.result );
  if( QMI_RESULT_FAILURE_V01 == ca_info_resp.resp.result )
  {
    printf ("get cell loc command failed: error: 0x%x\n", ca_info_resp.resp.error );
    goto done;
  }
  else
  {
    printf( "[GMS] CA_INFO lte_ca_pcc_info_valid       : %d\n", ca_info_resp.lte_ca_pcc_info_valid );
    printf( "[GMS] CA_INFO lte_ca_pcc_info.band_class  : %d\n", ca_info_resp.lte_ca_pcc_info.band_class );
    printf( "[GMS] CA_INFO lte_ca_pcc_info.channel     : %d\n", ca_info_resp.lte_ca_pcc_info.channel );
    printf( "[GMS] CA_INFO lte_ca_pcc_info.dl_bw       : %d\n", ca_info_resp.lte_ca_pcc_info.dl_bw );
    printf( "[GMS] CA_INFO lte_ca_pcc_info.pci         : %d\n", ca_info_resp.lte_ca_pcc_info.pci );
    printf( "[GMS] CA_INFO lte_ca_pcc_info.pci size    : %lu\n", sizeof( ca_info_resp.lte_ca_pcc_info.pci ) );
    printf( "[GMS] CA_INFO lte_ca_pcc_info.rsrp        : %d\n", ca_info_resp.lte_ca_pcc_info.rsrp );
    printf( "[GMS] CA_INFO lte_ca_pcc_info.rsrq        : %d\n", ca_info_resp.lte_ca_pcc_info.rsrq );
    printf( "[GMS] CA_INFO lte_ca_pcc_info.sinr        : %d\n", ca_info_resp.lte_ca_pcc_info.sinr );
    printf( "[GMS] CA_INFO lte_ca_pcc_info.tac         : %d\n", ca_info_resp.lte_ca_pcc_info.tac );
  }

  printf( "****************GMS PASSED *******************\n" );
  fail = 0;

done:
  if( gms_chndl )
    qmi_client_release( gms_chndl );
  
  gms_chndl = NULL;

  if( fail )
  {
    printf("****************GMS FAILED *******************\n");
  }
  printf("QMI GMS Test Finished\n");
}


