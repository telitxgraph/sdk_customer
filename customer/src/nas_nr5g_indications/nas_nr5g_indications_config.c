/******************************************************************************

  @file    nas_nr5g_indications_config.c
  @brief   Default configuration for TNS sync pulse parameters

  ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nas_nr5g_indications.h"

/*===========================================================================
                       CONFIG FUNCTIONS
===========================================================================*/

/**
  * @brief Set default configuration values.
  */
void tns_config_set_defaults(tns_sync_pulse_config_t *config)
{
	if (config == NULL)
		return;

	memset(config, 0, sizeof(tns_sync_pulse_config_t));

	config->pulse_period         = 10;   /* 10 * 10ms = 100ms (10Hz) */
	config->start_sfn            = 1024; /* next available SFN */
	config->report_period        = 10;   /* 10 * 10ms = 100ms */
	config->pulse_align_type     = 0;    /* NR5G frame boundary */
	config->pulse_trigger_action = 0;    /* Trigger pulse */
	config->pulse_get_cxo_count  = 0;    /* Do not get CXO count */
}
