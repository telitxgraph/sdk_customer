/******************************************************************************

  @file    mps_tns_config.c
  @brief   Configuration file reader for TNS sync pulse parameters

  ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>

#include "mps_tns.h"

/*===========================================================================
                       HELPER FUNCTIONS
===========================================================================*/

/**
  * @brief Trim leading and trailing whitespace from a string in-place.
  */
static char *trim_whitespace(char *str)
{
	char *end;

	/* Trim leading space */
	while (isspace((unsigned char)*str))
		str++;

	if (*str == '\0')
		return str;

	/* Trim trailing space */
	end = str + strlen(str) - 1;
	while (end > str && isspace((unsigned char)*end))
		end--;

	end[1] = '\0';
	return str;
}

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

/**
  * @brief Load sync pulse configuration from a file.
  *
  * Config file format (key=value, one per line, # for comments):
  *   pulse_period=10
  *   start_sfn=1024
  *   report_period=10
  *   pulse_align_type=0
  *   pulse_trigger_action=0
  *   pulse_get_cxo_count=0
  *
  * @param path   Path to the config file.
  * @param config Output config structure.
  * @return 0 on success, -1 on failure (defaults are still set).
  */
int tns_config_load(const char *path, tns_sync_pulse_config_t *config)
{
	FILE *fp = NULL;
	char line[256];
	char *key, *value, *sep;

	if (config == NULL)
		return -1;

	/* Set defaults first */
	tns_config_set_defaults(config);

	if (path == NULL) {
		LOGE("Config path is NULL, using defaults\n");
		return -1;
	}

	fp = fopen(path, "r");
	if (fp == NULL) {
		LOGE("Cannot open config file '%s': %s, using defaults\n", path, strerror(errno));
		return -1;
	}

	LOGI("Loading config from '%s'\n", path);

	while (fgets(line, sizeof(line), fp) != NULL) {
		/* Skip comments and empty lines */
		key = trim_whitespace(line);
		if (*key == '\0' || *key == '#')
			continue;

		/* Find '=' separator */
		sep = strchr(key, '=');
		if (sep == NULL)
			continue;

		*sep = '\0';
		value = trim_whitespace(sep + 1);
		key = trim_whitespace(key);

		if (strcmp(key, "pulse_period") == 0) {
			config->pulse_period = (uint32_t)strtoul(value, NULL, 0);
			if (config->pulse_period > 128)
				config->pulse_period = 128;
		} else if (strcmp(key, "start_sfn") == 0) {
			config->start_sfn = (uint32_t)strtoul(value, NULL, 0);
			if (config->start_sfn > 1024)
				config->start_sfn = 1024;
		} else if (strcmp(key, "report_period") == 0) {
			config->report_period = (uint32_t)strtoul(value, NULL, 0);
			if (config->report_period > 128)
				config->report_period = 128;
		} else if (strcmp(key, "pulse_align_type") == 0) {
			config->pulse_align_type = (uint8_t)strtoul(value, NULL, 0);
			if (config->pulse_align_type > 1)
				config->pulse_align_type = 0;
		} else if (strcmp(key, "pulse_trigger_action") == 0) {
			config->pulse_trigger_action = (uint8_t)strtoul(value, NULL, 0);
			if (config->pulse_trigger_action > 1)
				config->pulse_trigger_action = 0;
		} else if (strcmp(key, "pulse_get_cxo_count") == 0) {
			config->pulse_get_cxo_count = (uint8_t)strtoul(value, NULL, 0);
			if (config->pulse_get_cxo_count > 1)
				config->pulse_get_cxo_count = 0;
		} else {
			LOGI("Unknown config key: '%s'\n", key);
		}
	}

	fclose(fp);

	LOGI("Config loaded: pulse_period=%u, start_sfn=%u, report_period=%u, "
	     "align_type=%u, trigger_action=%u, get_cxo=%u\n",
	     config->pulse_period, config->start_sfn, config->report_period,
	     config->pulse_align_type, config->pulse_trigger_action,
	     config->pulse_get_cxo_count);

	return 0;
}
