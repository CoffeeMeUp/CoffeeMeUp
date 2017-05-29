/*
 * com.c
 *
 * Created: 19/04/2017 18:17:25
 *  Author: user
 */

#include "com.h"
#include "mensagens.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


com_t *com_interpretando_buffer(char *pdata){
	com_t *ret;
	char  *hour_data;
	ret = (com_t *) malloc(sizeof(com_t));
	if (!strcmp(pdata,PACOTE_LED_ON)){
		printf("%s \n", PACOTE_LED_ON);
		ret->pkg_type  = command_LED_ON;
		ret->pkg_value = NULL;
		return ret;
	} else 	if (!strcmp(pdata, PACOTE_LED_OFF)){
		printf("%s \n", PACOTE_LED_OFF);
		ret->pkg_type  = command_LED_OFF;
		ret->pkg_value = NULL;
		return ret;
	} else if (!strcmp(pdata,PACOTE_TESTE_RX)){
		ret->pkg_type  = PACOTE_TESTE_COM;
		ret->pkg_value = PACOTE_TESTE_tx_OK;
		return ret;
	} else if ((hour_data = strstr(pdata, PACOTE_HEADER_RX)) != NULL) {
		hour_t *h = parse_hour(hour_data + 5);
		ret->pkg_value = h;
		ret->pkg_type  = PACOTE_ALARM_SET;
		return ret;
	} else if (!strcmp(pdata,pacote_Cafeteria_ON)){
		printf("%s \n", pacote_Cafeteria_ON);
		ret->pkg_value = NULL;
		ret->pkg_type  = command_Cafeteria_ON;
		return ret;
	} else if (!strcmp(pdata,pacote_Cafeteria_OFF)){
		printf("%s \n", pacote_Cafeteria_OFF);
		ret->pkg_value = NULL;
		ret->pkg_type  = command_Cafeteria_OFF;
		return ret;
	} else if (!strcmp(pdata, pacote_Buzzer_ON)){
		printf("%s \n", pacote_Buzzer_ON);
		ret->pkg_type  = command_Buzzer_ON;
		ret->pkg_value = NULL;
		return(command_Buzzer_ON);
	} else if (!strcmp(pdata, pacote_Buzzer_OFF)){
		printf("%s \n", pacote_Buzzer_OFF);
		ret->pkg_value = NULL;
		ret->pkg_type  = command_Buzzer_OFF;
		return ret;
	}
	ret->pkg_type  = PACOTE_ERRO;
	ret->pkg_value = NULL;
	return ret;
}