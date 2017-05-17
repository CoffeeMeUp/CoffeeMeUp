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
	char *hour_data;
	com_t *ret;
	ret = (com_t *) malloc(sizeof(com_t));
	if (!strcmp(pdata,PACOTE_TESTE_RX)){
		ret->pkg_type  = PACOTE_TESTE_COM;
		ret->pkg_value = PACOTE_TESTE_tx_OK;
		return ret;
	} else if ((hour_data = strstr(pdata, PACOTE_HEADER_RX)) != NULL) {
		hour_t *h = parse_hour(hour_data + 5);
		ret->pkg_value = h;
		ret->pkg_type  = PACOTE_ALARM_SET;
		return ret;
	}
	ret->pkg_type  = PACOTE_ERRO;
	ret->pkg_value = NULL;
	return ret;
}