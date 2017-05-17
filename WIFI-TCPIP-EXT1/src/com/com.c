/*
 * com.c
 *
 * Created: 19/04/2017 18:17:25
 *  Author: user
 */

#include "com.h"
#include "mensagens.h"
#include "../helpers/hour_helper.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

char com_interpretando_buffer(char *pdata){

	char *hour_data;
	if (!strcmp(pdata,pacote_TESTE_rx)){
		printf("%s", pacote_TESTE_tx_ok);
		return(pacoteTesteCom);
	} else if ((hour_data = strstr(pdata, PACOTE_HEADER_RX)) != NULL) {
		// hour header
		//printf("%s\n", hour_data + 5 * sizeof(char));
		hour_t *h = parse_hour(hour_data + 5);
		printf("Hora: %hhu\nMinuto: %hhu\n", h->hour, h->minute);
		free(h);
		return(pacoteTesteCom);
	} else{
		printf("%s", pacote_TESTE_tx_nok);
		return(pacoteERRO);
	}
}