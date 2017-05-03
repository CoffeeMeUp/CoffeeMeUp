/*
 * com.c
 *
 * Created: 19/04/2017 18:17:25
 *  Author: user
 */

#include "com.h"
#include "mensagens.h"
#include <stdio.h>
#include <string.h>

char com_interpretando_buffer(char *pdata){
	char *hour_data;
	if (!strcmp(pdata,pacote_LED_ON)){
		printf("%s \n", pacote_LED_ON);
		return(command_LED_ON);
	} else 	if (!strcmp(pdata,pacote_LED_OFF)){
		printf("%s \n", pacote_LED_OFF);
		return(command_LED_OFF);
	}
	else if ((hour_data = strstr(pdata, PACOTE_HEADER_RX)) != NULL) {
		//puts("oi");
		printf("%s\n", hour_data + 5 * sizeof(char));
		return(command_REL_ALARME);
	}
	else{
		printf("%s", pacote_TESTE_tx_nok);
		return(command_ERRO);
	}
}