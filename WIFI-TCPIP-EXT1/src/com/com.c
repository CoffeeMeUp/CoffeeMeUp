/*
 * com.c
 *
 * Created: 19/04/2017 18:17:25
 *  Author: user
 */ 

#include "com.h"
#include "mensagens.h"

char com_interpretando_buffer(uint8_t *pdata){
	
	if (!strcmp(pdata,pacote_TESTE_rx)){
		printf("%s", pacote_TESTE_tx_ok);
		return(pacoteTesteCom);
	}
	else{
		printf("%s", pacote_TESTE_tx_nok);
		return(pacoteERRO);
	}
}