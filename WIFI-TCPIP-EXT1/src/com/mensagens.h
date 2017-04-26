/*
 * mensagens.h
 *
 * Created: 19/04/2017 18:24:59
 *  Author: user
 */ 


#ifndef MENSAGENS_H_
#define MENSAGENS_H_

const char pacote_TESTE_rx[]		= "Teste Com" ;
const char pacote_TESTE_tx_ok[]		= "OK \n";
const char pacote_TESTE_tx_nok[]	= "[ERRO] 1 \n";

#define pacoteTesteCom		0x01
#define pacoteERRO			0xFF

#endif /* MENSAGENS_H_ */