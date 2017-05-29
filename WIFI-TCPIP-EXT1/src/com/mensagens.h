/*
 * mensagens.h
 *
 * Created: 19/04/2017 18:24:59
 *  Author: user
 */


#ifndef MENSAGENS_H_
#define MENSAGENS_H_

#define PACOTE_TESTE_RX     "Teste Com"
#define PACOTE_HEADER_RX    "hora:"
#define PACOTE_TESTE_tx_OK  "OK \n"
#define PACOTE_TESTE_tx_NOK "[ERRO] 1 \n"
#define PACOTE_LED_ON       "LED ON"
#define PACOTE_LED_OFF      "LED OFF"

#define command_LED_ON		0x01
#define command_LED_OFF		0x02

#define PACOTE_TESTE_COM	0x01
#define PACOTE_ALARM_SET    0x10
#define command_REL_ALARME	0x11
#define PACOTE_ERRO			0xFF

#endif /* MENSAGENS_H_ */