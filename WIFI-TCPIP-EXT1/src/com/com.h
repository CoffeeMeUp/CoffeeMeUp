/*
 * com.h
 *
 * Created: 19/04/2017 18:13:07
 *  Author: user
 */ 


#ifndef COM_H_
#define COM_H_

/**
 * @brief STDINT possui as definições dos tipos de variáveis
 * e constantes
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Funcoes */
char com_interpretando_buffer(uint8_t *pdata);

#endif /* COM_H_ */