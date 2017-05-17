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

#include "../helpers/hour_helper.h"

typedef struct {
	// usado no switch case da main.c, pode conter pacoteTesteCom,
	// pacoteERRO etc
	char  pkg_type;
	void *pkg_value;
} com_t;

/* Funcoes */
com_t *com_interpretando_buffer(char *pdata);

#endif /* COM_H_ */