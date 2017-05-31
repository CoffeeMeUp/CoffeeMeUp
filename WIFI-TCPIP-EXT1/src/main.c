

/** \mainpage
 * \section intro Introduction
 * This example demonstrates the use of the WINC1500 with the SAM Xplained Pro
 * board to test TCP server.<br>
 * It uses the following hardware:
 * - the SAM Xplained Pro.
 * - the WINC1500 on EXT1.
 *
 * \section files Main Files
 * - main.c : Initialize the WINC1500 and test TCP server.
 *
 * \section usage Usage
 * -# Configure below code in the main.h for AP information to be connected.
 * \code
 *    #define MAIN_WLAN_SSID                    "DEMO_AP"
 *    #define MAIN_WLAN_AUTH                    M2M_WIFI_SEC_WPA_PSK
 *    #define MAIN_WLAN_PSK                     "12345678"
 *    #define MAIN_WIFI_M2M_PRODUCT_NAME        "NMCTemp"
 *    #define MAIN_WIFI_M2M_SERVER_IP           0xFFFFFFFF // "255.255.255.255"
 *    #define MAIN_WIFI_M2M_SERVER_PORT         (6666)
 *    #define MAIN_WIFI_M2M_REPORT_INTERVAL     (1000)
 * \endcode
 * -# Build the program and download it into the board.
 * -# On the computer, open and configure a terminal application as the follows.
 * \code
 *    Baud Rate : 115200
 *    Data : 8bit
 *    Parity bit : none
 *    Stop bit : 1bit
 *    Flow control : none
 * \endcode
 * -# Start the application.
 * -# In the terminal window, the following text should appear:
 * \code
 *    -- WINC1500 TCP server example --
 *    -- SAMD21_XPLAINED_PRO --
 *    -- Compiled: xxx xx xxxx xx:xx:xx --
 *    wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: CONNECTED
 *    wifi_cb: M2M_WIFI_REQ_DHCP_CONF: IP is xxx.xxx.xxx.xxx
 *    socket_cb: bind success!
 *    socket_cb: listen success!
 *    socket_cb: accept success!
 *    socket_cb: recv success!
 *    socket_cb: send success!
 *    TCP Server Test Complete!
 *    close socket
 * \endcode
 *
 * \section compinfo Compilation Information
 * This software was written for the GNU GCC compiler using Atmel Studio 6.2
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com">Atmel</A>.\n
 */

#include "asf.h"
#include <string.h>
#include <stdlib.h>
#include "main.h"
#include "common/include/nm_common.h"
#include "driver/include/m2m_wifi.h"
#include "socket/include/socket.h"
#include "com/com.h"
#include "com/mensagens.h"
#include "helpers/rtc_helper.h"

/** PWM frequency in Hz */
#define PWM_FREQUENCY      100
/** Period value of PWM output waveform */
#define PERIOD_VALUE       100
/** Initial duty cycle value */
#define INIT_DUTY_VALUE    0

#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- WINC1500 TCP server example --"STRING_EOL \
	"-- "BOARD_NAME " --"STRING_EOL	\
	"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

#define LED_PIO_ID			12
#define LED_PIO			 PIOC
#define LED_PIN			 8
#define LED_PIN_MASK (1<<LED_PIN)

#define CAFETERIA_PIO_ID	ID_PIOB
#define CAFETERIA_PIO		PIOB
#define CAFETERIA_PIN		1
#define CAFETERIA_PIN_MASK (1<<CAFETERIA_PIN)

#define BUZZER_PIO_ID	ID_PIOD
#define BUZZER_PIO		PIOD
#define BUZZER_PIN		28
#define BUZZER_PIN_MASK (1<<BUZZER_PIN)

/** PWM channel instance for LEDs */
pwm_channel_t g_pwm_channel_led;

/** Message format definitions. */
typedef struct s_msg_wifi_product {
	uint8_t name[9];
} t_msg_wifi_product;

/** Message format declarations. */
static t_msg_wifi_product msg_wifi_product = {
	.name = MAIN_WIFI_M2M_PRODUCT_NAME,
};

/** Receive buffer definition. */
static uint8_t gau8SocketTestBuffer[MAIN_WIFI_M2M_BUFFER_SIZE];

/** Socket for TCP communication */
static SOCKET tcp_server_socket = -1;
static SOCKET tcp_client_socket = -1;

/** Wi-Fi connection state */
static uint8_t wifi_connected;


/** contador de mensagens */

uint32_t g_nMensagensRx = 0 ;


void RTC_Handler(void)
{
	uint32_t status = rtc_get_status(RTC);

	// Second increment interrupt (why
	// is this even a thing...)
	if ((status & RTC_SR_SEC) == RTC_SR_SEC) {
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
		// time or date alarm (AKA what we want)
	}
	if ((status & RTC_SR_ALARM) == RTC_SR_ALARM) {
		rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
		puts("ALERTA CAFE!");
		CAFETERIA_PIO->PIO_CODR = CAFETERIA_PIN_MASK;
	}

}

/**
 * \brief Configure UART console.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate =		CONF_UART_BAUDRATE,
		.charlength =	CONF_UART_CHAR_LENGTH,
		.paritytype =	CONF_UART_PARITY,
		.stopbits =		CONF_UART_STOP_BITS,
	};

	/* Configure UART console. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

/**
 * \brief Callback to get the Data from socket.
 *
 * \param[in] sock socket handler.
 * \param[in] u8Msg socket event type. Possible values are:
 *  - SOCKET_MSG_BIND
 *  - SOCKET_MSG_LISTEN
 *  - SOCKET_MSG_ACCEPT
 *  - SOCKET_MSG_CONNECT
 *  - SOCKET_MSG_RECV
 *  - SOCKET_MSG_SEND
 *  - SOCKET_MSG_SENDTO
 *  - SOCKET_MSG_RECVFROM
 * \param[in] pvMsg is a pointer to message structure. Existing types are:
 *  - tstrSocketBindMsg
 *  - tstrSocketListenMsg
 *  - tstrSocketAcceptMsg
 *  - tstrSocketConnectMsg
 *  - tstrSocketRecvMsg
 */
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	switch (u8Msg) {
	/* Socket bind */
	case SOCKET_MSG_BIND:
	{
		tstrSocketBindMsg *pstrBind = (tstrSocketBindMsg *)pvMsg;
		if (pstrBind && pstrBind->status == 0) {
			printf("socket_cb: bind success!\r\n");
			listen(tcp_server_socket, 0);
		} else {
			printf("socket_cb: bind error!\r\n");
			close(tcp_server_socket);
			tcp_server_socket = -1;
		}
	}
	break;

	/* Socket listen */
	case SOCKET_MSG_LISTEN:
	{
		tstrSocketListenMsg *pstrListen = (tstrSocketListenMsg *)pvMsg;
		if (pstrListen && pstrListen->status == 0) {
			printf("socket_cb: listen success!\r\n");
			accept(tcp_server_socket, NULL, NULL);
		} else {
			printf("socket_cb: listen error!\r\n");
			close(tcp_server_socket);
			tcp_server_socket = -1;
		}
	}
	break;

	/* Connect accept */
	case SOCKET_MSG_ACCEPT:
	{
		tstrSocketAcceptMsg *pstrAccept = (tstrSocketAcceptMsg *)pvMsg;
		if (pstrAccept) {
			printf("socket_cb: accept success!\r\n");
			accept(tcp_server_socket, NULL, NULL);
			tcp_client_socket = pstrAccept->sock;
			recv(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 0);
		} else {
			printf("socket_cb: accept error!\r\n");
			close(tcp_server_socket);
			tcp_server_socket = -1;
		}
	}
	break;

	/* Message send */
	case SOCKET_MSG_SEND:
	{
		printf("socket_cb: send success!\r\n");
		printf("TCP Server Test Complete!\r\n");
		printf("close socket\n");
		//close(tcp_client_socket);
		//close(tcp_server_socket);
	}
	break;

	/* Message receive */
	case SOCKET_MSG_RECV:
	{
		tstrSocketRecvMsg *pstrRecv = (tstrSocketRecvMsg *)pvMsg;
		if (pstrRecv && pstrRecv->s16BufferSize > 0) {

			//printf("socket_cb: recv success!\r\n");
			//printf(" -------- \n", gau8SocketTestBuffer);
			g_nMensagensRx++;
			printf("Mensagem recebida do PUTTY: numero %d \n", g_nMensagensRx);

			printf("%s \n", gau8SocketTestBuffer);
			printf(" -------- \n", gau8SocketTestBuffer);

			send(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 0);

			/************************************************************************/
			/*               Checando comandos                                                        */
			/************************************************************************/

			com_t *pkg_buffer;
			pkg_buffer = com_interpretando_buffer(gau8SocketTestBuffer);
			switch (pkg_buffer->pkg_type){
				case command_LED_ON:
					puts("lED ON\N");
					send(tcp_client_socket, PACOTE_TESTE_tx_OK, sizeof(PACOTE_TESTE_tx_OK), 0);
					PIOC->PIO_CODR = LED_PIN_MASK;
					break;

				case PACOTE_ALARM_SET: ;
					/* C doesn't allow for declarations after labels,
					 * therefore this semi-collon after the ':' is
					 * necessary (it creates a new empty line)
					 */
					hour_t *h = (hour_t *) pkg_buffer->pkg_value;
					//printf("Hora: %hhu\nMinuto: %hhu\n", h->hour, h->minute);
					rtc_set_alarm(h);
					free(h);
					break;

				case command_LED_OFF:
					puts("led off \n");
					PIOC->PIO_SODR = LED_PIN_MASK;
					break;

				case command_Cafeteria_OFF:
					puts("Café OFF");
					CAFETERIA_PIO->PIO_SODR = CAFETERIA_PIN_MASK;
					break;

				case command_Cafeteria_ON:
					puts("Café ON\n");
					send(tcp_client_socket, PACOTE_TESTE_tx_OK, sizeof(PACOTE_TESTE_tx_OK), 0);
					CAFETERIA_PIO->PIO_CODR = CAFETERIA_PIN_MASK;
					break;

				case command_Buzzer_ON:
					puts("Testando Buzzer ON\n");
					send(tcp_client_socket, PACOTE_TESTE_tx_OK, sizeof(PACOTE_TESTE_tx_OK), 0);
					BUZZER_PIO->PIO_CODR = BUZZER_PIN_MASK;
					break;

				case command_Buzzer_OFF:
					puts("Buzzer OFF\n");
					BUZZER_PIO->PIO_SODR = BUZZER_PIN_MASK;
					break;

				case PACOTE_ERRO:
					printf("%s", (char *) pkg_buffer->pkg_value);
					break;

			}
			free(pkg_buffer);
			uint16 i;
			for(i=0;i< sizeof(gau8SocketTestBuffer); i++)
				gau8SocketTestBuffer[i] = 0;

			recv(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 0);

		} else {
			printf("socket_cb: recv error!\r\n");
			close(tcp_server_socket);
			tcp_server_socket = -1;
		}
	}

	break;

	default:
		break;
	}
}
/**
 * \brief Interrupt handler for the PWM controller.
 */
void PWM0_Handler(void)
{
	static uint32_t ul_duty = INIT_DUTY_VALUE;  /* PWM duty cycle rate */

	uint32_t events = pwm_channel_get_interrupt_status(PWM0);
	/* Set new duty cycle */
	g_pwm_channel_led.channel = PIN_PWM_LED1_CHANNEL;
	pwm_channel_update_duty(PWM0, &g_pwm_channel_led, 1);
}


void inicializa_pwm(){
	/* Enable PWM peripheral clock */
	pmc_enable_periph_clk(ID_PWM0);

	/* Disable PWM channels for LEDs */
	pwm_channel_disable(PWM0, PIN_PWM_LED1_CHANNEL);

	/* Set PWM clock A as PWM_FREQUENCY*PERIOD_VALUE (clock B is not used) */
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_cpu_hz()
	};

	pwm_init(PWM0, &clock_setting);
	pwm_channel_init(PWM0, &g_pwm_channel_led);

	/* Enable channel counter event interrupt */
	pwm_channel_enable_interrupt(PWM0, PIN_PWM_LED0_CHANNEL, 0);

	/* Initialize PWM channel for LED1 */
	/* Period is center-aligned */
	g_pwm_channel_led.alignment = PWM_ALIGN_CENTER;
	/* Output waveform starts at a high level */
	g_pwm_channel_led.polarity = PWM_HIGH;
	/* Use PWM clock A as source clock */
	g_pwm_channel_led.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	g_pwm_channel_led.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	g_pwm_channel_led.ul_duty = INIT_DUTY_VALUE;
	g_pwm_channel_led.channel = PIN_PWM_LED1_CHANNEL;

	pwm_channel_init(PWM0, &g_pwm_channel_led);

	/* Disable channel counter event interrupt */
	pwm_channel_disable_interrupt(PWM0, PIN_PWM_LED1_CHANNEL, 0);
	/* Configure interrupt and enable PWM interrupt */
	NVIC_DisableIRQ(PWM0_IRQn);
	NVIC_ClearPendingIRQ(PWM0_IRQn);
	NVIC_SetPriority(PWM0_IRQn, 0);
	NVIC_EnableIRQ(PWM0_IRQn);

	/* Enable PWM channels for LEDs */
	pwm_channel_enable(PWM0, PIN_PWM_LED0_CHANNEL);
	pwm_channel_enable(PWM0, PIN_PWM_LED1_CHANNEL);
}



/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] u8MsgType type of Wi-Fi notification. Possible types are:
 *  - [M2M_WIFI_RESP_CURRENT_RSSI](@ref M2M_WIFI_RESP_CURRENT_RSSI)
 *  - [M2M_WIFI_RESP_CON_STATE_CHANGED](@ref M2M_WIFI_RESP_CON_STATE_CHANGED)
 *  - [M2M_WIFI_RESP_CONNTION_STATE](@ref M2M_WIFI_RESP_CONNTION_STATE)
 *  - [M2M_WIFI_RESP_SCAN_DONE](@ref M2M_WIFI_RESP_SCAN_DONE)
 *  - [M2M_WIFI_RESP_SCAN_RESULT](@ref M2M_WIFI_RESP_SCAN_RESULT)
 *  - [M2M_WIFI_REQ_WPS](@ref M2M_WIFI_REQ_WPS)
 *  - [M2M_WIFI_RESP_IP_CONFIGURED](@ref M2M_WIFI_RESP_IP_CONFIGURED)
 *  - [M2M_WIFI_RESP_IP_CONFLICT](@ref M2M_WIFI_RESP_IP_CONFLICT)
 *  - [M2M_WIFI_RESP_P2P](@ref M2M_WIFI_RESP_P2P)
 *  - [M2M_WIFI_RESP_AP](@ref M2M_WIFI_RESP_AP)
 *  - [M2M_WIFI_RESP_CLIENT_INFO](@ref M2M_WIFI_RESP_CLIENT_INFO)
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters
 * (if any). It should be casted to the correct data type corresponding to the
 * notification type. Existing types are:
 *  - tstrM2mWifiStateChanged
 *  - tstrM2MWPSInfo
 *  - tstrM2MP2pResp
 *  - tstrM2MAPResp
 *  - tstrM2mScanDone
 *  - tstrM2mWifiscanResult
 */
static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			printf("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: CONNECTED\r\n");
			m2m_wifi_request_dhcp_client();
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			printf("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: DISCONNECTED\r\n");
			wifi_connected = 0;
			m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
		}
	}
	break;

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		wifi_connected = 1;
		printf("wifi_cb: M2M_WIFI_REQ_DHCP_CONF: IP is %u.%u.%u.%u\r\n",
				pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
	}
	break;

	default:
		break;
	}
}

/**
 * \brief Main application function.
 *
 * Initialize system, UART console, network then test function of TCP server.
 *
 * \return program return value.
 */
int main(void)
{
	tstrWifiInitParam param;
	int8_t ret;
	struct sockaddr_in addr;

	/* Initialize the board. */
	sysclk_init();
	board_init();

	PMC->PMC_PCER0 = (1<<LED_PIO_ID);
	PIOC->PIO_OER = (1 << 8);
	PIOC->PIO_PER = (1 << 8);

	PMC->PMC_PCER0 = (1<<CAFETERIA_PIO_ID);
	CAFETERIA_PIO->PIO_OER = CAFETERIA_PIN_MASK;
	CAFETERIA_PIO->PIO_PER = CAFETERIA_PIN_MASK;
	
	CAFETERIA_PIO->PIO_SODR = CAFETERIA_PIN_MASK;
	
	PMC->PMC_PCER0 = (1<<BUZZER_PIO_ID);
	BUZZER_PIO->PIO_OER = BUZZER_PIN_MASK;
	BUZZER_PIO->PIO_PER = BUZZER_PIN_MASK;

	/* Initialize the UART console. */
	configure_console();
	puts("BOARD INITIALIZED");
	printf(STRING_HEADER);

	/* Initialize the BSP. */
	nm_bsp_init();

	// Initialize RTC
	rtc_init();
	puts("RTC INITIATED");

	/* Initialize socket address structure. */
	addr.sin_family = AF_INET;
	addr.sin_port = _htons(MAIN_WIFI_M2M_SERVER_PORT);
	addr.sin_addr.s_addr = 0;

	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = wifi_cb;
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
		while (1) {
		}
	}
	puts("WIFI INITIATED");

	/* Initialize socket module */
	socketInit();
	registerSocketCallback(socket_cb, NULL);

	//inicializa_pwm();

	/* Connect to router. */
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);

	char was_conn = 0;
	puts("trying to connect...");
	while (1) {
		/* Handle pending events from network controller. */
		m2m_wifi_handle_events(NULL);

		if (wifi_connected == M2M_WIFI_CONNECTED) {
			was_conn = 1;
			if (tcp_server_socket < 0) {
				/* Open TCP server socket */
				if ((tcp_server_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
					printf("main: failed to create TCP server socket error!\r\n");
					continue;
				}

				/* Bind service*/
				bind(tcp_server_socket, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
			}
		} else if (was_conn) {
			printf("Connection lost.\n");
			break;
		}
	}

	return 0;
}
