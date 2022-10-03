#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

// Configurações botao 1
#define BUT_1_PIO PIOD
#define BUT_1_PIO_ID ID_PIOD
#define BUT_1_IDX 28
#define BUT_1_IDX_MASK (1u << BUT_1_IDX)

// Configurações botao 2
#define BUT_2_PIO PIOC
#define BUT_2_PIO_ID ID_PIOC
#define BUT_2_IDX 31
#define BUT_2_IDX_MASK (1u << BUT_2_IDX)

// Configurações botao 3
#define BUT_3_PIO PIOA
#define BUT_3_PIO_ID ID_PIOA
#define BUT_3_IDX 19
#define BUT_3_IDX_MASK (1u << BUT_3_IDX)

// in1 motor
#define IN1_PIO PIOD
#define IN1_PIO_ID ID_PIOD
#define IN1_IDX 30
#define IN1_IDX_MASK (1u << IN1_IDX)

// in2 motor
#define IN2_PIO PIOA
#define IN2_PIO_ID ID_PIOA
#define IN2_IDX 6
#define IN2_IDX_MASK (1u << IN2_IDX)

// in3 motor
#define IN3_PIO PIOC
#define IN3_PIO_ID ID_PIOC
#define IN3_IDX 19
#define IN3_IDX_MASK (1u << IN3_IDX)

// in4 motor
#define IN4_PIO PIOA
#define IN4_PIO_ID ID_PIOA
#define IN4_IDX 2
#define IN4_IDX_MASK (1u << IN4_IDX)

#define WIDTH_OLED 128
#define HEIGHT_OLED 32

typedef struct {
	Pio *p_pio; 
	const uint32_t ul_mask;
} entrada_motor;


/** RTOS  */
#define TASK_MODO_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_MODO_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_MOTOR_STACK_SIZE               (1024*6/sizeof(portSTACK_TYPE))
#define TASK_MOTOR_STACK_PRIORITY           (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

xQueueHandle xQueueModo;
xQueueHandle xQueueSteps;

xSemaphoreHandle xSemaphoreRTT;

/** prototypes */
void but_callback(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
static void io_init(void);
void desaciona_motor(entrada_motor in_n);
void aciona_motor(entrada_motor in_n);
void gfx_clear(void);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but_callback() {
	
}

void but_1_callback(void) {
	int angulo = 45;
	xQueueSendFromISR(xQueueModo, &angulo, 0);
}

void but_2_callback(void) {
	int angulo = 90;
	xQueueSendFromISR(xQueueModo, &angulo, 0);
}

void but_3_callback(void){
	int angulo = 180;
	xQueueSendFromISR(xQueueModo, &angulo, 0);
}

void RTT_Handler(void) {
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		xSemaphoreGiveFromISR(xSemaphoreRTT, 0);
	}
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_modo(void *pvParameters) {
	gfx_mono_ssd1306_init();
	
	char angulo;

	for (;;)  {
		if (xQueueReceive(xQueueModo, &angulo, 0)) {
			int n_passos = (int)((float) angulo / 0.17578125);
			printf("Recebeu um angulo!: %d\n", angulo);
			printf("Numero de passos: %d\n", n_passos);
			gfx_clear();
			char str[99];
			sprintf(str, "Angulo = %d", angulo);
			gfx_mono_draw_string(str, 0, 0, &sysfont);
			xQueueSend(xQueueSteps, &n_passos, 0);
		}
	}
}

static void task_motor(void *pvParameters) {
	int n_passos;
	int contador_passo = 0;
	int i;
	
	entrada_motor in_1 = {IN1_PIO, IN1_IDX_MASK};
	entrada_motor in_2 = {IN2_PIO, IN2_IDX_MASK};
	entrada_motor in_3 = {IN3_PIO, IN3_IDX_MASK};
	entrada_motor in_4 = {IN4_PIO, IN4_IDX_MASK};
	entrada_motor vetor_entradas[] = {in_1, in_2, in_3, in_4};
	
	
	for(;;) {
		if(xQueueReceive(xQueueSteps, &n_passos, 0)) {
			printf("Recebeu o numero de passos! %d\n", n_passos);
			i = 1;
			contador_passo = 0;
			aciona_motor(vetor_entradas[contador_passo]);
		}
		
		if ((xSemaphoreTake(xSemaphoreRTT, 0) == pdPASS) && (i <= n_passos)) {
			printf("Chegou aqui, i = %d, passo = %d\n", i, contador_passo);
			desaciona_motor(vetor_entradas[contador_passo]);
			contador_passo++;
			if (contador_passo > 3) {
				contador_passo = 0;
			}
			i++;
			aciona_motor(vetor_entradas[contador_passo]);
		}
		if (i > n_passos) {
			desaciona_motor(vetor_entradas[contador_passo]);
		}
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void aciona_motor(entrada_motor in_n) {
	pio_set(in_n.p_pio, in_n.ul_mask);
	RTT_init(1000, 50, RTT_MR_ALMIEN);
}

void desaciona_motor(entrada_motor in_n) {
	pio_clear(in_n.p_pio, in_n.ul_mask);
}

/** 
 * Configura RTT
 *
 * arg0 pllPreScale  : Frequência na qual o contador irá incrementar
 * arg1 IrqNPulses   : Valor do alarme 
 * arg2 rttIRQSource : Pode ser uma 
 *     - 0: 
 *     - RTT_MR_RTTINCIEN: Interrupção por incremento (pllPreScale)
 *     - RTT_MR_ALMIEN : Interrupção por alarme
 */
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

  uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
  rtt_sel_source(RTT, false);
  rtt_init(RTT, pllPreScale);
  
  if (rttIRQSource & RTT_MR_ALMIEN) {
	uint32_t ul_previous_time;
  	ul_previous_time = rtt_read_timer_value(RTT);
  	while (ul_previous_time == rtt_read_timer_value(RTT));
  	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
  }

  /* config NVIC */
  NVIC_DisableIRQ(RTT_IRQn);
  NVIC_ClearPendingIRQ(RTT_IRQn);
  NVIC_SetPriority(RTT_IRQn, 4);
  NVIC_EnableIRQ(RTT_IRQn);

  /* Enable RTT interrupt */
  if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
  else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
		  
}


static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

static void io_init(void) {
	/* configura prioridae */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);

	/* conf botão como entrada */
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback);
	
	pmc_enable_periph_clk(BUT_1_PIO_ID);
	pio_set_input(BUT_1_PIO, BUT_1_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);
	pio_pull_up(BUT_1_PIO, BUT_1_IDX_MASK, 1);

	// Configura handler para o botao 1 para interrupcao
	pio_handler_set(BUT_1_PIO,
		BUT_1_PIO_ID,
		BUT_1_IDX_MASK,
		PIO_IT_FALL_EDGE,
		but_1_callback);

	// Ativa interrupção e limpa primeira IRQ do botao 1 gerada na ativacao
	pio_enable_interrupt(BUT_1_PIO, BUT_1_IDX_MASK);
	pio_get_interrupt_status(BUT_1_PIO);

	// Configura NVIC para receber interrupcoes do PIO do botao 1
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT_1_PIO_ID);
	NVIC_SetPriority(BUT_1_PIO_ID, 5);


	pmc_enable_periph_clk(BUT_2_PIO_ID);
	pio_set_input(BUT_2_PIO, BUT_2_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);
	pio_pull_up(BUT_2_PIO, BUT_2_IDX_MASK, 1);

	// Configura handler para o botao 1 para interrupcao
	pio_handler_set(BUT_2_PIO,
		BUT_2_PIO_ID,
		BUT_2_IDX_MASK,
		PIO_IT_FALL_EDGE,
		but_2_callback);

	// Ativa interrupção e limpa primeira IRQ do botao 1 gerada na ativacao
	pio_enable_interrupt(BUT_2_PIO, BUT_2_IDX_MASK);
	pio_get_interrupt_status(BUT_2_PIO);

	// Configura NVIC para receber interrupcoes do PIO do botao 1
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT_2_PIO_ID);
	NVIC_SetPriority(BUT_2_PIO_ID, 5);

	pmc_enable_periph_clk(BUT_3_PIO_ID);
	pio_set_input(BUT_3_PIO, BUT_3_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);
	pio_pull_up(BUT_3_PIO, BUT_3_IDX_MASK, 1);

	// Configura handler para o botao 1 para interrupcao
	pio_handler_set(BUT_3_PIO,
		BUT_3_PIO_ID,
		BUT_3_IDX_MASK,
		PIO_IT_FALL_EDGE,
		but_3_callback);

	// Ativa interrupção e limpa primeira IRQ do botao 1 gerada na ativacao
	pio_enable_interrupt(BUT_3_PIO, BUT_3_IDX_MASK);
	pio_get_interrupt_status(BUT_3_PIO);

	// Configura NVIC para receber interrupcoes do PIO do botao 1
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT_3_PIO_ID);
	NVIC_SetPriority(BUT_3_PIO_ID, 5);

	// --------------------- Ativa os PIOs do motor ---------------------
	pmc_enable_periph_clk(IN1_PIO_ID);
	pio_set_output(IN1_PIO, IN1_IDX_MASK, 0, 0, 0);
	
	pmc_enable_periph_clk(IN2_PIO_ID);
	pio_set_output(IN2_PIO, IN2_IDX_MASK, 0, 0, 0);
	
	pmc_enable_periph_clk(IN3_PIO_ID);
	pio_set_output(IN3_PIO, IN3_IDX_MASK, 0, 0, 0);
	
	pmc_enable_periph_clk(IN4_PIO_ID);
	pio_set_output(IN4_PIO, IN4_IDX_MASK, 0, 0, 0);
}

void gfx_clear(void) {
	for (int x = 0; x < WIDTH_OLED; x++) {
		for (int y = 0; y < HEIGHT_OLED; y++) {
			gfx_mono_draw_pixel(x, y, GFX_PIXEL_CLR);
		}
	}
}


/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	io_init();

	/* Initialize the console uart */
	configure_console();

	/* Create task to control oled */
	if (xTaskCreate(task_modo, "Modo", TASK_MODO_STACK_SIZE, NULL, TASK_MODO_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create modo task\r\n");
	}
	
	if (xTaskCreate(task_motor, "Motor", TASK_MOTOR_STACK_SIZE, NULL, TASK_MOTOR_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create motor task\r\n");
	}
	
	xSemaphoreRTT = xSemaphoreCreateBinary();
	
	if (xSemaphoreRTT == NULL)
		printf("falha em criar o semaforo \n");
	
	// cria fila de 32 slots de char
	xQueueModo = xQueueCreate(32, sizeof(int));

	// verifica se fila foi criada corretamente
	if (xQueueModo == NULL)
		printf("falha em criar a fila \n");
	
	xQueueSteps = xQueueCreate(32, sizeof(int));
	if (xQueueSteps == NULL)
		printf("Falha ao criar a fila\n");
		
		
	

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
