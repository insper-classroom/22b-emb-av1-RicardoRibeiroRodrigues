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
static void io_init(void);

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

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_modo(void *pvParameters) {
	gfx_mono_ssd1306_init();
	gfx_mono_draw_string("Exemplo RTOS", 0, 0, &sysfont);
	gfx_mono_draw_string("oii", 0, 20, &sysfont);
	char angulo;

	for (;;)  {
		if (xQueueReceive(xQueueModo, &angulo, 0)) {
			int n_passos = (int)((float) angulo / 0.17578125);
			printf("Recebeu um angulo!: %d\n", angulo);
			printf("Numero de passos: %d\n", n_passos);
			xQueueSend(xQueueSteps, &n_passos, 0);
		}
	}
}

static void task_motor(void *pvParameters) {
	
	for(;;) {
		if()
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

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
