#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
#include "driver/uart.h"
#include "dados.h"

#include "esp_heap_caps.h"

#define LED_VERDE GPIO_NUM_0
#define LED_AZUL  GPIO_NUM_6
#define LED_VERMELHO GPIO_NUM_13
#define BUTTON1 GPIO_NUM_4
#define BUTTON2 GPIO_NUM_10
#define C_SELECT GPIO_NUM_5

#define X_POS 40

//SPI defines
#define SENDER_HOST SPI2_HOST
#define GPIO_HANDSHAKE 6
#define GPIO_MOSI 21
#define GPIO_MISO 19
#define GPIO_SCLK 20
#define GPIO_CS 18

#define LCD_CE GPIO_NUM_23
#define LCD_RESET GPIO_NUM_15
#define LCD_DC GPIO_NUM_22

//============================== MACROS ==============================
#define SELECIONA_CS(n) gpio_set_level(n, (uint32_t)0)
//============================== END MACROS ==============================
//Functions
void GPIO_Init();
void SPI_Init(spi_device_handle_t* handle, spi_device_interface_config_t* t_devcfg, spi_bus_config_t* t_buscfg);
void UART_Init();
void UART_INTR_Init();
char Check_UART_Data(char* data);
void Change_UART_Data(char* UART_Write);
void SPI_Write(char* dados_spi_write, int size);
void SPI_Write_Read(char *dados_spi_write, char *dados_spi_read, int size);
int Read_Button();
void Timer_Init(gptimer_handle_t *timer_handle, int time_ms, int cb, void *callback);

void ASCII_to_INT(int *ADC_Read_SPI, char *recv_spi_dados, int size);
void DFT_Calc(int n, int *x_n, int real, int complex);


/* ===== LCD Libraries =====*/
void LCD_Init();
void LCD_Clear();
void LCD_Reset();
void LCD_Write_Command(char cmd);
void LCD_Write_Data(char cmd);
void LCD_Write_String(uint8_t PosX, uint8_t PosY, char * str);
void LCD_Write_Char(uint8_t ch);
void LCD_Set_Position(uint8_t PosX, uint8_t PosY);


/*=========== Handle ==========*/
spi_transaction_t transaction;
spi_device_handle_t handle;


int counter = 0;
int flag = 0;

int Global_BT1 = 1;
int Global_BT2 = 0;

static bool level = false;
static int level_global = 0;

char State = 'S';


static void IRAM_ATTR gpio1_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    if(gpio_num == 4){
        Global_BT1 += 1;
        if(Global_BT1 > 8) Global_BT1 = 1;
    }
}

static void IRAM_ATTR gpio2_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    if(gpio_num == 7){
        Global_BT2 = 1;  
    }

    State = 'S';
}

static void timer0_callback_on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    

    if(level) level = false;
    else if(!level) level = true;
       
}

static void timer1_callback_on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    

    level_global += 1;
       
}



void app_main(void)
{
    
    spi_bus_config_t buscfg;
    spi_device_interface_config_t devcfg;
    gptimer_handle_t timer_handle_0;
    gptimer_handle_t timer_handle_1;

    esp_err_t ret;

    GPIO_Init();
    UART_Init();
    SPI_Init(&handle, &devcfg, &buscfg);
    Timer_Init(&timer_handle_0, 100 , 1, &timer0_callback_on_alarm);
    Timer_Init(&timer_handle_1, 1000, 1, &timer1_callback_on_alarm);

    if(gptimer_start(timer_handle_0) == ESP_OK) ESP_LOGI("timer", "TIMER0 STARTED!");
    if(gptimer_start(timer_handle_1) == ESP_OK) ESP_LOGI("timer", "TIMER1 STARTED!");


    spi_bus_initialize(SENDER_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SENDER_HOST, &devcfg, &handle);

    LCD_Init();
    LCD_Clear();
    LCD_Write_String(0, 0, "LCD");
    LCD_Write_String(0, 1, "Starting...");
    vTaskDelay(1000/portTICK_PERIOD_MS);
    LCD_Clear();
    LCD_Write_String(0,1, "Started!");
    vTaskDelay(1000/portTICK_PERIOD_MS);
    LCD_Clear();

    char *ptr = NULL;

    ptr = (char*) heap_caps_calloc(10, sizeof(char), MALLOC_CAP_8BIT);
    if(ptr == NULL) ESP_LOGI("MEM", "MEMORY NOT ALLOCATED!");
    else {
        for(int i = 0; i < 10; i ++) *(ptr+i) = i + 48;
        for(int i = 0; i < 10; i ++) ESP_LOGI("MEM", "DATA: %c", *(ptr+i));
        ESP_LOGI("MEM", "MEMORY ALLOCATED!");
        heap_caps_free(ptr);
    }






    spi_transaction_t transaction_1;
    memset(&transaction_1, 0, sizeof(transaction_1));

    
    char* test_str = "This is a test string.\n";
    char data[128];
    int length = 0;

    

    char send_spi_device_size[8] = {'E', '0', '0', '5', 'A', 'B', 'C', 'D'};   
    char send_spi_dados[20 * 4] = {0xFF};

    char recv_spi_device_size[20 * 4] = {0};
    char recv_spi_dados[20 * 4] = {0};


    char buffer[30];
    char buffer2[20];
    char buffer3[20];
    counter = 0;

    uint8_t bool_data = 0;
   
    int N_Conv = 5;
    int ADC_Read_SPI[20] = {0};

    int Click = 0;
    int BT_Flag = 0;


    while(1)
    {
        switch(State){
            case 'I':   //Estado que inicia os periféricos necessários
                        //Periferico + Especificação + Num1 + Num2 + Num3 + Multiplicador + Sinal + Verificacao
                        if(level)
                        {
                            char command_send[8] = {'T', '1', '5', '0'};
                            char command_send_2[8] = {'0', '3', '-', '8'};
                            char recvbuf[8] = {0};
                            spi_transaction_t trans_t;
                            memset(&trans_t, 0, sizeof(trans_t));

                            trans_t.length = sizeof(command_send) * 8;
                            trans_t.tx_buffer = command_send;
                            trans_t.rx_buffer = recvbuf;
                            gpio_set_level(C_SELECT, (uint32_t)0);
                            ret = spi_device_transmit(handle, &trans_t);
                            gpio_set_level(C_SELECT, (uint32_t)1);
                            if(ret == ESP_OK) ESP_LOGI("SPI", "Enviado1.");
                            ESP_LOGI("SPI", "Recvbuf: %s", recvbuf);

                            vTaskDelay(100/portTICK_PERIOD_MS);

                            trans_t.tx_buffer = command_send_2;
                            gpio_set_level(C_SELECT, (uint32_t)0);
                            ret = spi_device_transmit(handle, &trans_t);
                            gpio_set_level(C_SELECT, (uint32_t)1);
                            if(ret == ESP_OK) ESP_LOGI("SPI", "Enviado2.");
                            ESP_LOGI("SPI", "Recvbuf: %s", recvbuf);
                            vTaskDelay(1000/portTICK_PERIOD_MS);
                        }
                        
                        /*
                        gpio_set_level(C_SELECT, (uint32_t)0);
                        SPI_Write_Read(command_send, recvbuf, 8);
                        gpio_set_level(C_SELECT, (uint32_t)1);
                        ESP_LOGI("SPI", "SPI SENT");
                        vTaskDelay(1000/portTICK_PERIOD_MS);

                        while(gpio_get_level(GPIO_HANDSHAKE)){int x = 1;}

                        gpio_set_level(C_SELECT, (uint32_t)0);
                        SPI_Write_Read(command_send, recvbuf, 8);
                        gpio_set_level(C_SELECT, (uint32_t)1);

                        ESP_LOGI("SPI", "[0]: %d, [1]: %d,[2]: %d,[3]: %d,[4]: %c, [5]: %c,[6]: %c,[7]: %c", recvbuf[0], recvbuf[1],recvbuf[2],recvbuf[3],recvbuf[4],recvbuf[5],recvbuf[6],recvbuf[7]);
                        
                        if(!strncmp(recvbuf, "Cmd", 3)) 
                        {
                            State = 'I';
                            ESP_LOGI("SPI", "Confirmation OK!");
                        }
                        */
                        vTaskDelay(10/portTICK_PERIOD_MS);


                        

            break;
            case 'S':
                    send_spi_device_size[3] = (char)(N_Conv + 48);
                    
                    gpio_set_level(GPIO_NUM_5, (uint32_t)0);
                    SPI_Write_Read(send_spi_device_size, recv_spi_device_size, 8);
                    gpio_set_level(GPIO_NUM_5, (uint32_t)1);
                    vTaskDelay(100/portTICK_PERIOD_MS);

                    if(!gpio_get_level(GPIO_HANDSHAKE)){
                        
                        
                        gpio_set_level(GPIO_NUM_5, (uint32_t)0);
                        SPI_Write_Read(send_spi_dados, recv_spi_dados, N_Conv * 4);
                        ESP_LOGI("SPI", "Recvbuf: %s", recv_spi_dados);
                        gpio_set_level(GPIO_NUM_5, (uint32_t)1);
                    
                    }
            
                    ASCII_to_INT(ADC_Read_SPI, recv_spi_dados, 8);

                    if(counter > 999){ 
                        counter = 0; 
                        LCD_Clear();
                    }
                    LCD_Clear();

                    if(N_Conv > 0) {
                        sprintf(buffer2, "0-%d", ADC_Read_SPI[0]);
                        LCD_Write_String(0,0, buffer2);
                    }
                    if(N_Conv > 1) {
                        sprintf(buffer2, "1-%d", ADC_Read_SPI[1]);
                        LCD_Write_String(X_POS,0, buffer2);
                    }
                    if(N_Conv > 2) {
                        sprintf(buffer2, "2-%d", ADC_Read_SPI[2]);
                        LCD_Write_String(0,1, buffer2);
                    }
                    if(N_Conv > 3) {
                        sprintf(buffer2, "3-%d", ADC_Read_SPI[3]);
                        LCD_Write_String(X_POS,1, buffer2);
                    }
                    if(N_Conv > 4) {
                        sprintf(buffer2, "4-%d", ADC_Read_SPI[4]);
                        LCD_Write_String(0,2, buffer2);
                    }
                    if(N_Conv > 5) {
                        sprintf(buffer2, "5-%d", ADC_Read_SPI[5]);
                        LCD_Write_String(X_POS,2, buffer2);
                    }
                    if(N_Conv > 6) {
                        sprintf(buffer2, "6-%d", ADC_Read_SPI[6]);
                        LCD_Write_String(0,3, buffer2);
                    }
                    if(N_Conv > 7) {
                        sprintf(buffer2, "7-%d", ADC_Read_SPI[7]);
                        LCD_Write_String(X_POS,3, buffer2);
                    }
                    sprintf(buffer, "S:%d NC:%d %d", Global_BT2, Global_BT1, N_Conv);
                    LCD_Write_String(0,5, buffer);
                    vTaskDelay(1000/portTICK_PERIOD_MS);
                    
                    State = 'P';
            break;
            case 'P':
                    if(Global_BT2 == 1){
                        N_Conv = Global_BT1;
                        Global_BT2 = 0;
                    }
                    LCD_Clear();
                    
                    
                    /*
                    if(N_Conv > 0) {
                        sprintf(buffer2, "V0-%.1f", (float)(((ADC_Read_SPI[0]*3.3) / 4095)));
                        LCD_Write_String(0,3, buffer2);
                    }
                    if(N_Conv > 1) {
                        sprintf(buffer2, "V1-%.1f", (float)(((ADC_Read_SPI[1]* 3.3) / 4095)));
                        LCD_Write_String(X_POS,3, buffer2);
                    }
                    if(N_Conv > 2) {
                        sprintf(buffer2, "V2-%.1f", (float)(((ADC_Read_SPI[2]* 3.3) / 4095)));
                        LCD_Write_String(0,4, buffer2);
                    }
                    */
                    sprintf(buffer2, "S:%d NC:%d %d", Global_BT2, Global_BT1, N_Conv);
                    LCD_Write_String(0,5, buffer2);
                    vTaskDelay(500/portTICK_PERIOD_MS);
                    State = 'W';
            break;
            case 'W':
                    LCD_Clear();
                    
                    LCD_Write_String(0, 2, "Calculando");
                    
                    sprintf(buffer, "DFT... %d", level_global);
                 
                    if(level) LCD_Write_String(0, 3, buffer);
                    else if(!level) LCD_Write_String(0, 3, "");
                    sprintf(buffer2, "S:%d NC:%d %d", Global_BT2, Global_BT1, N_Conv);
                    LCD_Write_String(0,5, buffer2);
                    vTaskDelay(100/portTICK_PERIOD_MS);

            break;

        }
       







        /*
        switch(State)
        {
            case 's':   
                        gpio_set_level(LED_VERMELHO, (uint32_t)1);
                        gpio_set_level(LED_VERDE, (uint32_t)0);                                  
                        gpio_set_level(LED_AZUL, (uint32_t)0);
                        //LCD_Clear();
                        LCD_Write_String(0, 2, "Vermelho");
                        
                        uart_get_buffered_data_len(UART_NUM_0, (size_t*)&length);
                        
                        if(length != 0) {
                            uart_read_bytes(UART_NUM_0, data, length, 100);
                            State = Check_UART_Data(&data);
                            uart_write_bytes(UART_NUM_0, "DoneR\n", 6);
                        }
                        
            break;
            case 'x':   
                        gpio_set_level(LED_VERMELHO, (uint32_t)0);
                        gpio_set_level(LED_VERDE, (uint32_t)1);                                  
                        gpio_set_level(LED_AZUL, (uint32_t)0);
                        LCD_Clear();
                        LCD_Write_String(0, 2, "Verde");

                        uart_get_buffered_data_len(UART_NUM_0, (size_t*)&length);

                        if(length != 0) {
                            uart_read_bytes(UART_NUM_0, data, length, 100);
                            State = Check_UART_Data(&data);
                            uart_write_bytes(UART_NUM_0, "DoneG\n", 6);
                        }
                        
                        
            break;
            case 'H':   
                        gpio_set_level(LED_VERMELHO, (uint32_t)0);
                        gpio_set_level(LED_VERDE, (uint32_t)0);                                  
                        gpio_set_level(LED_AZUL, (uint32_t)1);
                        LCD_Clear();
                        LCD_Write_String(0, 2, "Azul");
                        uart_get_buffered_data_len(UART_NUM_0, (size_t*)&length);

                        if(length != 0) {
                            uart_read_bytes(UART_NUM_0, data, length, 100);
                            State = Check_UART_Data(&data);
                            uart_write_bytes(UART_NUM_0, "DoneB\n", 6);
                        }
            break;
        }
        */
    }
}

//=============== OTHER FUNCTIONS ===============
void ASCII_to_INT(int *ADC_Read_SPI, char *recv_spi_dados, int size)
{
    for(int i = 0; i < size; i ++)
    {
        ADC_Read_SPI[i] = ((recv_spi_dados[i * 4] - 48) * 1000) + ((recv_spi_dados[i * 4 + 1] - 48) * 100) + ((recv_spi_dados[i * 4 + 2] - 48) * 10) + ((recv_spi_dados[i * 4 + 3] - 48) * 1); 
    }
}

void DFT_Calc(int n, int *x_n, int real, int complex)
{

}
//=============== END OTHER FUNCT ===============

//=============== INIT  FUNCTIONS ===============
char Check_UART_Data(char* data){

        char State = NULL;
        if(!strncmp(data, "StateR", 6)) State = 'R';
        else if(!strncmp(data, "StateG", 6)) State = 'G';
        else if(!strncmp(data, "StateB", 6)) State = 'B';
        
        return State;
}

void Change_UART_Data(char* UART_Write)
{

}

void GPIO_Init()
{
     gpio_config_t config_19_t = {
        .pin_bit_mask = 1 << LCD_CE,
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config_t config_20_t = {
        .pin_bit_mask = 1 << LCD_RESET,
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config_t config_21_t = {
        .pin_bit_mask = 1 << LCD_DC,
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };

    gpio_config_t config_15_t = {
        .pin_bit_mask = 1 << LED_VERDE,
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config_t config_23_t = {
        .pin_bit_mask = 1 << LED_AZUL,
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config_t config_22_t = {
        .pin_bit_mask = 1 << LED_VERMELHO,
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };


    gpio_config_t config_5_t = {
        .pin_bit_mask = 1 << 5,
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config_t config_HS_t = {
        .pin_bit_mask = 1 << GPIO_HANDSHAKE,
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };

    gpio_config_t config_BT1_t = {
        .pin_bit_mask = 1 << BUTTON1,
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };

    gpio_config_t config_BT2_t = {
        .pin_bit_mask = 1 << BUTTON2,
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };

    gpio_set_intr_type(BUTTON1, GPIO_INTR_NEGEDGE);
    gpio_set_intr_type(BUTTON2, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    if(gpio_isr_handler_add(BUTTON1, gpio1_isr_handler, (void*)4) == ESP_OK) ESP_LOGI("ISR", "ISR GPIO %d OK!", BUTTON1);
    if(gpio_isr_handler_add(BUTTON2, gpio2_isr_handler, (void*)7) == ESP_OK) ESP_LOGI("ISR", "ISR GPIO %d OK!", BUTTON2);

    gpio_config(&config_19_t);
    gpio_config(&config_20_t);
    gpio_config(&config_21_t);
    gpio_config(&config_15_t);
    gpio_config(&config_23_t);
    gpio_config(&config_22_t);
    gpio_config(&config_5_t);
    gpio_config(&config_HS_t);
    gpio_config(&config_BT1_t);
    gpio_config(&config_BT2_t);

}

void SPI_Init(spi_device_handle_t* handle, spi_device_interface_config_t* t_devcfg, spi_bus_config_t* t_buscfg)
{
    esp_err_t ret;
    
    spi_bus_config_t buscfg={
        .mosi_io_num=GPIO_MOSI,
        .miso_io_num=GPIO_MISO,
        .sclk_io_num=GPIO_SCLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
   
    spi_device_interface_config_t devcfg = {
        .command_bits=0,
        .address_bits=0,
        .dummy_bits=0,
        .clock_speed_hz=1000000,
        .duty_cycle_pos=128,        //50% duty cycle
        .mode=0,
        .spics_io_num=-1,
        .cs_ena_posttrans=3,        //Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .queue_size=3
    };

    *t_devcfg = devcfg;
    *t_buscfg = buscfg;

}

void UART_Init()
{
    QueueHandle_t uart_queue;
    
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, 2048, 2048, 10, &uart_queue,0);
}

void UART_INTR_Init()
{
    
}

void Timer_Init(gptimer_handle_t *timer_handle, int time_ms, int cb, void *callback)
{
    //handle local para configurar e retornar configurado.
    gptimer_handle_t local_timer_handle;

    const int CLOCK = 80E6;
    
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1E6,
    };

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = (time_ms*1000),
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true, 
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &local_timer_handle));
    ESP_ERROR_CHECK(gptimer_set_alarm_action(local_timer_handle, &alarm_config));
    if(cb != NULL)
    {
        gptimer_event_callbacks_t timer_callback = {
            .on_alarm = callback,
        };
        ESP_ERROR_CHECK(gptimer_register_event_callbacks(local_timer_handle, &timer_callback, NULL));
    }
    ESP_ERROR_CHECK(gptimer_enable(local_timer_handle));
    //ESP_ERROR_CHECK(gptimer_start(local_timer_handle));

    *timer_handle = local_timer_handle;

}
//=============== END INIT FUNCTIONS ===============

void SPI_Write_Read(char *dados_spi_write, char *dados_spi_read, int size)
{
    spi_transaction_t transaction_local;
    memset(&transaction_local, 0, sizeof(transaction_local));
    char rec[size];
    char send[size];
    for(int i = 0; i < size; i ++) send[i] = dados_spi_write[i];
    transaction_local.length = sizeof(send) * 8;
    transaction_local.tx_buffer = send;
    transaction_local.rx_buffer = rec;

    spi_device_transmit(handle, &transaction_local);

    //dados_spi_read = rec;
    for(int i = 0; i < size; i ++) *(dados_spi_read + i) = rec[i];

}

void SPI_Write(char* dados_spi_write, int size)
{
    spi_transaction_t transaction_local;
    memset(&transaction_local, 0, sizeof(transaction_local));
    char rec[size];
    char send[size];
    for(int i = 0; i < size; i ++) send[i] = dados_spi_write[i];
    transaction_local.length = sizeof(send) * 8;
    transaction_local.tx_buffer = send;
    transaction_local.rx_buffer = rec;

    spi_device_transmit(handle, &transaction_local);
}

int Read_Button()
{
    static int BT_Flag = 0;
    static int Click = 0;

    if(gpio_get_level(BUTTON1) && !BT_Flag) 
    {
        BT_Flag = 1;
        Click = 0;
    }
    else if(!gpio_get_level(BUTTON1) && BT_Flag){
        BT_Flag = 0;
        Click = 1;
    } 

    return Click;

}

void LCD_Init()
{
    LCD_Reset();
    LCD_Write_Command(0x21);
    LCD_Write_Command(0xD0);
    LCD_Write_Command(0x20);
    LCD_Write_Command(0x0C);
    LCD_Clear();
}

void LCD_Reset()
{
    gpio_set_level(LCD_RESET, (uint32_t)0);
    vTaskDelay(100/portTICK_PERIOD_MS);
    gpio_set_level(LCD_RESET, (uint32_t)1);
    
}

void LCD_Clear()
{
    uint8_t t;
    uint8_t k;
    LCD_Set_Position(0,0);
    for(t=0;t<6;t++)
    {
      for(k=0;k<84;k++)
      { 
        LCD_Write_Data(0x00);
      }
    }
}

void LCD_Write_Command(char cmd)
{
    gpio_set_level(LCD_CE, (uint32_t)0);
    gpio_set_level(LCD_DC, (uint32_t)0);
    SPI_Write(&cmd, 1);
    gpio_set_level(LCD_CE, (uint32_t)1);
    gpio_set_level(LCD_DC, (uint32_t)1);
}
 
void LCD_Write_Data(char cmd)
{
    gpio_set_level(LCD_CE, (uint32_t)0);
    gpio_set_level(LCD_DC, (uint32_t)1);
    SPI_Write(&cmd, 1);
    gpio_set_level(LCD_CE, (uint32_t)1);
    gpio_set_level(LCD_DC, (uint32_t)0);
}

void LCD_Write_String(uint8_t PosX, uint8_t PosY, char * str){
    LCD_Set_Position(PosX, PosY);
    while(* str){
        LCD_Write_Char(* str);
        str ++;
    }
}

void LCD_Write_Char(uint8_t ch){
    uint8_t line;
    ch -= 32;
    for (line=0; line<6; line++) LCD_Write_Data(font6x8[ch][line]);

}

void LCD_Set_Position(uint8_t PosX, uint8_t PosY){
    LCD_Write_Command(0x40 | PosY);
    LCD_Write_Command(0x80 | PosX);
}

