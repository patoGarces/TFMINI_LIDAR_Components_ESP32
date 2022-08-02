#include <stdio.h>
#include "include/tfMini.h"
#include "driver/uart.h"
#include "string.h"


// const int rxPin = 14;
// const int txPin = 12;
// const int uartPort= UART_NUM_2;


const int rxPin = 26;
const int txPin = 25;
const int uartPort= UART_NUM_1;


uint16_t muestras=0;

/*
 * Uso esta funcion para unir 2 uint8
 */
uint16_t makeUint16(uint8_t low,uint8_t high){
    return (uint16_t)(high * 0xff) + low;
}

/*
 *      Tarea de recepcion
 *      Los datos entrantes del UART van a parar a un buffer  FIFO
 *      esta funcion se llama a la misma frecuencia que al tasa de muestreo del sensor para procesar cada frame
 */
void receiveTask(void *pvParameters ){

    /*read Data*/
    uint8_t data[9];
    int length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(uartPort, (size_t*)&length));
    length = uart_read_bytes(uartPort, data, length, 100);

    while(1){
        
        uart_get_buffered_data_len(uartPort, (size_t*)&length);          // obtengo datos para leer
        if(length > 0){    
            length = uart_read_bytes(uartPort,data,length,100);

            if(data[0] == 0x59 && data[1] == 0x59){

                uint8_t checksum= data[8];     
                uint8_t calcChecksum = data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6] + data[7];
                if( checksum == calcChecksum){

                    uint16_t dist = makeUint16(data[2],data[3]);
                    uint16_t strength = makeUint16(data[4],data[5]);
                    uint8_t mode = data[6];
                    //uint8_t cero= data[7];                                      // el datasheet dice que deberia ser 0 pero en la practica es 9 y el checksum da correcto
                    
                    muestras++;
                    printf("PAQUETE CORRECTO: distancia: %dcms, strength: %d,modo: %d\n",dist,strength,mode);
                }
                else{
                    printf("ERROR1\n");
                    uart_flush(uartPort);               // limpio el buffer 
                }
            }
            else{
                printf("ERROR2\n");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void tfMiniInit(void){

    /*configuro periferico*/
    uart_config_t uart_config={
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .parity = UART_PARITY_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        //.source_clk,
        .stop_bits = UART_STOP_BITS_1,
        //.use_ref_tick = 
    };
    uart_param_config(uartPort,&uart_config);

    /* configuro pines a utilizar por el uart*/
    uart_set_pin(uartPort,txPin,rxPin,-1,-1);


    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    uart_driver_install(uartPort, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0);


    // Write data to UART.
    char* test_str = "This is a test string.\n";
    uart_write_bytes(uartPort, (const char*)test_str, strlen(test_str));

    printf("Inicializado");

    xTaskCreate(&receiveTask,"tarea recepcion", 4096, NULL, 3 , NULL);

    while(1){

        printf("Velocidad de muestreo: %d\n",muestras);
        muestras=0;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}