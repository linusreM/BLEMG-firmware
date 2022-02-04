#include "gd32vf103.h"
#include "liobt.h"


char rx_dma_buffer[LIO_BT_BUFFER_SIZE] = {'\0'};
char tx_dma_buffer[LIO_BT_BUFFER_SIZE] = {'\0'};


void lio_init_bt()
{
    dma_parameter_struct dma_init_struct;
    /* enable DMA0 */
    rcu_periph_clock_enable(RCU_DMA0);
    /* initialize USART */
    rcu_periph_clock_enable(RCU_GPIOA);

    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART0);

    /* connect port to USARTx_Tx */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    /* connect port to USARTx_Rx */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    /* USART configure */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);

    usart_dma_transmit_config(USART0, USART_DENT_ENABLE);
    usart_dma_receive_config(USART0, USART_DENR_ENABLE);

   

    dma_deinit(DMA0, DMA_CH4);
    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr = (uint32_t)rx_dma_buffer;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = LIO_BT_BUFFER_SIZE;
    dma_init_struct.periph_addr = (uint32_t)&USART_DATA(USART0);
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA0, DMA_CH4, &dma_init_struct);
        /* configure DMA mode */
    dma_circulation_disable(DMA0, DMA_CH4);
        /* enable DMA channel4 */
    dma_channel_enable(DMA0, DMA_CH4);
}

size_t lio_read_bt(char* buffer, uint32_t size)
{
    if(rx_dma_buffer[0] != 0)
    {
        int i = 0;
        for(; i < LIO_BT_BUFFER_SIZE && rx_dma_buffer[i] != 0 && i < size - 1; i++){
            buffer[i] = rx_dma_buffer[i];
            rx_dma_buffer[i] = 0;
        }
        buffer[i] = '\0';

        dma_parameter_struct dma_init_struct;

        dma_deinit(DMA0, DMA_CH4);
        dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
        dma_init_struct.memory_addr = (uint32_t)rx_dma_buffer;
        dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
        dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
        dma_init_struct.number = LIO_BT_BUFFER_SIZE;
        dma_init_struct.periph_addr = (uint32_t)&USART_DATA(USART0);
        dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
        dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
        dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
        dma_init(DMA0, DMA_CH4, &dma_init_struct);
        /* configure DMA mode */
        dma_circulation_disable(DMA0, DMA_CH4);
        /* enable DMA channel4 */
        dma_channel_enable(DMA0, DMA_CH4);
        return i;
    }
    return 0;
}

size_t lio_send_bt(char* message, uint32_t size)
{
    for(int i = 0; i < size && i < LIO_BT_BUFFER_SIZE; i++) {
        tx_dma_buffer[i] = message[i];
    }

    dma_parameter_struct dma_init_struct;
    dma_deinit(DMA0, DMA_CH3);
    dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_addr = (uint32_t)tx_dma_buffer;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = size;
    dma_init_struct.periph_addr = (uint32_t)&USART_DATA(USART0);
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA0, DMA_CH3, &dma_init_struct);
    /* configure DMA mode */
    dma_circulation_disable(DMA0, DMA_CH3);
    /* enable DMA channel3 */
    dma_channel_enable(DMA0, DMA_CH3);
}
