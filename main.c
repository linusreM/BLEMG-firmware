/* 
Replace this file with your code. Put your source files in this directory and any libraries in the lib folder. 
If your main program should be assembly-language replace this file with main.S instead.

Libraries (other than vendor SDK and gcc libraries) must have .h-files in /lib/[library name]/include/ and .c-files in /lib/[library name]/src/ to be included automatically.
*/

#include "gd32vf103.h"
#include "systick.h"
#include "liobt.h"
#include "string.h"
#include "stdio.h"



#define SERVICE_UUID "4028f84c05c04181843ebdbee6e1030d"
#define RX_CHARACTERISTIC_UUID "e399efc079f94e0882a8f3aa1dc609f2"
#define RX_MTU "14" //Hex
#define RX_PROPERTY "0C" //
#define RX_HANDLE "0072"
#define TX_CHARACTERISTIC_UUID "e399efc079f94e0882a8f3aa1dc609f1"
#define TX_MTU "14" //Hex
#define TX_PROPERTY "1C" //
#define TX_HANDLE "0074"

enum command_enum {
	COMMAND_MODE,
	SET_SERVICE,
	SET_RX_CHARACTERISTIC,
	SET_TX_CHARACTERISTIC,
	READ_HANDLE,
	WRITE_HANDLE,
	
};

static const char* commands[] = {
	[COMMAND_MODE] = "$$$" "\r\n",
	[SET_SERVICE] = "PS," SERVICE_UUID "\r\n",
	[SET_RX_CHARACTERISTIC] = "PC," RX_CHARACTERISTIC_UUID "," RX_PROPERTY "," RX_MTU "\r\n",
	[SET_TX_CHARACTERISTIC] = "PC," TX_CHARACTERISTIC_UUID "," TX_PROPERTY "," TX_MTU "\r\n",
	[READ_HANDLE] = "SHR,",
	[WRITE_HANDLE] = "SHW,"
};

//Indicate: 			0b00100000   
//Notify:				0b00010000      
//Write:				0b00001000  
//Write w/o response:	0b00000100  
//Read:					0b00000010  

// N =   1
// I =   2
// I+N = 3

// R =           2
// Wo =          4
// Wo + R =      6
// Wr =          8
// Wr + R =      A    
// Wr + Wo =     C
// Wr + Wo + R = E

//Service:           PS,4028f84c05c04181843ebdbee6e1030d
// 4028 f84c 05c0 4181 843e bdbe e6e1 030d
//Characteristic RX: PC,e399efc079f94e0882a8f3aa1dc609f2,0c,14
//Characteristic TX: PC,e399efc079f94e0882a8f3aa1dc609f1,1c,14

void register_gatt_service(){

	uint8_t command_buffer[256] = {0};
	uint8_t response_buffer[1024] = {0};

	strcpy(command_buffer, commands[SET_SERVICE]);
}

int send_command_with_response(uint8_t* pCommand, uint8_t* response_buffer, uint32_t timeout_ms){
	return 0;
}

void send_command(uint8_t* pCommand){

}

int read_handle(uint16_t handle, uint8_t* response_buffer, uint32_t timeout){
	//Construct command
	uint8_t string_buffer[16] = {'\0'};
	uint8_t receive_buffer[512] = {'\0'};
	uint8_t finish_state[] = "CMD>";
	sprintf(string_buffer, "%s,%.4x\r\n");
	//Flush rx buffer
	lio_read_bt(receive_buffer, 512);
	//Send command
	lio_send_bt(string_buffer, strlen(string_buffer));
	
	for(int i = 0, msg_state = 0; timeout > 0; timeout--){
		//Get new characters
		int read_chars = 0;
		read_chars = lio_read_bt(&receive_buffer[i], 512-i);

		for(;read_chars > 0; read_chars--, ++i){
			if(receive_buffer[i] == finish_state[msg_state]) msg_state++; 
			else msg_state = 0;
			if(finish_state[msg_state] == '\0'){
				receive_buffer[i - strlen(finish_state)] = '\0';
				strncpy(response_buffer, receive_buffer, 512);
				return 1;
			}
		}
		return 0;
	}
	
	//wait for response
	return 0;
}

void write_handle_raw(uint16_t handle, uint8_t* byte_data, uint8_t size){
	uint8_t string_buffer[128] = {'\0'};
	uint8_t data_buffer[(20*2)+1] = {'\0'};
	const uint8_t lut_hex[] = "0123456789ABCDEF";
	for(int i = 0; i < size; i++){
		data_buffer[i*2] = lut_hex[byte_data[i] / 16];
		data_buffer[(i*2)+1] = lut_hex[byte_data[i] % 16];
	}
	data_buffer[size*2] = '\0';
	sprintf(string_buffer, "%s%.4x,%s\r\n", commands[WRITE_HANDLE], handle, data_buffer);
	lio_send_bt(string_buffer, strlen(string_buffer));
}

void rcu_config(void)
{
    rcu_periph_clock_enable(RCU_ADC0);
	rcu_periph_clock_enable(RCU_DMA0);
	rcu_periph_clock_enable(RCU_TIMER1);
	rcu_periph_clock_enable(RCU_GPIOA);
}

//Setup timer 1 to trigger adc to measure every 1 ms
int timer_config(uint32_t sample_rate)
{
	// Max 2000hz 108
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    /* deinit a timer */
    timer_deinit(TIMER1);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER1 configuration */
    timer_initpara.prescaler         = 5399; //  108 000 000
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = sample_rate;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1,&timer_initpara);

    /* CH0 configuration in PWM mode1 */
    timer_channel_output_struct_para_init(&timer_ocintpara);
    timer_ocintpara.ocpolarity  = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
    timer_channel_output_config(TIMER1, TIMER_CH_1, &timer_ocintpara);

    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, 1);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_1, TIMER_OC_MODE_PWM1);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);

	timer_auto_reload_shadow_enable(TIMER1);
}

void adc_config(void)
{
	gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    /* reset ADC */
    adc_deinit(ADC0);
    /* ADC mode config */
    adc_mode_config(ADC_MODE_FREE);
    /* ADC continous function enable */
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 1);
 
    /* ADC inserted channel config */
    adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_0, ADC_SAMPLETIME_55POINT5);
   
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_EXTTRIG_REGULAR_T1_CH1); 
    
    /* ADC external trigger enable */
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);
    /* clear the ADC flag 
    adc_interrupt_flag_clear(ADC1, ADC_INT_FLAG_EOC);
    adc_interrupt_flag_clear(ADC1, ADC_INT_FLAG_EOIC);*/
    /* enable ADC interrupt 
    adc_interrupt_enable(ADC1, ADC_INT_EOIC); */

	adc_dma_mode_enable(ADC0);

    /* enable ADC interface */
    adc_enable(ADC0);
    delay_1ms(1);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);
}

uint16_t *pSample_buffer;
uint32_t sample_buffer_size;

void dma_config(uint16_t *pBuffer, uint32_t samples)
{
	pSample_buffer = pBuffer;
	sample_buffer_size = samples;
    /* ADC_DMA_channel configuration */
    dma_parameter_struct dma_data_parameter;
    rcu_periph_clock_enable(RCU_DMA0); 
    /* ADC DMA_channel configuration */
    dma_deinit(DMA0, DMA_CH0);
    
    /* initialize DMA data mode */
    dma_data_parameter.periph_addr  = (uint32_t)(&ADC_RDATA(ADC0));
    dma_data_parameter.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_data_parameter.memory_addr  = (uint32_t)(pBuffer);
    dma_data_parameter.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_16BIT;  
    dma_data_parameter.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_data_parameter.number       = samples;
    dma_data_parameter.priority     = DMA_PRIORITY_HIGH;
    dma_init(DMA0, DMA_CH0, &dma_data_parameter);

	dma_interrupt_enable(DMA0, DMA_CH0, DMA_INT_FTF);
	dma_interrupt_enable(DMA0, DMA_CH0, DMA_INT_HTF);

    dma_circulation_enable(DMA0, DMA_CH0);
  
    /* enable DMA channel */
    dma_channel_enable(DMA0, DMA_CH0);
}

void config_clic_irqs (void)
{
    eclic_global_interrupt_enable();
    eclic_priority_group_set(ECLIC_PRIGROUP_LEVEL3_PRIO1);
    eclic_irq_enable(DMA0_Channel0_IRQn, 1, 1);

}

int main(){

	//Start of program

	
	
	uint8_t enter_data_mode[] = "$$$";

	uint16_t sample_data[4000] = {0xaa};
	rcu_config();
	timer_config(20);
	adc_config();

	//Set samples to 20, which since we are using double buffers it will generate an interrupt every ten samples.
	dma_config(sample_data, 20);
	

	lio_init_bt();
	config_clic_irqs();
	delay_1ms(1000);
	lio_send_bt(enter_data_mode, strlen(enter_data_mode));
	delay_1ms(50);
	timer_enable(TIMER1);

	while(1){
		//This program does nothing in the main loop currently. So it would be entirely possible to add some (not timing critical) stuff in here
		__WFI();
	}
}


//This is the main service interrupt. 
//Currently just sends the raw buffer unmodified, 
//it double buffers by using the halftime flag to 
//avoid memory corruption.


void DMA0_Channel0_IRQHandler(void)
{
	uint8_t* pSend_data;
	//If you want to do some transformation it's probably best to first copy the sample data.
	//Then perform transformation, and then send the transformed data array.

	//Since this is basically all of the code that is running on the MCU there should be a decent amount of time
	//to do calculations, although floats should probably be discouraged since no FPU is present.
    if(dma_interrupt_flag_get(DMA0, DMA_CH0, DMA_INT_FLAG_HTF)){ 
		pSend_data = (uint8_t*)pSample_buffer;
    }
	if(dma_interrupt_flag_get(DMA0, DMA_CH0, DMA_INT_FLAG_FTF)){ 
		pSend_data = (uint8_t*)(pSample_buffer+(sample_buffer_size/2));     
    }

	write_handle_raw(0x0074, pSend_data, sample_buffer_size);
	dma_interrupt_flag_clear(DMA0, DMA_CH0, DMA_INT_FLAG_G);
}


void emg_RMS_filter(uint16_t* input, uint16_t input_size){
	//ingest samples

	//remove bias - running average?

	//RMS
}