/*
 * Project myProject
 * Author: Your Name
 * Date:
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);

// Show system, cloud connectivity, and application logs over USB
// View logs with CLI using 'particle serial monitor --follow'
SerialLogHandler logHandler(LOG_LEVEL_INFO);


#include "adc_hal.h"
#include "gpio_hal.h"
#include "pinmap_hal.h"
#include "pinmap_impl.h"

const size_t SAMPLE_BUF_SIZE = 1024 * 10;  // Size of the sample buffer
const int SAMPLE_PIN = A0; // The pin connected to ADC

const long SAMPLE_RATE = 32000; // sampling rate

// void buttonHandler(system_event_t event, int data); // forward declaration

const unsigned long MAX_RECORDING_LENGTH_MS = 10000; // Hit the setup button to stop recording. If it not hitted, record 50s

// IP Address and port that the server is running on.
//ipIPAddress serverAddr = IPAddress(172,20,10,7);
IPAddress serverAddr = IPAddress(192,168,254,7);
int serverPort = 8888;

// Each element of the array is an unsigned 8-bit integer. 
// This means each value in the array can range from 0 to 255.
uint16_t samples[SAMPLE_BUF_SIZE];

TCPClient client;
unsigned long recordingStart;

// enumeration, a user-defined type consisting of a set of named integral constants.
// a convenient way to group together a number of integer constants under a single name
enum State { STATE_WAITING, STATE_CONNECT, STATE_RUNNING, STATE_FINISH };
// enum State { STATE_WAITING, STATE_CONNECT, STATE_RUNNING, STATE_FINISH };
State state = STATE_WAITING;
unsigned long sampleCount = 0;


class ADCDMA {
	/*
	int pin: pin number where the ADC is connected
	uint8_t *buf: a pointer to an array where the ADC data will be stored.
	size_t bufSize：size of the buffer, indicating how many elements it can hold.
	*/
public:
	ADCDMA(int pin, uint16_t *buf, size_t bufSize);
	virtual ~ADCDMA();

	void start(size_t freqHZ); // start the ADC conversion. Specifies the frequency in Hertz at which the ADC should sample the analog signal
	void stop(); // Stops the ADC conversion
	
	private:
	int pin;
	uint16_t *buf;
	size_t bufSize;
};

// Constructor of the ADCDMA class. Constructor's name is always the same as the class name
ADCDMA::ADCDMA(int pin, uint16_t *buf, size_t bufSize) : pin(pin), buf(buf), bufSize(bufSize) {
}

ADCDMA::~ADCDMA() {
}

void ADCDMA::start(size_t freqHZ) {
    	// enables the clock for the DMA2 peripheral on the AHB1 bus
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
		// enables the clock for ADC1 on the APB2 bus. The clock needs to be enabled for the peripheral to operate.
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
		// enables the clock for ADC2 on the APB2 bus. 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
		// enables the clock for Timer 3 on the APB1 bus. 
		//The APB1 bus is designed for peripherals that do not require as high a clock frequency as those on APB2 or AHB.
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	// Set the pin as analog input
    HAL_Pin_Mode(pin, AN_INPUT);
    
    // enabling an interrupt that will be triggered when DMA2 Stream 0 completes a data transfer
    // defines the initialization and configuration of an interrupt in the NVIC
	NVIC_InitTypeDef NVIC_InitStructure;
	
	// identifies the specific interrupt channel being configured, in this case, 
	// the interrupt number for DMA controller 2's stream 0. The IRQn part usually stands for Interrupt Request Number
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	
	// Sets the preemption priority for the interrupt. 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	
	// Sets the NVIC_IRQChannelCmd to ENABLE, which indicates that the specified IRQ channel should be enabled.
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	// declares a variable TIM_TimeBaseStructure of type TIM_TimeBaseInitTypeDef, which is a structure that holds all the configuration settings for a timer.
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	// This function initializes the TIM_TimeBaseStructure with default settings. 
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	
	// This sets the period of the timer.
	TIM_TimeBaseStructure.TIM_Period = (60000000UL / freqHZ) - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	// Sets the trigger output source for the timer. Here, the trigger is set to occur on the timer's update event, which is when the timer overflows
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
	// Starts the timer TIM3 with the ENABLE command
	TIM_Cmd(TIM3, ENABLE);
	
	// Configure parameters for ADCs in the microcontroller
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC1_InitStructure;
	ADC_InitTypeDef ADC2_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	
	// DMA2 Stream0 channel0 configuration
	// https://www.disca.upv.es/aperles/arm_cortex_m3/curset/stm32f4xx_dsp_stdperiph_lib_v1.0.1/html/struct_d_m_a___init_type_def.html
	DMA_InitStructure.DMA_Channel = DMA_Channel_0; /* Specifies the channel used for the specified stream */
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)buf;
	DMA_InitStructure.DMA_PeripheralBaseAddr = 0x40012308; 
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = bufSize;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // 8-bit data samples, Word????
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; // 8-bit data samples, 16-bit per access
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; // Trigger a transfer when the FIFO is 1/2 full
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	
	// Enable the DMA Stream
	DMA_Cmd(DMA2_Stream0, ENABLE); 
	
	
	// ADC Common Init
	ADC_CommonInitStructure.ADC_Mode = ADC_DualMode_Interl;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	// DMA mode 3 is used in interleaved mode in 6-bit and 8-bit resolutions.
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_2;
	// Configures the Delay between 2 sampling phases.
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	// ADC1 configuration
	// This sets the resolution of the ADC to 8 bits
	ADC1_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	// The scan conversion mode is disabled. When enabled, the ADC would convert multiple channels in sequence. Disabling this means that only one ADC channel will be converted.
	ADC1_InitStructure.ADC_ScanConvMode = ENABLE;
	// This disables continuous conversion mode. When enabled, the ADC would continuously convert the input signal and provide a stream of digital data. Disabling it means that the ADC will perform a single conversion and stop.
	ADC1_InitStructure.ADC_ContinuousConvMode = DISABLE;//Use continuous mode for higher freq
	// This sets the ADC to start conversion at the rising edge of an external trigger
	ADC1_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;//ADC_ExternalTrigConvEdge_None;  //ADC_ExternalTrigConvEdge_Rising;
	// It specifies which external signal will trigger the ADC conversion.  In this case, it's set to start conversion on the trigger output (TRGO) of Timer 3
	ADC1_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;// ?????????
	ADC1_InitStructure.ADC_DataAlign = ADC_DataAlign_Left; 
	ADC1_InitStructure.ADC_NbrOfConversion = 1;
	
	
	
	// ADC2 configuration
	
	// This sets the resolution of the ADC to 8 bits
	ADC2_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	// The scan conversion mode is disabled. When enabled, the ADC would convert multiple channels in sequence. Disabling this means that only one ADC channel will be converted.
	ADC2_InitStructure.ADC_ScanConvMode = DISABLE;
	// This disables continuous conversion mode. When enabled, the ADC would continuously convert the input signal and provide a stream of digital data. Disabling it means that the ADC will perform a single conversion and stop.
	ADC2_InitStructure.ADC_ContinuousConvMode = DISABLE;//Use continuous mode for higher freq
	// This sets the ADC to start conversion at the rising edge of an external trigger
	ADC2_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;//ADC_ExternalTrigConvEdge_None;  //ADC_ExternalTrigConvEdge_Rising;
	// It specifies which external signal will trigger the ADC conversion.  In this case, it's set to start conversion on the trigger output (TRGO) of Timer 3
	ADC2_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;// ?????????
	ADC2_InitStructure.ADC_DataAlign = ADC_DataAlign_Left; 
	ADC2_InitStructure.ADC_NbrOfConversion = 1;
	
	// Initialize ADC1 and ADC2 with settings defined in the ADC_InitStructure
	ADC_Init(ADC1, &ADC1_InitStructure);
	ADC_Init(ADC2, &ADC2_InitStructure);
	
	// Configure the regular channels
	// Sample time equal to 3 cycles
	Hal_Pin_Info* PIN_MAP = HAL_Pin_Map();
	ADC_RegularChannelConfig(ADC1, PIN_MAP[pin].adc_channel, 1, ADC_SampleTime_15Cycles); 
	//ADC_RegularChannelConfig(ADC2, PIN_MAP[pin].adc_channel, 1, ADC_SampleTime_15Cycles); 

	
	// Enables the DMA request after the last transfer in multi-ADC mode. When multiple ADCs are operating in a combined mode (for example, regular simultaneous mode, interleaved mode, etc.), this setting ensures that the DMA request is only raised after the last ADC conversion in the sequence is complete
	ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

	// Enable ADCs
	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);
	
	// Enable ADC2 DMA since ADC1 is the master and will be triggered by the DMA
	//ADC_DMACmd(ADC1, ENABLE);
    //ADC_DMACmd(ADC2, ENABLE);

	// initiates the conversion process for ADC1. 
	ADC_SoftwareStartConv(ADC1);
}
	
	/*
stop the ADC and DMA operations that were previously started
*/
void ADCDMA::stop() {
	// Stop the ADC
	ADC_Cmd(ADC1, DISABLE);
	ADC_Cmd(ADC2, DISABLE);

	DMA_Cmd(DMA2_Stream0, DISABLE);

	// Stop the timer
	TIM_Cmd(TIM3, DISABLE);
}

ADCDMA adcDMA(SAMPLE_PIN, samples, SAMPLE_BUF_SIZE);
// End ADCDMA


// button handler for the SETUP button, used to toggle recording on and off
void buttonHandler(system_event_t event, int data) {
	switch(state) {
	case STATE_WAITING:
		state = STATE_CONNECT; // When the SETUP button is pressed in this state, the state changes to STATE_CONNECT
		break;

	case STATE_RUNNING:
		state = STATE_FINISH;
		break;
	}
}




void setup() {
    pinMode(A0, INPUT);
    Serial.begin(9600);
    // Register handler to handle clicking on the SETUP button
    System.on(button_click, buttonHandler);
    pinMode(D7, OUTPUT);
    
    
}

void loop() {
    
    uint16_t *sendBuf = NULL;
  
    switch(state) {
	  case STATE_WAITING:
		// Waiting for the user to press the SETUP button. 
		break;
    
    
    case STATE_CONNECT:
    		
        while (!client.connected()) {
        if (client.connect(serverAddr, serverPort)) {
            Serial.println("Connected to server");
            Particle.publish("Test", String("Connected to server"), PRIVATE);
            digitalWrite(D7, HIGH);
            
			// Connected
			adcDMA.start(SAMPLE_RATE);

			Serial.println("start sampling");
			//client.println("start sampling");
			Particle.publish("Test", String("start sampling"), PRIVATE);

			recordingStart = millis(); // Returns the number of milliseconds passed since begin

			state = STATE_RUNNING;
			break;
		}
		else {
            Serial.println("Lost connection to server");  // Print an error if the connection failed
            Particle.publish("Test", String("lost connection"), PRIVATE);
            digitalWrite(D7, LOW);
            state = STATE_WAITING;
        }
        delay(1000);  // Delay 1s before trying again
    }
		break;
    
    
    case STATE_RUNNING:
        Particle.publish("Test", String("state running"), PRIVATE);
        
        if (sendBuf == NULL) {
            digitalWrite(D7, HIGH); 
            delay(500);
            digitalWrite(D7, LOW); 
            delay(500);
        }
        
    // checks if the Half Transfer (HT) interrupt flag is set for DMA2 Stream 0.  Start processing the first half of the buffer while the second half is still being filled
        if (DMA_GetFlagStatus(DMA2_Stream0, DMA_FLAG_HTIF0)) {
            // Clears the half transfer interrupt flag for DMA2 Stream 0, so the system understands that you have acknowledged and handled the half transfer event.
		    DMA_ClearFlag(DMA2_Stream0, DMA_FLAG_HTIF0);
		    //  assign the address of array 'samples' to a pointer 'sendBuf'. The name of the array can be used to refer to the address of the first element of the array.
		    sendBuf = samples;
		    sampleCount += SAMPLE_BUF_SIZE / 2;
		}
		// checks if the DMA Transfer Complete (TC) interrupt flag is set. The flag indicates it has filled the entire buffer
		if (DMA_GetFlagStatus(DMA2_Stream0, DMA_FLAG_TCIF0)) {
		    // If the transfer complete flag was set, this line clears it. Clearing the flag is necessary because the DMA will not trigger another transfer complete interrupt until the flag has been cleared.
		    DMA_ClearFlag(DMA2_Stream0, DMA_FLAG_TCIF0);
		    // Point to the midpoint of the samples array
		    sendBuf = &samples[SAMPLE_BUF_SIZE / 2];
		    sampleCount += SAMPLE_BUF_SIZE / 2;
		}
		
		if (sendBuf != NULL) {
		    int count = client.write((uint8_t *)sendBuf, SAMPLE_BUF_SIZE); 
		    
		    if (count == SAMPLE_BUF_SIZE){
		        Serial.println("Data sent successfully");
		        client.print("Data sent successfully: ");
		        client.println(sampleCount);
		        Particle.publish("Test", String("Data sent successfully: " + String(sampleCount)), PRIVATE);
		        state = STATE_FINISH;
		        //Particle.publish("Test", String(sampleCount), PRIVATE);
		        
		    } else if (count == -16) {
		        // TCP Buffer full
				Serial.printlnf("buffer full, discarding");
				Particle.publish("Test", String("buffer full, discarding"), PRIVATE);
				state = STATE_FINISH;
		    } else{
		        // Error
				Serial.printlnf("error writing %d", count);
				Particle.publish("Test", String("error writing %d"), PRIVATE);
				state = STATE_FINISH;
		    }
		    
		    if (!client.connected()){
				state = STATE_FINISH; // Transition to finish state
			}
		    
		    if (millis() - recordingStart >= MAX_RECORDING_LENGTH_MS) {
                state = STATE_FINISH; // Transition to finish state
            }
		    
		}
		
		break;
		    
    case STATE_FINISH:
		digitalWrite(D7, LOW);
		adcDMA.stop();
		client.println("stopping");
		client.stop();
		Serial.println("stopping");
        Particle.publish("Test", String("stopping"), PRIVATE);
		state = STATE_WAITING;
		sampleCount = 0; 
	
		break;
	    
	}
}
