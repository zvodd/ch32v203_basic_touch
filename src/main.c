#include <ch32v20x.h>
#include <debug.h>

// LED Configuration
#define LED_GPIO_PORT GPIOA
#define LED_GPIO_PIN GPIO_Pin_15
#define LED_CLOCK_ENABLE RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE)

// TouchKey Configuration
#define TOUCH_PIN_ADC_CHANNEL ADC_Channel_0  // PA0 (ADC Channel 0)
#define TOUCH_THRESHOLD 100

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void Delay_Init(void);
void Delay_Ms(uint32_t n);

void LED_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    
    LED_CLOCK_ENABLE;
    GPIO_InitStructure.GPIO_Pin = LED_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);
    
    // LED off initially
    GPIO_WriteBit(LED_GPIO_PORT, LED_GPIO_PIN, Bit_RESET);
}

void TouchKey_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    ADC_InitTypeDef ADC_InitStructure = {0};
    
    // Enable clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);  // ADC clock = PCLK2/8
    
    // Configure PA0 as analog input for TouchKey
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // Configure ADC for TouchKey mode
    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    
    // Configure TouchKey channel
    ADC_RegularChannelConfig(ADC1, TOUCH_PIN_ADC_CHANNEL, 1, ADC_SampleTime_239Cycles5);
    
    // Enable ADC
    ADC_Cmd(ADC1, ENABLE);
    
    // ADC calibration
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    
    // Enable TouchKey mode - disable buffer for capacitive sensing
    ADC_BufferCmd(ADC1, DISABLE);
    
    printf("TouchKey initialized on PA0\r\n");
}

uint16_t TouchKey_Read(void) {
    // Start TouchKey conversion
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    
    // Wait for conversion to complete
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    
    // Read and return the TouchKey value
    return ADC_GetConversionValue(ADC1);
}

// The CH32V SDK provides built-in debug UART functionality
// Just need to call USART_Printf_Init() to enable it

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Delay_Init();
    
    // Initialize the SDK's built-in debug UART (PA9=TX @ 115200 baud)
    USART_Printf_Init(115200);
    
    // Small delay to let UART settle
    Delay_Ms(100);
    
    printf("\r\n=== CH32V203 TouchKey LED Control Demo ===\r\n");
    printf("Debug output via USART1 (PA9=TX, PA10=RX) @ 115200 baud\r\n");
    printf("Touch PA0 to toggle the LED on PA15\r\n\r\n");
    
    LED_Init();
    TouchKey_Init();
    
    uint16_t baseline = 0;
    uint16_t touch_value = 0;
    uint8_t led_state = 0;
    uint8_t touch_detected = 0;
    uint8_t last_touch_state = 0;
    
    // Calibrate baseline - take several readings when not touched
    printf("Calibrating baseline...\r\n");
    uint32_t baseline_sum = 0;
    for(int i = 0; i < 10; i++) {
        baseline_sum += TouchKey_Read();
        Delay_Ms(50);
    }
    baseline = baseline_sum / 10;
    printf("Baseline value: %d\r\n", baseline);
    printf("Touch threshold: %d\r\n", TOUCH_THRESHOLD);
    printf("Touch range: %d to %d\r\n", baseline - TOUCH_THRESHOLD, baseline + TOUCH_THRESHOLD);
    printf("Ready! Touch PA0 to control LED\r\n\r\n");
    
    uint32_t debug_counter = 0;
    
    while (1)
    {
        touch_value = TouchKey_Read();
        
        // Check if touch is detected (value significantly different from baseline)
        if(touch_value > baseline + TOUCH_THRESHOLD || touch_value < baseline - TOUCH_THRESHOLD) {
            touch_detected = 1;
        } else {
            touch_detected = 0;
        }
        
        // Toggle LED on touch press (not hold) - edge detection
        if(touch_detected && !last_touch_state) {
            led_state ^= 1;  // Toggle LED state
            GPIO_WriteBit(LED_GPIO_PORT, LED_GPIO_PIN, led_state);
            
            printf("TOUCH! Value: %d (diff: %d), LED: %s\r\n", 
                   touch_value, 
                   touch_value - baseline,
                   led_state ? "ON" : "OFF");
        }
        
        // Print debug info every 2 seconds if no touch
        if(debug_counter % 40 == 0 && !touch_detected) {
            printf("Current: %d, Baseline: %d, Diff: %d, LED: %s\r\n",
                   touch_value, baseline, touch_value - baseline, led_state ? "ON" : "OFF");
        }
        
        last_touch_state = touch_detected;
        debug_counter++;
        
        Delay_Ms(50);  // Small delay for debouncing and to avoid flooding serial
    }
}

void NMI_Handler(void) {}
void HardFault_Handler(void)
{
    while (1)
    {
    }
}