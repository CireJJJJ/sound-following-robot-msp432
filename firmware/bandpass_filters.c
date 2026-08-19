/* DriverLib Includes */
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>
#include "../inc/SysTick.h"
#include "../inc/Clock.h"


///////////// Global variables for CCS graph //////////////////

volatile uint32_t Size;
volatile uint32_t I;
volatile uint16_t c;

volatile int32_t INPUT_P6_1[1024];
volatile float Real_INPUT_P6_1[1024];

volatile int32_t INPUT_P6_0[1024];
volatile float Real_INPUT_P6_0[1024];

volatile float x[1024];        // raw input copied from P6.1
volatile float y[1024];        // high-pass filter output for D.1
volatile float z[1024];        // high-pass + low-pass output for D.2 extra credit

volatile float alpha;          // high-pass coefficient
volatile float alpha_lp;       // low-pass coefficient
volatile uint32_t filterCount; // debug counter, increases each time filter updates

/////////////////////////////////////////////////////////////

#define PERIOD   100

/* Timer_A UpMode Configuration Parameter */
const Timer_A_UpModeConfig upConfig =
{
    TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source 12 MHz
    TIMER_A_CLOCKSOURCE_DIVIDER_12,         // SMCLK/12 = 1 MHz Timer clock
    PERIOD,                                 // 100 timer clocks => 10 kHz sampling rate
    TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,     // Enable CCR0 interrupt
    TIMER_A_DO_CLEAR                        // Clear value
};


void TimerA2_Init(void);
void ADC_Ch14Ch15_Init(void);


//////////////////////// MAIN FUNCTION /////////////////////////////////////

int main(void)
{
    WDT_A_holdTimer();

    Size = 1000;
    I = Size - 1;
    filterCount = 0;

    // D.1 high-pass filter cutoff = 200 Hz
    // alpha = fs / (fs + 2*pi*fc)
    // fs = 10000 Hz, fc = 200 Hz
    alpha = 0.88836f;

    // D.2 optional low-pass filter cutoff = 3000 Hz
    // alpha_lp = 2*pi*fc / (2*pi*fc + fs)
    // fs = 10000 Hz, fc = 3000 Hz
    alpha_lp = 0.65338f;

    // Set microcontroller clock = 48 MHz
    Clock_Init48MHz();

    // Systick configuration
    SysTick_Init();

    // P6.4 is a debug output pin. It toggles during sampling.
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN4);

    // Setup ADC for P6.1 and P6.0
    ADC_Ch14Ch15_Init();

    // Timer A2 starts the sampling interrupt
    TimerA2_Init();

    while (1)
    {
        // Empty loop.
        // Sampling and filtering are done inside TA2_0_IRQHandler().
    }
}


//////////////////////// INTERRUPT HANDLER /////////////////////////////////////

void TA2_0_IRQHandler(void)
{
    GPIO_toggleOutputOnPin(GPIO_PORT_P6, GPIO_PIN4);

    ADC14_toggleConversionTrigger(); // ask ADC to get data

    while(ADC14_isBusy())
    {
    }

    // ADC_MEM0 = P6.1 / A14
    INPUT_P6_1[I] = ADC14_getResult(ADC_MEM0);
    Real_INPUT_P6_1[I] = (INPUT_P6_1[I] * 3.3f) / 16384.0f;

    // ADC_MEM1 = P6.0 / A15
    INPUT_P6_0[I] = ADC14_getResult(ADC_MEM1);
    Real_INPUT_P6_0[I] = (INPUT_P6_0[I] * 3.3f) / 16384.0f;

    if(I == 0)
    {
        I = Size - 1;

        // Copy P6.1 input into x[].
        // ADALM W1 should be connected to P6.1.
        for(c = 0; c < 1000; c++)
        {
            x[c] = Real_INPUT_P6_1[c];
        }

        // D.1: first-order high-pass filter, cutoff = 200 Hz
        // y[n] = alpha*y[n-1] + alpha*(x[n] - x[n-1])
        y[0] = 0.0f;

        for(c = 1; c < 1000; c++)
        {
            y[c] = alpha * y[c - 1] + alpha * (x[c] - x[c - 1]);
        }

        // D.2 optional: low-pass filter, cutoff = 3000 Hz
        // High-pass output y[] cascades into low-pass output z[].
        // z[n] = alpha_lp*y[n] + (1 - alpha_lp)*z[n-1]
        z[0] = y[0];

        for(c = 1; c < 1000; c++)
        {
            z[c] = alpha_lp * y[c] + (1.0f - alpha_lp) * z[c - 1];
        }

        filterCount++;
    }
    else
    {
        I--;
    }

    Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
}


//////////////////////// TIMER FUNCTION /////////////////////////////////////

void TimerA2_Init(void)
{
    /* Configuring Timer_A2 for Up Mode */
    Timer_A_configureUpMode(TIMER_A2_BASE, &upConfig);

    /* Enabling interrupts and starting the timer */
    Interrupt_enableInterrupt(INT_TA2_0);
    Interrupt_setPriority(INT_TA2_0, 0x20);

    Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);

    /* Enabling MASTER interrupts */
    Interrupt_enableMaster();
}


//////////////////////// ADC FUNCTION /////////////////////////////////////

void ADC_Ch14Ch15_Init(void)
{
    /* Initializing ADC */
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, 0);

    /* Configuring GPIOs for Analog Input */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN0 | GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);

    /* Configuring ADC Memory
     * ADC_MEM0 = A14 = P6.1
     * ADC_MEM1 = A15 = P6.0
     */
    ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, false);

    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A14, false);
    ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A15, false);

    /* Disable ADC interrupt because TimerA2 controls sampling */
    ADC14_disableInterrupt(ADC_INT1);
    Interrupt_disableInterrupt(INT_ADC14);

    /* Automatic sequence sampling */
    ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    /* Enable ADC conversion */
    ADC14_enableConversion();
}

/////////////////////////////////////// END /////////////////////////////////////////////
