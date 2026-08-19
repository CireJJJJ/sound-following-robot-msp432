/* DriverLib Includes */
#include "driverlib.h"
/* Standard Includes */
#include <stdint.h>
#include "../inc/SysTick.h"
#include "../inc/Clock.h"


/////////////You will define variables here//////////////////

uint32_t Size;
uint32_t I;

int32_t INPUT_P6_1[1024];
float Real_INPUT_P6_1[1024];

int32_t INPUT_P6_0[1024];
float Real_INPUT_P6_0[1024];

float HP_OUTPUT[1024];          // high-pass filtered (DC-removed) signal

uint8_t DIRECTION;

#define FORWARD    1
#define BACKWARD   2
#define LEFT       3
#define RIGHT      4

uint8_t MODE;

#define SAMPLING_MODE    1
#define RUNNING_MODE     2

/* -----------------------------------------------------------------------
 *  SOUND DETECTION SETTINGS
 *
 *  Which ADC array holds your microphone signal?  Graph BOTH Real_INPUT_P6_0
 *  and Real_INPUT_P6_1 in CCS while making a sound; the one that wiggles is
 *  your mic pin. Set SOUND_ARRAY to that one. (Part D wires audio to P6.1,
 *  so if it never goes FORWARD, switch this to Real_INPUT_P6_1.)
 * --------------------------------------------------------------------- */
#define SOUND_ARRAY   Real_INPUT_P6_0

/* High-pass filter coefficient (removes the DC offset so we measure the
 * real audio swing, not the ~1.67 V bias).
 *   y[n] = ALPHA * ( y[n-1] + x[n] - x[n-1] )
 *   ALPHA = fs / (fs + 2*pi*fc)
 *   fs = 5 kHz (Timer A2 rate), fc ~ 100 Hz  ->  5000/(5000+628) = 0.888  */
#define ALPHA   0.888f

/* Amplitude (in volts) that counts as "a sound is present".
 * This is now a SWING amplitude, not an absolute voltage. Start at 0.15 V,
 * then watch avgmax in CCS: it should be near 0 when quiet and clearly
 * larger when you make a sound. Put the threshold between those two. */
#define SOUND_THRESHOLD   0.15f

/////////////////////////////////////////////////////////////

#define PERIOD   100

/* Timer_A UpMode Configuration Parameter */
const Timer_A_UpModeConfig upConfig =
{
    TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source 12 MHz
    TIMER_A_CLOCKSOURCE_DIVIDER_24,         // SMCLK/24 = 500 kHz Timer clock
    PERIOD,                                 // a period of 100 timer clocks => 5 KHz Frequency
    TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
    TIMER_A_DO_CLEAR                        // Clear value
};


/* Application Defines */
#define TIMER_PERIOD 15000  // 10 ms PWM Period
#define DUTY_CYCLE1 0
#define DUTY_CYCLE2 0



/* Timer_A UpDown Configuration Parameter */
Timer_A_UpDownModeConfig upDownConfig =
{
    TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock SOurce
    TIMER_A_CLOCKSOURCE_DIVIDER_4,          // SMCLK/1 = 1.5MHz
    TIMER_PERIOD,                           // 15000 period
    TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
    TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,    // Disable CCR0 interrupt
    TIMER_A_DO_CLEAR                        // Clear value

};

/* Timer_A Compare Configuration Parameter  (PWM1) */
Timer_A_CompareModeConfig compareConfig_PWM1 =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_1,          // Use CCR1 for P2.4 right motor PWM
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
        TIMER_A_OUTPUTMODE_TOGGLE_RESET,            // Toggle output
        DUTY_CYCLE1
};

/* Timer_A Compare Configuration Parameter (PWM2) */
Timer_A_CompareModeConfig compareConfig_PWM2 =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_2,          // Use CCR2 for P2.5 left motor PWM
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
        TIMER_A_OUTPUTMODE_TOGGLE_RESET,            // Toggle output
        DUTY_CYCLE2
};


/////////////////////////////////////////////////////////////


void TimerA2_Init(void);
void PWM_Init12(void);
void PWM_duty1(uint16_t duty1, Timer_A_CompareModeConfig* data);
void PWM_duty2(uint16_t duty1, Timer_A_CompareModeConfig* data);
void MotorInit(void);
void motor_forward(uint16_t leftDuty, uint16_t rightDuty);
void motor_right(uint16_t leftDuty, uint16_t rightDuty);
void motor_left(uint16_t leftDuty, uint16_t rightDuty);
void motor_backward(uint16_t leftDuty, uint16_t rightDuty);
void motor_stop(void);
void ADC_Ch14Ch15_Init(void);


//////////////////////// MAIN FUNCTION /////////////////////////////////////

int main(void)
{
    Size=1000;
    I=Size-1;

    // Set Microcontroller Clock = 48 MHz
    Clock_Init48MHz();

    PWM_Init12();

    // Systick Configuration
    SysTick_Init();

    // Motor Configuration
    MotorInit();

    /* Sleeping when not in use */

    // Port 6 Configuration: make P6.4 out
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN4);

    // Setup ADC for Channel A6 and A7
    ADC_Ch14Ch15_Init();

    // Timer A2 Configuration
    TimerA2_Init();

    DIRECTION  = BACKWARD;
    MODE  = SAMPLING_MODE;

    while (1)
    {

    }

}


//////////////////////// FUNCTIONs /////////////////////////////////////


void TA2_0_IRQHandler(void)
{
    GPIO_toggleOutputOnPin(GPIO_PORT_P6, GPIO_PIN4);

    // IN SAMPLING MODE
    if(MODE == SAMPLING_MODE)
    {
        motor_stop();

        ADC14_toggleConversionTrigger(); // ask ADC to get data

        while(ADC14_isBusy()){};

        INPUT_P6_0[I] = ADC14_getResult(ADC_MEM1);
        Real_INPUT_P6_0[I] = (INPUT_P6_0[I] * 3.3) / 16384;

        INPUT_P6_1[I] = ADC14_getResult(ADC_MEM0);
        Real_INPUT_P6_1[I] = (INPUT_P6_1[I] * 3.3) / 16384;


        if(I == 0)
        {
            I = Size-1;
            MODE = RUNNING_MODE;


//////////// MAKE DIRECTION DECISION BASED SAMPLING RESULTS HERE ///////////////////////
            // The microphone signal sits on a DC bias (~1.67 V). Comparing the
            // raw voltage to an absolute threshold is fragile, so instead we:
            //   1) high-pass filter the samples to remove the DC offset
            //   2) measure the average peak SWING (amplitude) of the result
            //   3) if that swing is large enough, a sound was made -> FORWARD
            //      otherwise it is quiet -> BACKWARD

            uint32_t k;
            uint32_t segment;
            uint32_t j;
            uint32_t start;
            float    avgmax;
            float    maxValue;
            float    mag;

            // ---- High-pass filter: alpha + initialization step ----
            HP_OUTPUT[0] = 0.0f;                       // initialization
            for(k = 1; k < Size; k++)
            {
                HP_OUTPUT[k] = ALPHA * (HP_OUTPUT[k-1]
                                        + SOUND_ARRAY[k]
                                        - SOUND_ARRAY[k-1]);
            }

            // ---- Average of the peak magnitude over 10 segments ----
            avgmax = 0.0f;
            for(segment = 0; segment < 10; segment++)
            {
                maxValue = 0.0f;
                start = 100 * segment;

                for(j = start; j < start + 100; j++)
                {
                    mag = HP_OUTPUT[j];
                    if(mag < 0.0f) mag = -mag;          // take |.|  (signal is bipolar now)
                    if(mag > maxValue) maxValue = mag;
                }

                avgmax = avgmax + maxValue;
            }
            avgmax = avgmax / 10.0f;

            // ---- Decision ----
            if(avgmax > SOUND_THRESHOLD)
            {
                DIRECTION = FORWARD;    // sound detected
            }
            else
            {
                DIRECTION = BACKWARD;   // quiet
            }

////////////////////////////////////////////////////////////////////////////////////

        }

        else
        {
            I--;
        }
    }


    // IN RUNNING MODE
    if(MODE == RUNNING_MODE)
    {
        uint16_t turn_speed = 6000;
        uint16_t turn_speed_slow = 3000;

        if(DIRECTION  == FORWARD)
        {
            motor_forward(turn_speed, turn_speed); // Move forward when sound is detected
            SysTick_Wait10ms(300); // Wait 3s for the motor to run
        }

        else if (DIRECTION  == BACKWARD)
        {
            motor_backward(turn_speed, turn_speed); // Move backward when no sound is detected
            SysTick_Wait10ms(300); // Wait 3s for the motor to run
        }

        else if (DIRECTION  == LEFT)
        {
            motor_forward(turn_speed_slow, turn_speed); // Move forward left
            SysTick_Wait10ms(300); // Wait 3s for the motor to run
        }

        else if (DIRECTION  == RIGHT)
        {
            motor_forward(turn_speed, turn_speed_slow); // Move forward right
            SysTick_Wait10ms(300); // Wait 3s for the motor to run
        }

        MODE = SAMPLING_MODE;
    }


    Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
}


////////////////////////////////////////////////////////////////////////////////////


void TimerA2_Init(void){
    /* Configuring Timer_A1 for Up Mode */
    Timer_A_configureUpMode(TIMER_A2_BASE, &upConfig);

    /* Enabling interrupts and starting the timer */
    Interrupt_enableSleepOnIsrExit();
    Interrupt_enableInterrupt(INT_TA2_0);
    Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);

    /* Enabling MASTER interrupts */
    Interrupt_setPriority(INT_TA2_0, 0x20);
    Interrupt_enableMaster();

}


void PWM_Init12(void){

        /* Setting P2.4 and P2.5 and peripheral outputs for CCR */
        // P2.4 = right motor PWM
        // P2.5 = left motor PWM
        GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN4 + GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);

        /* Configuring Timer_A1 for UpDown Mode and starting */
        Timer_A_configureUpDownMode(TIMER_A0_BASE, &upDownConfig);
        Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UPDOWN_MODE);

        /* Initialize compare registers to generate PWM1 */
        Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM1);

        /* Initialize compare registers to generate PWM2 */
        Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM2);
}


void PWM_duty1(uint16_t duty1, Timer_A_CompareModeConfig* data)  // function definition
{
  if(duty1 >= TIMER_PERIOD) return; // bad input
  data->compareValue = duty1; // access a struct member through a pointer using the -> operator
  /* Initialize compare registers to generate PWM1 */
  Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM1);
}

void PWM_duty2(uint16_t duty2, Timer_A_CompareModeConfig* data)  // function definition
{
  if(duty2 >= TIMER_PERIOD) return; // bad input
  data->compareValue = duty2; // access a struct member through a pointer using the -> operator
  /* Initialize compare registers to generate PWM2 */
  Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM2);
}




void MotorInit(void){

    // Right motor:
    // P3.7 = direction
    // P3.6 = sleep
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN7|GPIO_PIN6);

    // Left motor:
    // P3.5 = direction
    // P3.0 = sleep
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN5|GPIO_PIN0);

    // Wake up both motor drivers
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6|GPIO_PIN0);
}


void motor_forward(uint16_t leftDuty, uint16_t rightDuty){

    // Forward:
    // Right direction P3.7 = LOW
    // Left direction P3.5 = LOW
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7|GPIO_PIN5);

    // Wake up both motor drivers
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6|GPIO_PIN0);

    // PWM1 = right motor P2.4
    // PWM2 = left motor P2.5
    PWM_duty1(rightDuty, &compareConfig_PWM1);
    PWM_duty2(leftDuty,  &compareConfig_PWM2);

}



// ------------Motor_Right------------
// Turn the robot to the right by running the
// left wheel forward and the right wheel
// backward with the given duty cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void motor_right(uint16_t leftDuty, uint16_t rightDuty){
    // write this as part of Lab 7

    // Right wheel backward: P3.7 = HIGH
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);

    // Left wheel forward: P3.5 = LOW
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5);

    // Wake up both motor drivers
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6|GPIO_PIN0);

    // Run both wheels for pivot turn
    PWM_duty1(rightDuty, &compareConfig_PWM1);
    PWM_duty2(leftDuty,  &compareConfig_PWM2);
}

// ------------Motor_Left------------
// Turn the robot to the left by running the
// left wheel backward and the right wheel
// forward with the given duty cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void motor_left(uint16_t leftDuty, uint16_t rightDuty){
    // write this as part of Lab 7

    // Right wheel forward: P3.7 = LOW
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);

    // Left wheel backward: P3.5 = HIGH
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5);

    // Wake up both motor drivers
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6|GPIO_PIN0);

    // Run both wheels for pivot turn
    PWM_duty1(rightDuty, &compareConfig_PWM1);
    PWM_duty2(leftDuty,  &compareConfig_PWM2);
}

// ------------Motor_Backward------------
// Drive the robot backward by running left and
// right wheels backward with the given duty
// cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void motor_backward(uint16_t leftDuty, uint16_t rightDuty){
    // write this as part of Lab 7

    // Backward:
    // Right direction P3.7 = HIGH
    // Left direction P3.5 = HIGH
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7|GPIO_PIN5);

    // Wake up both motor drivers
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6|GPIO_PIN0);

    // PWM1 = right motor P2.4
    // PWM2 = left motor P2.5
    PWM_duty1(rightDuty, &compareConfig_PWM1);
    PWM_duty2(leftDuty,  &compareConfig_PWM2);
}


void motor_stop(void){

    PWM_duty1(0, &compareConfig_PWM1);
    PWM_duty2(0, &compareConfig_PWM2);

}


void ADC_Ch14Ch15_Init(void){

    /* Initializing ADC (MCLK/1/1) */
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1,0);

    /* Configuring GPIOs for Analog In */

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6,GPIO_PIN0 | GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);


    /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM1 (A14 - A15) */
    ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, false);
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A14, false);
    ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A15, false);

    /* Enabling the interrupt when a conversion on channel 1 (end of sequence)
    *  is complete and enabling conversions */
    ADC14_disableInterrupt(ADC_INT1);

    /* Enabling Interrupts */
    Interrupt_disableInterrupt(INT_ADC14);
    //  Interrupt_enableMaster();

    /* Setting up the sample timer to automatically step through the sequence
    * convert.
    */
    ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    /* Triggering the start of the sample */
    ADC14_enableConversion();

}

///////////////////////////////////////END/////////////////////////////////////////////
