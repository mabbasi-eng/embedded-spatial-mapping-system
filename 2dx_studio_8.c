#include <stdint.h>
#include <stdio.h>
#include "tm4c1294ncpdt.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "VL53L1X_api.h"

#define I2C_MCR_MFE 0x00000010 //used to turn on I2C master mode

/* -------- Deliverable 2 settings -------- */
#define NUM_SCANS                   3 //perform ten scans
#define NUM_SAMPLES                 32 //each scan has 32 measurement points
#define ANGLE_STEP_HUNDREDTHS       1125    // 11.25 degrees (hundreths of a degree)
#define FAKE_SCAN_SPACING           10      // pretend scans are 10 cm apart

/* -------- Student-specific LED assignments --------
   Student number: 400589380
   H = 8  -> PF4 measurement, PF0 UART Tx, PN0 additional status
*/
#define PF0_LED         0x01 //bit 0 for PortF (UART LED)
#define PF4_LED         0x10 //bit 4 for PortF (measurement LED)
#define PN0_LED         0x01 //bit 0 for PortN (additional scan status LED)
#define PK0_PROOF 0x01  //bit 0 of Port K used to output bus-speed proof signal

/* -------- Stepper motor -------- */
#define STEPPER_MASK                0x0F //PH0 to PH3 for stepper
#define STEPS_PER_SHAFT_ROTATION    2048 //step states for one full shaft rotation
#define STEPS_PER_SAMPLE            (STEPS_PER_SHAFT_ROTATION / NUM_SAMPLES)   // 64 for 32 samples
#define STEPS_PER_360_DEG           2048 //used for unwinding wires

/* -------- Globals -------- */
uint16_t dev = 0x29; //The VL53L1X I2C address
int status = 0; //Stores return values from API calls

uint16_t distData[NUM_SCANS][NUM_SAMPLES]; //array of measured distances from the sensor
int angleData_hundredths[NUM_SCANS][NUM_SAMPLES]; //angle of each sample, in hundreths of a degree
int dispData[NUM_SCANS][NUM_SAMPLES]; //a fake displayment value for each scan layer (scan 1 -> displayment 10)

//each sample has scan layer, discplacement, angle and distance

const uint8_t fullStepSeq[4] = {0x03, 0x06, 0x0C, 0x09};
uint32_t stepIndex = 0;

/* ------------- PORT INIT ------------- */

void PortJ_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8; //enable clock to portJ
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0){} //wait until port ready
    GPIO_PORTJ_DIR_R &= ~0x03; //set PJ0 and PJ1 as inputs
    GPIO_PORTJ_AFSEL_R &= ~0x03; //disable alternate functions 
    GPIO_PORTJ_DEN_R |= 0x03; //enable digital function
    GPIO_PORTJ_AMSEL_R &= ~0x03; //disable analog
    GPIO_PORTJ_PUR_R |= 0x03; //enable pull-up resistors
}
void PortK_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R9; // enable clock for Port K
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R9) == 0){} // wait until Port K is ready

    GPIO_PORTK_DIR_R |= PK0_PROOF;  // set PK0 as output
    GPIO_PORTK_AFSEL_R &= ~PK0_PROOF; // disable alternate functions
    GPIO_PORTK_DEN_R |= PK0_PROOF; // enable digital function
    GPIO_PORTK_AMSEL_R &= ~PK0_PROOF; // disable analog function
}

void PortH_Init(void){ //PH0-PH3 as digital outputs for stepper motor
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0){}
    GPIO_PORTH_DIR_R |= 0x0F;
    GPIO_PORTH_AFSEL_R &= ~0x0F;
    GPIO_PORTH_DEN_R |= 0x0F;
    GPIO_PORTH_AMSEL_R &= ~0x0F;
    GPIO_PORTH_DATA_R &= ~0x0F; //clears output so the motor starts de-energized
}

void PortN_Init(void){ //PN0-PN1 as LED outputs
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R12) == 0){}
    GPIO_PORTN_DIR_R |= 0x03;
    GPIO_PORTN_AFSEL_R &= ~0x03;
    GPIO_PORTN_DEN_R |= 0x03;
    GPIO_PORTN_AMSEL_R &= ~0x03;
    GPIO_PORTN_DATA_R &= ~0x03;
}

void PortF_Init(void){ //PF0 and PF1 as LED outputs
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0){}
    GPIO_PORTF_DIR_R |= 0x11;
    GPIO_PORTF_AFSEL_R &= ~0x11;
    GPIO_PORTF_DEN_R |= 0x11;
    GPIO_PORTF_AMSEL_R &= ~0x11;
    GPIO_PORTF_DATA_R &= ~0x11;
}

void PortG_Init(void){ //PG0 as digital pin 
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R6) == 0){}
    GPIO_PORTG_DIR_R &= ~0x01;
    GPIO_PORTG_AFSEL_R &= ~0x01;
    GPIO_PORTG_DEN_R |= 0x01;
    GPIO_PORTG_AMSEL_R &= ~0x01;
}

void I2C_Init(void){
    SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0; //enable I2C clk
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1; //enable portB clock
    while((SYSCTL_PRGPIO_R & 0x0002) == 0){} //wait for PortB ready

    GPIO_PORTB_AFSEL_R |= 0x0C; //enable alternate functions
    GPIO_PORTB_ODR_R   |= 0x08; //open drain
    GPIO_PORTB_DEN_R   |= 0x0C; //enable digital
    GPIO_PORTB_PCTL_R  = (GPIO_PORTB_PCTL_R & 0xFFFF00FF) | 0x00002200; //selects I2C peripheral function on PB2/PB3
    GPIO_PORTB_AMSEL_R &= ~0x0C; //disable analog

    I2C0_MCR_R = I2C_MCR_MFE; //enables 12C master
    I2C0_MTPR_R = 4; //sets clock timing / bus speed
}

/* ------------- LED HELPERS ------------- */

void MeasurementLED_Blink(void){ //turn PF4 LED on for 10 ms, then off
    GPIO_PORTF_DATA_R |= PF4_LED;
    SysTick_Wait10ms(1);
    GPIO_PORTF_DATA_R &= ~PF4_LED;
}

void UARTLED_On(void){ //lights an LED during UART transmission
    GPIO_PORTF_DATA_R |= PF0_LED;
}

void UARTLED_Off(void){ 
    GPIO_PORTF_DATA_R &= ~PF0_LED;
}

void ScanLED_On(void){ //shows scanning is active
    GPIO_PORTN_DATA_R |= PN0_LED;
}

void ScanLED_Off(void){
    GPIO_PORTN_DATA_R &= ~PN0_LED;
}

/* ------------- BUTTON HELPERS ------------- */

uint8_t StartStopButtonPressed(void){ //checks PJ0 pull-up enabled(notpressed = 1 and pressed = 0) 
    return ((GPIO_PORTJ_DATA_R & 0x01) == 0); //returns true when bit is 0
}

void WaitForButtonRelease(void){ //used for debouncing; wait until button no longer pressed then delay 20ms 
    while(StartStopButtonPressed()){}
    SysTick_Wait10ms(2);
}

/* ------------- SENSOR RESET ------------- */

//driving xshut low resets the sensor, then releasing it allows sensor to boot again
//xshut is pulled up internally, releasing line by changing to input is good
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R  |= 0x01; //make PG0 output
    GPIO_PORTG_DATA_R &= ~0x01; //drive it low
    SysTick_Wait10ms(10); //wait 100ms
    GPIO_PORTG_DIR_R  &= ~0x01; //make it input again
}

/* ------------- STEPPER MOTOR ------------- */

void StepMotorCW_OneState(void){ //Outputs the current coil pattern, then advances to the next pattern
    GPIO_PORTH_DATA_R = (GPIO_PORTH_DATA_R & ~STEPPER_MASK) | fullStepSeq[stepIndex]; //clears PH0-PH3
    stepIndex = (stepIndex + 1) % 4; //writes current stepper state then increments so motor steps one state clockwise
}

void StepMotorCCW_OneState(void){ //decrement first then output state for CCW
    if(stepIndex == 0){
        stepIndex = 3;
    } else {
        stepIndex--;
    }
    GPIO_PORTH_DATA_R = (GPIO_PORTH_DATA_R & ~STEPPER_MASK) | fullStepSeq[stepIndex];
}

uint8_t RotateStepsCW_CheckAbort(int steps){ //Rotate clockwise by a specified number of step states.
    int i;
    for(i = 0; i < steps; i++){ //checks for button press, if pressed; abort and return 1
        if(StartStopButtonPressed()){
            WaitForButtonRelease();
            GPIO_PORTH_DATA_R = 0x00;
            return 1;
        }
        StepMotorCW_OneState(); //otherwise performs one step
        SysTick_Wait10ms(1);
    }
    GPIO_PORTH_DATA_R = 0x00;
    SysTick_Wait10ms(20);
    return 0;
}

uint8_t RotateStepsCCW_CheckAbort(int steps){
    int i;
    for(i = 0; i < steps; i++){
        if(StartStopButtonPressed()){
            WaitForButtonRelease();
            GPIO_PORTH_DATA_R = 0x00;
            return 1;
        }
        StepMotorCCW_OneState();
        SysTick_Wait10ms(1);
    }
    GPIO_PORTH_DATA_R = 0x00;
    SysTick_Wait10ms(20);
    return 0;
}

uint8_t RotateOneSampleStep_CheckAbort(void){ //moves motor exactly by one sample angle 64 motor steps corresponding to 11.25*
    return RotateStepsCW_CheckAbort(STEPS_PER_SAMPLE);
}

uint8_t Unwind360Deg_CheckAbort(void){ //rotates one full turn in reverse
    return RotateStepsCCW_CheckAbort(STEPS_PER_360_DEG);
}

/* ------------- TOF ------------- */

uint16_t GetOneDistance(void){ //This function waits for the sensor to have a fresh reading, then retrieves it.
    uint8_t dataReady = 0; //sensor is ready flag
    uint16_t distance = 0; //measured result

    while(dataReady == 0){ //checks for abort button
        if(StartStopButtonPressed()){
            WaitForButtonRelease();
            return 0xFFFF;
        }

        status = VL53L1X_CheckForDataReady(dev, &dataReady); //asks sensor if data is ready
        if(status != 0){
            UART_printf("ERROR: CheckForDataReady failed\r\n");
            return 0;
        }
        SysTick_Wait10ms(1); //delays 10ms
    }

    status = VL53L1X_GetDistance(dev, &distance); //on sensor ready; calls get disctance
    if(status != 0){
        UART_printf("ERROR: GetDistance failed\r\n");
        return 0;
    }

    status = VL53L1X_ClearInterrupt(dev);
    if(status != 0){
        UART_printf("ERROR: ClearInterrupt failed\r\n");
    }

    return distance;
}

/* ------------- UART SEND ------------- */

void SendOneScanData(int scan){ //sends one scan's worth of data over UART; DATA, scan index, displacement, angle, distance
    int sample;

    UARTLED_On();

    for(sample = 0; sample < NUM_SAMPLES; sample++){
        sprintf(printf_buffer, "DATA,%d,%d,%d,%u\r\n",
                scan,
                dispData[scan][sample],
                angleData_hundredths[scan][sample],
                distData[scan][sample]);
        UART_printf(printf_buffer);
    }

    sprintf(printf_buffer, "ENDSCAN,%d\r\n", scan);
    UART_printf(printf_buffer);

    UARTLED_Off();
    SysTick_Wait10ms(2);
}

void SendDone(void){ //marker meaning all scans are finished	
    UART_printf("DONE\r\n");
}

void BusSpeedProofMode(void){
    while(1){
        GPIO_PORTK_DATA_R ^= PK0_PROOF;
        SysTick_Wait(100000);
    }
}


/* ------------- MAIN ------------- */

int main(void){
    uint8_t sensorState = 0; //tracks whether sensor is booted
    int scan, sample; // loop indexes
    int angle_hundredths; //current angles
    uint16_t d; //one distance measurement
    uint8_t aborted; //flag for user cancellation

    PLL_Init(); //clk setup
    SysTick_Init(); //delay timer setup
    UART_Init();

    PortJ_Init();
		PortK_Init();
    PortH_Init();
    PortN_Init();
    PortF_Init();
    PortG_Init();
    I2C_Init();
	
	  // Uncomment this line only when testing bus speed with the AD3
		//BusSpeedProofMode();

    UART_printf("System booted\r\n");

    VL53L1X_XSHUT(); //hardware resets the ToF sensor

    while(sensorState == 0){ //repeatedly checks if it has finished booting
        status = VL53L1X_BootState(dev, &sensorState);
        SysTick_Wait10ms(1);
    }

    status = VL53L1X_SensorInit(dev);
    if(status != 0){
        UART_printf("ERROR: SensorInit failed\r\n");
        while(1){}
    }

    UART_printf("ToF ready\r\n");

    while(1){
        if(StartStopButtonPressed()){
            WaitForButtonRelease();

            status = VL53L1X_StartRanging(dev); 
            if(status != 0){
                UART_printf("ERROR: StartRanging failed\r\n");
                continue;
            }

            ScanLED_On(); //indicate scanning is active
            aborted = 0;

            for(scan = 0; scan < NUM_SCANS && !aborted; scan++){ //performs scans
                angle_hundredths = 0; //set angle back to 0

                for(sample = 0; sample < NUM_SAMPLES; sample++){ //for each sample check if user aborted
                    if(StartStopButtonPressed()){
                        WaitForButtonRelease();
                        aborted = 1;
                        break;
                    }

                    d = GetOneDistance(); //get one distance reading
                    if(d == 0xFFFF){
                        aborted = 1;
                        break;
                    }

                    angleData_hundredths[scan][sample] = angle_hundredths; //storing the data
                    distData[scan][sample]             = d;
                    dispData[scan][sample]             = scan * FAKE_SCAN_SPACING;

                    MeasurementLED_Blink(); //Visual confirmation that one measurement happened

                    if(sample < (NUM_SAMPLES - 1)){ //rotate motor by one anglular increment + update angle
                        if(RotateOneSampleStep_CheckAbort()){
                            aborted = 1;
                            break;
                        }
                        angle_hundredths += ANGLE_STEP_HUNDREDTHS;
                    }
                }

                /* Return to home position after each scan */
                if(!aborted){
                    if(RotateOneSampleStep_CheckAbort()){
                        aborted = 1;
                    }
                }

                /* Send this scan immediately after it is completed */
                if(!aborted){
                    SendOneScanData(scan);
                }

                /* Unwind wires by rotating 360 degrees in opposite direction */
                if(!aborted){
                    if(Unwind360Deg_CheckAbort()){
                        aborted = 1;
                    }
                }
            }

            status = VL53L1X_StopRanging(dev);
            if(status != 0){
                UART_printf("ERROR: StopRanging failed\r\n");
            }

            ScanLED_Off();
            GPIO_PORTH_DATA_R = 0x00;

            if(aborted){
                UART_printf("ABORTED\r\n");
            } else {
                SendDone();
            }
        }
    }
}