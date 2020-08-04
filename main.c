/*
 *  ======= main ========
 *
 *  Created on: 20.11.2017
 *  Author:     Roman Grewenig M.Sc.
 *  Company:	Hochschule Trier
 *
 *  Target: TM4C123GH6PM
 *  JTAG Chain: Stellaris In-Circuit Debug Interface (on-board)
 *
 *  Compiler: TI-ARM v17.9
 *  IDE: CCS v6.2.0.00050
 *
 *  Description:
 *  ------------
 *  Minimal example for the EK-TM4C123GXL Launch Pad.
 *  Use as a starting point for your own projects.
 *  This example blinks all three colours of the RGB LED using a state machine and a
 *  cyclic task. The current state is transmitted every time step (CAN-ID 0x1).
 *
 *  Important note: Check Project > Properties > Resource > Linked Resources for relative
 *  file paths.
 *
 *  Needs TI-RTOS (an the included TivaWare) to be installed from:
 *  http://software-dl.ti.com/dsps/dsps_public_sw/sdo_sb/targetcontent/tirtos/2_16_01_14/exports/tirtos_tivac_setupwin32_2_16_01_14.exe
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>

/* Standard IO stream etc. */
#include <stdbool.h>			// Defines for standard boolean representation.
#include <stdio.h>				// IO-stream for console display etc.

/* Drivers etc. */
#include "inc/hw_ints.h"		// Macros defining the interrupts.
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"		// Macros defining the memory map of the device.
#include "inc/hw_sysctl.h"		// Macros for usage with SysCtl.
#include "inc/hw_types.h"		// Macros for common types.
#include "inc/hw_can.h"			// Macros for usage with the CAN controller peripheral.
#include "driverlib/debug.h"	// Macros to debug the driverlib.
#include "driverlib/gpio.h"		// Macros defining the GPIO assignments.
#include "driverlib/pin_map.h"	// Macros defining the pin map of the device.
#include "driverlib/sysctl.h"	// Prototypes for the SysCtl driver.
#include "driverlib/can.h"		// Defines and macros for the CAN controller.
#include "driverlib/qei.h"      // for quadratur encoder
#include "driverlib/interrupt.h"//for use of interrupt
#include "driverlib/pwm.h"      //use for pwm Output

/* Global variables */
// Put your global variables here.
tCANMsgObject MsgObjectTx;
tCANMsgObject sMsgObjectDataRx0;

uint8_t pui8TxBuffer[8];
uint8_t pui8RxBuffer[8];
uint8_t TestVariable;
uint32_t Position;
bool test = true;

/* Function prototypes */
void Init_Clock(void);
//void Init_GPIO(void);
void Init_CAN(void);
void ConfigureQEI0(void);
void Init_PWM(void);
// initialisiere Winkelencoder
void Init_Winkel(void);


void CAN0IntHandler(void)
{
    static uint32_t ui32Status = 0;

    /*gibt das Message Objekt (0-31) raus, welches eine Nachricht enthält*/
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);
    /* Switch for message object */
    switch(ui32Status)
    {
        /* TX handler */
        case 1:     CANIntClear(CAN0_BASE, 1);
                    break;

        /* RX handler */
        case 2:     CANIntClear(CAN0_BASE, 2);
                   // ui32CanRxFlags = (ui32CanRxFlags | 0b10);
                    break;

        case 3:     CANIntClear(CAN0_BASE, 3);
                    sMsgObjectDataRx0.pui8MsgData = pui8RxBuffer;
                    CANMessageGet(CAN0_BASE, 3, &sMsgObjectDataRx0, 0);
                    TestVariable = pui8RxBuffer[0];
                    if(TestVariable == 200)
                    {
                        pui8TxBuffer[0] = 55;
                        CANMessageSet(CAN0_BASE, 1, &MsgObjectTx, MSG_OBJ_TYPE_TX);
                    }
                    break;

        /*pending status error */
        case 32768: CANIntClear(CAN0_BASE, CAN_INT_INTID_STATUS);
                  //  printf("Status Error \n");
                    break;

        /* Unused message objects: These should never trigger */
        default:    break;
    }
    return;
}
/*
 * ======== main ========
 */
void main(void)
{
    printf("BIOS starting...\n");

    // Initialize the main clock/oscillator.
    Init_Clock();

    // Initialize your peripherals (GPIOs etc.).
    //Init_GPIO();

    // Initialize the CAN peripheral.
    Init_CAN();

    //QEI Module aktivieren
    ConfigureQEI0();

    //PWM Module aktivieren
    Init_PWM();

    //Winkel Enoceder aktivieren
    Init_Winkel();

    printf("Finished.\n");

    // Start TI-RTOS. Everything is now controlled by the BIOS, as defined in your project .CFG file.
    BIOS_start();

}

/*
 * ======== tasks ========
 */

// This is the background task.
void Task_Idle(void)
{
	// Code here ...
    System_flush();
}

// This is a repetitive task that is executed every 100 ms.
// Timing can be modified in the .CFG file.
void Task_100ms(void)
{
	/*static uint8_t ui8State = 0;

	// Handle the LED blinking state machine.
	switch(ui8State)
	{
		// Turn on the red LED.
		case 0:	GPIOPinWrite(GPIO_PORTF_BASE, (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3), GPIO_PIN_1);
				ui8State++;
				break;
		// Turn on the blue LED.
		case 1:	GPIOPinWrite(GPIO_PORTF_BASE, (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3), GPIO_PIN_2);
				ui8State++;
				break;
		// Turn on the green LED.
		case 2:	GPIOPinWrite(GPIO_PORTF_BASE, (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3), GPIO_PIN_3);
				ui8State = 0;
				break;
		// Spurious states go here.
		default:printf("State machine error.\n"); // This should never happen.
				break;
	}*/

    // Read the encoder position.
    Position = QEIPositionGet(QEI1_BASE);
    // display values in debug window
    System_printf("Position: %d\n", Position);


	// Send a CAN message with the current state placed in the least significant byte.
    //CANMessageSet(CAN0_BASE, 2, &MsgObjectRx, MSG_OBJ_TYPE_RX);


	pui8TxBuffer[0] = 200;
	// pui8TxBuffer[1] = Position >> 8;
	CANMessageSet(CAN0_BASE, 1, &MsgObjectTx, MSG_OBJ_TYPE_TX);
	//CANMessageGet(CAN0_BASE, 2, MsgObjectRx, MSG_OBJ_TYPE_RX);

}

/*
 * ======== user functions ========
 */

// This function initializes the clock/oscillator used in the project.
void Init_Clock(void)
{
	// Settings for 80 MHz.
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	return;
}

// This function initializes the GPIO resources used in the project.
// Modify to your demands.
/*
void Init_GPIO(void)
{
	printf("Initializing GPIOs...");
	// Enable the peripheral.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
		// Wait until it is ready...
		while (! SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
	// Set the GPIO directions for the respective pins.
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3));
	printf("done.\n");
	return;
}
*/

// This function initializes the CAN controller resources used in the project.
// Modify to your demands.
void Init_CAN(void)
{
	printf("Initializing CAN...");
	// Enable the peripheral.
	SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOB);
		// Wait until it is ready...
		while (! SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
	// Set the GPIO special functions (CANTX, CANRX) for the respective pins.
	GPIOPinConfigure(GPIO_PB5_CAN0TX);
	GPIOPinConfigure(GPIO_PB4_CAN0RX);
	GPIOPinTypeCAN (GPIO_PORTB_BASE , GPIO_PIN_4 | GPIO_PIN_5);
	// Enable the CAN peripheral CAN0.
	SysCtlPeripheralEnable (SYSCTL_PERIPH_CAN0);
	// Reset the CAN controller.
	CANInit(CAN0_BASE);
	// Set the baud rate to 250 kBaud/s based on the system clocking. Modify if needed.
	CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 250000);
	// Disable auto-retry if no ACK-bit is received by the CAN controlled.
	CANRetrySet(CAN0_BASE, 0);
	// More sophisticated CAN communication requires CAN interrupt handling.
	// For CAN0, the interrupt vector number is 55 (Macro: INT_CAN0_TM4C123).
	// Use the BIOS configuration file to set up a HWI for CAN0, with a suitable interrupt handler.
	// Uncomment the following lines to enable CAN interrupts.
	//CANIntEnable (CAN0_BASE , CAN_INT_MASTER);
	//IntEnable (INT_CAN0);
	CANEnable (CAN0_BASE);

	// Initialize all required message objects.
	MsgObjectTx.ui32MsgID = 0x0013;			// Message ID is '1'.
	MsgObjectTx.ui32Flags = 0x0000;			// No flags are used on this message object.
	MsgObjectTx.ui32MsgIDMask = 0x0000;		// No masking is used for TX.
	MsgObjectTx.ui32MsgLen = 8;			    // Set the DLC to '8' (8 bytes of data)
	MsgObjectTx.pui8MsgData = pui8TxBuffer; // A buffer, to which this message object points for data storage.
	CANMessageSet(CAN0_BASE, 1, &MsgObjectTx, MSG_OBJ_TYPE_TX);

	sMsgObjectDataRx0.ui32MsgID = 0x0020;
	sMsgObjectDataRx0.ui32MsgIDMask = 0xFFFF;
	sMsgObjectDataRx0.ui32Flags = (MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER);
	sMsgObjectDataRx0.ui32MsgLen = 8;
	CANMessageSet ( CAN0_BASE , 2, &sMsgObjectDataRx0 ,MSG_OBJ_TYPE_RX );

	printf("done.\n");
	printf("System clock used for CAN bus timing: %d Hz\n", SysCtlClockGet());
	return;
}

void ConfigureQEI0(){
/* ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----
 * GPIO Konfiguration
 * PHA = PF0
 * PHB = PF1
*/

    printf("Initializing QEI ...");
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);         // Enable the QEI0 peripheral
   // while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);         // Enable the QEI0 peripheral
   // while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0));  // Wait for the QEI0 module to be ready.
    //Set Pins to be PHA0 and PHB0

    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

    GPIOPinConfigure(GPIO_PF0_PHA0);
    GPIOPinConfigure(GPIO_PF1_PHB0);

    GPIOPinTypeQEI(GPIO_PORTF_BASE, GPIO_PIN_0);
    GPIOPinTypeQEI(GPIO_PORTF_BASE, GPIO_PIN_1);

    //DISable peripheral and int before configuration
    QEIDisable(QEI0_BASE);
    QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

    QEIConfigure(QEI0_BASE, QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP, 1000);

    //Enable velocity capture QEI Module 0
    //QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, SysCtlClockGet() * TIME_TO_COUNT);
    // 3rd Parameter = Number of clock cycles to count ticks
    // ex. 80000000 = sysctlclock = 1 sek
    //QEIVelocityEnable(QEI0_BASE);

    // enable QEI module
    QEIEnable(QEI0_BASE);

    //Set Register start Values
    QEIPositionSet(QEI0_BASE,500);

    // Configure velocity measurement
    //QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, QEI_TIME_TO_COUNT*SysCtlClockGet());
    //QEIVelocityEnable(QEI0_BASE);

    // enable gpio interrupts to allow period measurement
    GPIOIntDisable(GPIO_PORTF_BASE, GPIO_PIN_0);
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_0);
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0,GPIO_RISING_EDGE);
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0);
    printf(" done.\n");
    return;
}


void Init_PWM(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

  //  HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
  //  HWREG(GPIO_PORTF_BASE + GPIO_O_CR)   |= 0x01;

    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;


    //Seite 1233 im Datenblatt
    GPIOPinConfigure(GPIO_PF3_M1PWM7);

    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);

    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 25000);

    //!!!!
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 2000);

    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT , true);
}

void Init_Winkel(void)
{
    /* ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----
     * GPIO Konfiguration
     * PHA = PC5
     * PHB = PC6
     * IDX = PC4 -> not used at the moment
    */

        printf("Initializing QEI-Winkel ...");
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);         // Enable the QEI0 peripheral
       // while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
        SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);         // Enable the QEI0 peripheral
       // while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0));  // Wait for the QEI0 module to be ready.
        //Set Pins to be PHA0 and PHB0

        GPIOPinConfigure(GPIO_PC5_PHA1);
        GPIOPinConfigure(GPIO_PC6_PHB1);

        GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5);
        GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_6);

        //DISable peripheral and int before configuration
        QEIDisable(QEI1_BASE);
        QEIIntDisable(QEI1_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

        QEIConfigure(QEI1_BASE, QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP, 1000);

        //Enable velocity capture QEI Module 0
        //QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, SysCtlClockGet() * TIME_TO_COUNT);
        // 3rd Parameter = Number of clock cycles to count ticks
        // ex. 80000000 = sysctlclock = 1 sek
        //QEIVelocityEnable(QEI0_BASE);

        // enable QEI module
        QEIEnable(QEI1_BASE);

        //Set Register start Values
        QEIPositionSet(QEI1_BASE,500);

        // Configure velocity measurement
        //QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, QEI_TIME_TO_COUNT*SysCtlClockGet());
        //QEIVelocityEnable(QEI0_BASE);

        // enable gpio interrupts to allow period measurement
        //GPIOIntDisable(GPIO_PORTF_BASE, GPIO_PIN_0);
        //GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_0);
        //GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0,GPIO_RISING_EDGE);
        //GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0);
        printf(" done.\n");
        return;

}
// End of file.
