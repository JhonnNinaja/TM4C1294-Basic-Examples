#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "inc/hw_ints.h"

#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "driverlib/fpu.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/pwm.h"


#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define MPU_9250_ADDRESS 0x68
#define SMPLRT_DIV 0x19
#define PWR_MAGNT_1 0x6B
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define INT_ENABLE 0x38
#define g_Div 400

int16_t Accel_X, Accel_Y, Accel_Z;
int16_t Gyro_X, Gyro_Y, Gyro_Z;
float Ax, Ay, Az, Gx, Gy, Gz;
char buffer[20];
float angulo;


int32_t g_ui32SysClock;
char data[20]="";
uint32_t w_left = 0;
uint32_t w_right = 0;
uint32_t pulses = 0;
uint32_t pulses2 = 0;
int32_t error = 0;
int32_t error2 = 0;
uint8_t w = 0;
uint8_t s = 0;
uint8_t l = 0;
uint8_t r = 0;






void ConfigureTimer1A(void)
{
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);


    // se configura los timers en 32 bits 
    MAP_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);

    MAP_TimerLoadSet(TIMER1_BASE, TIMER_A, g_ui32SysClock/g_Div);


    MAP_IntEnable(INT_TIMER1A);

    MAP_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    MAP_TimerEnable(TIMER1_BASE, TIMER_A);

}
void
Timer1IntHandler(void)
{
    
    MAP_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	
	
	MAP_IntMasterDisable();
	
    
    MAP_IntMasterEnable();
	
	
	
}
int main()
{
	

    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_240), 120000000);
                                             
    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    //MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    //MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
	
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);


	ConfigureTimer1A();
	MAP_IntMasterEnable();
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_2);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_4); //rele
    //pololu2
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2); //STEP
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_2); //DIR
    //pololu4
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_0); //STEP
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_4); //DIR

    // Se habilita el enable de todos los pololus
    MAP_GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_2, 0x00);
    // Se establece la dirección del pololu 2 y 4
    MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_2, GPIO_PIN_2);
    MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4, GPIO_PIN_4);

    //se activa el rele
    MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_4, 0x00);
    SysCtlDelay(g_ui32SysClock);
    MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_4, GPIO_PIN_4);
    while (1)
    {
        
		MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
        MAP_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, GPIO_PIN_0);
	    SysCtlDelay(g_ui32SysClock/(g_Div*2));
        MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);
        MAP_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, 0x00);
        SysCtlDelay(g_ui32SysClock/(g_Div*2));
    }
    
}

