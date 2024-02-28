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


void
GPIOEIntHandler(void)
{
    uint32_t ui32Status;

    ui32Status = MAP_GPIOIntStatus(GPIO_PORTE_BASE, true);

    MAP_GPIOIntClear(GPIO_PORTE_BASE, ui32Status);
    UARTCharPut(UART0_BASE, 'M');
    pulses++;

    if(pulses >= w_left){
            //MOTOR 1 
         

            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, 0x00);
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_2, 0x00);
            
        }
}

void
GPIOGIntHandler(void)
{
    uint32_t ui32Status;

    ui32Status = MAP_GPIOIntStatus(GPIO_PORTG_BASE, true);

    MAP_GPIOIntClear(GPIO_PORTG_BASE, ui32Status);

    pulses2++;

    if(pulses2 >= w_right){
            //MOTOR 2


            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, 0x00);
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, 0x00);
            
        } 
}
void
ConfigureUART(void)
{

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 115200, g_ui32SysClock);
    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    MAP_UARTConfigSetExpClk(UART0_BASE, g_ui32SysClock, 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    MAP_IntEnable(INT_UART0);
    MAP_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

}

void
UARTIntHandler(void)
{
	
    uint32_t ui32Status;
    ui32Status = MAP_UARTIntStatus(UART0_BASE, true);
	uint8_t idx=0;
    MAP_UARTIntClear(UART0_BASE, ui32Status);
	
    while(MAP_UARTCharsAvail(UART0_BASE))
    {
	data[idx]=MAP_UARTCharGetNonBlocking(UART0_BASE);//tomamos el dato
        //MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
        SysCtlDelay(g_ui32SysClock / (1000 * 3));
        //MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
        idx++;
		
    }
	
	//UARTprintf("-------------------------------\n%s\n",data);
     
        
    if(data[0]=='U' && data[1]=='P') 
    {
        
        pulses = 0;
        pulses2 = 0;

        w_left = 100000;
        w_right = 100000;
        SysCtlDelay(g_ui32SysClock / (1000));
        //MOTOR 1      
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, 0x00);
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_2, GPIO_PIN_2);
        //MOTOR 2
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, 0x00);
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, GPIO_PIN_7);
        
        

        
    }
    if(data[0]=='D' && data[1]=='O'&& data[2]=='W'&& data[3]=='N') 
    { 
        pulses = 0;
        pulses2 = 0;

        w_left = 100000;
        w_right = 100000;
        SysCtlDelay(g_ui32SysClock / (1000));
        //MOTOR 1      
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, GPIO_PIN_1);
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_2, 0x00);
        //MOTOR 2
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, GPIO_PIN_6);
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, 0x00);
        
    }
    if(data[0]=='L' && data[1]=='E'&& data[2]=='F'&& data[3]=='T') 
    { 
        pulses = 0;
        pulses2 = 0;

        w_left = 100000;
        w_right = 100000;
        SysCtlDelay(g_ui32SysClock / (1000));
        //MOTOR 1      
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, 0x00);
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_2, GPIO_PIN_2);
        //MOTOR 2
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, GPIO_PIN_6);
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, 0x00);
        
    }
    if(data[0]=='R' && data[1]=='I'&& data[2]=='G'&& data[3]=='T'&& data[4]=='H')  {
        pulses = 0;
        pulses2 = 0;

        w_left = 100000;
        w_right = 100000;
        SysCtlDelay(g_ui32SysClock / (1000));
        //MOTOR 1      
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, GPIO_PIN_1);
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_2, 0x00);
        //MOTOR 2
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, 0x00);
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, GPIO_PIN_7);
        
    }
    if(data[0]=='S' && data[1]=='T'&& data[2]=='O'&& data[3]=='P')  {
        //MOTOR 1      
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, 0x00);
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_2, 0x00);
        //MOTOR 2
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, 0x00);
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, 0x00);
        SysCtlDelay(g_ui32SysClock / (1000 * 3));
        w_left = 0;
        w_right = 0;
        
    }
		
}

void pwm_config()
{
    uint32_t ui32PWMClockRate;

    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_240), 120000000);
    // The PWM peripheral must be enabled for use.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    //
    // Enable the GPIO port that is used for the PWM output.

    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    MAP_GPIOPinConfigure(GPIO_PK4_M0PWM6);
    MAP_GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_4); //pulses
    MAP_GPIOPinConfigure(GPIO_PK5_M0PWM7);
    MAP_GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_5); //pulses2
    MAP_PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_8);

    ui32PWMClockRate = g_ui32SysClock / 8;

    MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_3,
                        PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);


    MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, (ui32PWMClockRate / 250));

    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
                         MAP_PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) / 4);
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7,
                         MAP_PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) / 4);

    MAP_PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
    MAP_PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
    MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_3);
}

void initI2C2(void)
{

	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);

	MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2);

	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

	// Se onfigura el MUX para los pines en el puerto N
	MAP_GPIOPinConfigure(GPIO_PN5_I2C2SCL);
	MAP_GPIOPinConfigure(GPIO_PN4_I2C2SDA);

	// Se selecciona la función para los pines designados
	MAP_GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);
	MAP_GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);
	MAP_I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), false);

	//Se limpian los FiFos
	HWREG(I2C2_BASE + I2C_O_FIFOCTL) = 80008000;
}

uint8_t readI2C2(uint16_t device_address, uint16_t device_register)
{
	I2CMasterSlaveAddrSet(I2C2_BASE, device_address, false);
	I2CMasterDataPut(I2C2_BASE, device_register);
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_SEND);
	
	while(!(I2CMasterBusy(I2C2_BASE)));
	while(I2CMasterBusy(I2C2_BASE));
	I2CMasterSlaveAddrSet(I2C2_BASE, device_address, true);
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
	while(!(I2CMasterBusy(I2C2_BASE)));
	while(I2CMasterBusy(I2C2_BASE));
	return( I2CMasterDataGet(I2C2_BASE));
}

void writeI2C2(uint16_t device_address, uint16_t device_register, uint8_t device_data)
{
	I2CMasterSlaveAddrSet(I2C2_BASE, device_address, false);
	I2CMasterDataPut(I2C2_BASE, device_register);
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while(!(I2CMasterBusy(I2C2_BASE)));
	while(I2CMasterBusy(I2C2_BASE));

	I2CMasterSlaveAddrSet(I2C2_BASE, device_address, true);
	I2CMasterDataPut(I2C2_BASE, device_data);
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

	while(!(I2CMasterBusy(I2C2_BASE)));
	while(I2CMasterBusy(I2C2_BASE));
}
void MPU_9250_INIT()
{
	writeI2C2(MPU_9250_ADDRESS,0x6B, 0x00);
	writeI2C2(MPU_9250_ADDRESS, SMPLRT_DIV, 7);
	writeI2C2(MPU_9250_ADDRESS, PWR_MAGNT_1, 1);
	writeI2C2(MPU_9250_ADDRESS, CONFIG, 0);
	writeI2C2(MPU_9250_ADDRESS, GYRO_CONFIG, 0x00); //antes 24
	writeI2C2(MPU_9250_ADDRESS, INT_ENABLE, 1);
}

int16_t read_raw_data(uint16_t addr)
{
	uint16_t high, low, value;
	high = readI2C2(MPU_9250_ADDRESS, addr);
	low = readI2C2(MPU_9250_ADDRESS, addr+1);
	value = ((high<<8)|low);

	if(value > 32768)
	{
		value = value - 65536;

	}
	return value;
}

void ConfigureTimer1A(void)
{
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);


    // se configura los timers en 32 bits 
    MAP_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);

    MAP_TimerLoadSet(TIMER1_BASE, TIMER_A, g_ui32SysClock/10);


    MAP_IntEnable(INT_TIMER1A);

    MAP_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    MAP_TimerEnable(TIMER1_BASE, TIMER_A);

}
void
Timer1IntHandler(void)
{
    
    MAP_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	
	
	MAP_IntMasterDisable();
	
    Accel_X = read_raw_data(ACCEL_XOUT_H);
	Accel_Y = read_raw_data(ACCEL_YOUT_H);
	Accel_Z = read_raw_data(ACCEL_ZOUT_H);
	Gyro_X = read_raw_data(GYRO_XOUT_H);
	Gyro_Y = read_raw_data(GYRO_YOUT_H);
	Gyro_Z = read_raw_data(GYRO_ZOUT_H);

	
	
    MAP_IntMasterEnable();
	Ax = Accel_X/16384.0;
	Ay = Accel_Y/16384.0;
	Az = Accel_Z/16384.0;
	Gx = Gyro_X/131.0;
	Gy = Gyro_Y/131.0;
	Gz = Gyro_Z/131.0;
	angulo = angulo + Gx*0.1 - 0.046;
	
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
	pwm_config();
    initI2C2();

    MPU_9250_INIT();
	ConfigureUART();
    
	ConfigureTimer1A();
	MAP_IntMasterEnable();
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_1);//HABILITAMOS PK1
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_2);//HABILITAMOS PK2
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_6);//HABILITAMOS PK6
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_7);//HABILITAMOS PK7

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //motor1
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_1,
                    GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);//motor2
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTG_BASE, GPIO_PIN_0);
    GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_0,
                    GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOIntTypeSet(GPIO_PORTE_BASE,GPIO_PIN_1,GPIO_FALLING_EDGE); //pulses ++
    GPIOIntRegister(GPIO_PORTE_BASE,&GPIOEIntHandler);
    MAP_IntEnable(INT_GPIOF);
    MAP_GPIOIntEnable(GPIO_PORTE_BASE, GPIO_INT_PIN_1); //pulses ++

    GPIOIntTypeSet(GPIO_PORTG_BASE,GPIO_PIN_0,GPIO_FALLING_EDGE); //pulses2 ++
    GPIOIntRegister(GPIO_PORTG_BASE,&GPIOGIntHandler);
    MAP_IntEnable(INT_GPIOG);
    MAP_GPIOIntEnable(GPIO_PORTG_BASE, GPIO_INT_PIN_0); //pulses2 ++
    
    uint32_t kp = 50;
    

    uint32_t u = 0;
    uint32_t y = 0;
 

    while (1)
    {
        
		error = pulses2 - pulses;
        error2 = pulses - pulses2;
        
        u = 20000+kp*error;
        y = 20000+kp*error2;


        if(u<=0){
            u = -1*u;
        }
        if(u>=59999){
            u = 59999;
        }
        
        if(y<=0){
            y = -1*y;
        }
        if(y>=59999){
            y = 59999;
        }
        //y = kp+kp*error1;

        MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,u); //pulses
        MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7,y); //pulses2
        
        //UARTprintf("%s\n", buffer);
		
		int ang = (int)angulo;
		UARTprintf("%d\n",ang);
        
        
		//SysCtlDelay(1000000);
    }
    
}

