//main.cpp

/********************************************
Projet CONNONBALL using NUCLEO STM32F401RE
								Year 2015 
					Created by MAJiiiC Team
					
			@mail jerome_laurent16@yahoo.fr
*********************************************/

//Include Library
#include "mbed.h"
#include "rtos.h"
#include "SerialDriver.h"

//
//            DU1>--------------------- angle = 0 + i*360 ; i entier
//						 *
//					 *   *
//				 *       *
//			 *  *  *  *  *
//		DU2             DU3>------------- angle = 240 + i*360;
//		  >------------------------------ angle = 120 + i*360;
//
const float SuD = (57.3 * 340) / (0.2f * 1000000) ;  // Speed divided by the distance between 2 points
// 57.3 = 180 / pi to have degrees
// 1000000 to convert in µs
	
	
	
//IO definition
//SerialDriver usb(USBTX, USBRX,2,2);

Serial usb(USBTX, USBRX);
InterruptIn detect1(D2);
InterruptIn detect2(D3);
InterruptIn detect3(D4); // set utrasonic connectors
PwmOut servo(D9); //PWM on the D9 connector

PwmOut STEERING_SERVO_PIN(D5); // set servo-motor connector for steering
PwmOut THROTTLE_SERVO_PIN(D6); // set servo-motor connector for throttle
DigitalOut led(LED1);

// PWM car control
uint8_t steeringTarget;
uint8_t throttleTarget;

//Timers
//Timer timer1;
Timer timer2;

//Variables
unsigned int time_data_check;
unsigned int time1,time2,time3;
uint8_t reach1,reach2,reach3;



//declaration of the Threads
Thread * USB_receive_thread;
Thread * car_control_thread;
Thread * car_emergency_thread;

Thread * camera_control_thread;

// Definition of triggers for the interrupts
void trigger1();
void trigger2();
void trigger3();

//definition of the function used in threads
void USB_recieve(void const *argument);
void car_emergency(void const *argument);
void car_control(void const *argument);


void trigger1();
void trigger2();
void trigger3();
void camera_control(void const *argument);
void treatement(uint16_t &angle, const uint8_t &combination, uint8_t &round);
void set_camera(const uint16_t &angle);


/********************************************************************************/
//Car control
/*
void USB_recieve(void const *argument) {
	usb.baud(115200);
	//usb.printf("USB reception started\n");
	//led = false;
	//while (usb.getc() != 255);
	
	//led = true;
	//usb.printf("OK\n");
	while(true)
		{
			//usb.printf("Ready to get data\n");
			time_data_check=timer1.read_ms();
			
			usb.putc(steeringTarget = usb.getc());
			usb.putc(throttleTarget = usb.getc());
			
			car_control_thread->signal_set(0x01); //there is a data
			car_emergency_thread->signal_set(0x01); //free the emergency
		}
}

void car_emergency(void const *argument) {
	//usb.printf("Car emergency started\n");
	while(1) {
		if ((timer1.read_ms() - time_data_check > 5000)) //the delay is to hight
		{ 
				//usb.printf("Emergency as occured\n");
				led = true;
				if (throttleTarget < 89) //high speed
						STEERING_SERVO_PIN.write(130/255); //brake
				else 
						THROTTLE_SERVO_PIN.write(91/255); //only slow down, not to go reverse
				
				Thread::signal_wait(0x01); // waiting for a new transmition
				//usb.printf("End the emergency< state\n");
				led = false;
				time_data_check = timer1.read_ms();
		}
		else
			Thread::wait(100);
	}
}

void car_control(void const *argument) {
	STEERING_SERVO_PIN.write(90/255); //initialisation
	THROTTLE_SERVO_PIN.write(91/255);
	timer1.start();
	time_data_check = timer1.read_us();
	//usb.printf("Car control started\n");
	USB_receive_thread = new Thread(USB_recieve);
	car_emergency_thread = new Thread(car_emergency);
	//car_emergency_thread->set_priority(osPriorityHigh); //high level of priority (REAL TIME)
	
	while(1) {
		Thread::signal_wait(0x01); //If there is a data;
		//usb.printf("Write the control to pwm\n");
		STEERING_SERVO_PIN.write(steeringTarget/255);
		THROTTLE_SERVO_PIN.write(throttleTarget/255);
	}
}
*/

/********************************************************************************/


/********************************************************************************/
//Camera_control

void trigger1() {
	time1 = timer2.read_us();
	reach1 = true;
	detect1.disable_irq(); // disable to prevent multi rises
	camera_control_thread->signal_set(0x00);
}
void trigger2() {
	time2 = timer2.read_us();
	reach2 = true;
	detect2.disable_irq(); // disable to prevent multi rises
	camera_control_thread->signal_set(0x00);
}
void trigger3() {
	time3 = timer2.read_us();
	reach3 = true;
	detect3.disable_irq(); // disable to prevent multi rises
	camera_control_thread->signal_set(0x00);
}

void camera_control(void const *argument) { // using in a thread

		unsigned int time_r;
		uint8_t combination = 0;
		uint16_t angle[4] = {0,0,0,0}; // set the servo to 0 T
		uint8_t round = 3;
		
		set_camera(360*3);
		Thread::wait(2000);  //set in middle and wait  ²
	
		timer2.start();
	
    while (true) { 
			// *********  RAZ
			time1 = time2 = time3 = 0;
			reach1 = reach2 = reach3 = false;
			timer2.reset(); 
			
			
			// ********* Detection of signal
			detect1.enable_irq(); // allows interrupt
			detect2.enable_irq();
			detect3.enable_irq();
			
			detect1.fall(&trigger1);
			detect2.fall(&trigger2);
			detect3.fall(&trigger3);
			
			Thread::signal_wait(0x00);// Wait for a receive
			Thread::wait(5); //5ms more to wait for delayed signals
				
			detect1.disable_irq(); // for those not reached
			detect2.disable_irq();
			detect3.disable_irq();
				
			// ********** Info			
			usb.printf("Signal(s) reached :\n");
			usb.printf("1)\tr = %d\tt = %d\n",reach1,time1);
			usb.printf("2)\tr = %d\tt = %d\n",reach2,time2);
			usb.printf("3)\tr = %d\tt = %d\n",reach3,time3);
			
			// ********** Treatment
			combination = (reach3 << 2) + (reach2 << 1) + reach1 ;  //the 3 lasts bits are all the reach's possibilities 	
			treatement(angle[0],combination,round);
			
			// ********** Reject false measure
			// if the position is set logicaly (e.g. not to the oposit in a short time)
			// if no signal since a long time 1second, consider that a good signal
			if( (!(abs(angle[1] - angle[0]) > 130) && (abs(angle[1] - angle[0])	< 230)) || (timer2.read_ms() > 1000) ) {
					
					// ********** Add turn parameter	
						if( ((angle[1] - angle[0]) > 180) && (round < 6) ) {//set the number of rotation
							round++;
						}
						else if( ((angle[1] - angle[0]) < -180) && (round > 0) ) {
							round--;
						}
					angle[1] = angle[0]; //memorize the last value
					angle[2] = angle[0] + 360 * round;	
				
					// ********** Smooth
					
					
					// *********** Set the camera
					set_camera(angle[0]);
			}
			else if(timer2.read_ms() > 1000) {  // if no signal since a long time 1second, consider that a good signal
				
				
			
			// *********** Regulation, time loop control
			time_r = timer2.read_ms();
			if(time_r < 10) { // wait if less than 10ms loop
				Thread::wait(1000 - time_r);
			}
    }
}


void treatement(uint16_t &angle, const uint8_t &combination, uint8_t &round) {

		//usb.printf("Case n %d\n",combination);
		switch(combination) {
			case 1 : // DU1 is reached
				angle = 0;
				break;
		
			case 2 : // DU2 is reached
				angle = 120;
				break;
			
			case 3 : // DU1 and DU2 are reached
				angle = 60 - SuD * (time1 - time2); //determined in the repport , 57.3 = 180 / pi
				break;
			
			case 4 : // DU3 is reached
				angle = 240;
				break;
			
			case 5 : // DU1 and DU3 are reached
				angle = 300 - SuD * (time3 - time1); //determined in the repport
				break;
			
			case 6 : // DU2 and DU3 are reached
				angle = 180 - SuD * (time2 - time3);  //determined in the repport
				break;
		
			case 7 : // DU1 and DU2 and DU3 are reached
				if(time1 >= time2 & time1 >= time3) // DU1 is rejected, DU2 and DU3 are reached
					angle = 180 - SuD * (time2 - time3); // as case 6
				else if(time2 >= time1 & time2 >= time3) // DU2 is rejected, DU1 and DU3 are reached
					angle = 300 - SuD * (time3 - time1); // as case 5
				else // DU3 is rejected, DU1 and DU2 are reached
					angle = 60 - SuD * (time1 - time2); // as case 3
				break;
		 
			case 0 : // no reach	
			default :
				angle = 0;
				break;
		}
		//usb.printf("angle is compute : %d\n",angle);
}
	

void set_camera(const uint16_t &angle) { // between 0 and 2160 (= 6*360°)
		if( angle < 2160) {
			servo = (float(angle) / 43200.0f) + 0.05f; //initialy 0.05 for 0T, 0.10 for 6T
			//usb.printf("Camera is set at %d\n",angle);
		}
}

/********************************************************************************/


/********************************************************************************/
//Main function
int main() {
	int X;
	while (1) {
		set_camera(X);
		wait_ms(100);
		X += 50;
		if(X > 2160 )
			X = 0;
	}
	m
	usb.baud(9600);
	//led = false;
	//car_control_thread = new Thread(car_control);
	camera_control_thread = new Thread(camera_control);
	
    while(1) {}
}
