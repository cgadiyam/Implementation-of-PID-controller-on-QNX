/*
 * control_project.c
 *
 *  Created on: Oct 20, 2015
 *      Authors: cg8327, bec5030
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>

#include <semaphore.h>
#include <pthread.h>
#include <sys/trace.h>
#include <sys/neutrino.h>
#include <stdint.h>
#include <sys/mman.h>
#include <hw/inout.h>
#include <stdbool.h>

#define DELAY_USEC  10000
#define BASE 0x280

#define IO_PORT_SIZE 32
#define BASE_ADDRESS 0x280

sem_t syncADC;
sem_t syncDAC;
sem_t syncPID;
int set_point = 2300;
int millivolts = 0;
int Digital_Output = 0;
bool kill_threads = false;
int loop_counter = 0;

void* ADCin ();
void* DACout();
void* PID();
void* kill_thread_func();

int main(int argc, char* argv[])
{
	pthread_t Input_Scan;
	pthread_t PID_Controller;
	pthread_t Plant_Simulator_Output;
	pthread_t Kill_all;
	pthread_attr_t attr;
	sem_init(&syncPID,0,0);
	sem_init(&syncDAC,0,0);
	sem_init(&syncADC,0,0);
	pthread_attr_init( &attr );
	pthread_create( &Kill_all, &attr, &kill_thread_func, NULL);
	pthread_create( &Input_Scan, &attr, &ADCin, NULL );
	pthread_create( &PID_Controller, &attr, &DACout, NULL );
	pthread_create( &Plant_Simulator_Output, &attr, &PID, NULL );
	while(!kill_threads);
	sleep(1);
	pthread_join(Kill_all, NULL);
	pthread_join(Input_Scan, NULL);
	pthread_join(PID_Controller, NULL);
	pthread_join(Plant_Simulator_Output, NULL);
	sem_destroy(&syncADC);
	sem_destroy(&syncDAC);
	sem_destroy(&syncPID);
	printf("loop counter = %d\n", loop_counter);
	return 0;
}


void* kill_thread_func ()
{
	kill_threads = false;
	sleep(15);
	kill_threads = true;
	return NULL;
}

void* ADCin ()
{
	sem_post(&syncADC);
	/* Request access to the hardware from QNX */
	if ( ThreadCtl(_NTO_TCTL_IO, NULL) == -1)
	{
		perror("Failed to get I/O access permission");
	}

	/* Set the ADC */
	/* Map the registers */
	uintptr_t inChannelSelect = mmap_device_io(IO_PORT_SIZE, BASE_ADDRESS+2);
	if(inChannelSelect == MAP_DEVICE_FAILED)
	{
		perror("Failed to map control register");
	}
	// Choose channel 4 only!
	out8(inChannelSelect, 0x44);

	uintptr_t rangeSelect = mmap_device_io(IO_PORT_SIZE, BASE_ADDRESS+3);
	if(rangeSelect == MAP_DEVICE_FAILED)
	{
		perror("Failed to map control register");
	}

	// Choose channel +-5v!
	out8(rangeSelect, 0x01);

	//Wait for it to settle
	while (in8(BASE_ADDRESS+3) & 0x20){
	}

	uintptr_t startConversion = mmap_device_io(IO_PORT_SIZE, BASE_ADDRESS+0);
	if(startConversion == MAP_DEVICE_FAILED)
	{
		perror("Failed to map control register");
	}

	uintptr_t MSB = mmap_device_io(IO_PORT_SIZE, BASE_ADDRESS+1);
	if(MSB == MAP_DEVICE_FAILED)
	{
		perror("Failed to map control register");
	}
	out8(startConversion, 0x80);

	while(true)
	{
		sem_wait(&syncADC);

		//Wait for conversion
		while (in8(BASE_ADDRESS+3) & 0x80){
		}

		int high = in8(MSB);
		//low bits are same register as startConverstion
		int low = in8(startConversion);

		int adc_val = high*256 + low;
		millivolts = (adc_val * 5000) / 32768;
		if (millivolts>5000){
			millivolts -=10000;
		}

		if (kill_threads == true) {
			sem_post(&syncPID);
			break;
		}
		else {
			//Start converting the next one!
			out8(startConversion, 0x80);
			sem_post(&syncPID);
		}
	}
	return NULL;
}

void* DACout()
{
	// This has a resolution of 4.88 millivolts.

	/* Request access to the hardware from QNX */
		if ( ThreadCtl(_NTO_TCTL_IO, NULL) == -1)
		{
			perror("Failed to get I/O access permission");
		}

	/* Set the DAC */
	/* Map the registers */
	uintptr_t outLSB = mmap_device_io(IO_PORT_SIZE, BASE_ADDRESS+6);
	if(outLSB == MAP_DEVICE_FAILED)
	{
		perror("Failed to map control register");
	}
		uintptr_t outMSB = mmap_device_io(IO_PORT_SIZE, BASE_ADDRESS+7);
	if(outMSB == MAP_DEVICE_FAILED)
	{
		perror("Failed to map control register");
	}

	uintptr_t conversionTime = mmap_device_io(IO_PORT_SIZE, BASE_ADDRESS+3);
	if(conversionTime == MAP_DEVICE_FAILED)
	{
		perror("Failed to map control register");
	}



	while(true)
	{
		sem_wait(&syncDAC);

		//Wait for previous DA to convert
		while (in8(BASE_ADDRESS+3) & 0x10){
		}
		//Digital_Output = set_point;
		int dac_val = ((2048*Digital_Output)/10000)+ 2048;
		// Write stuff to the registers!
		out8(outLSB, (dac_val & 0xff));
		//(1<<6) is choosing channel 1
		out8(outMSB, (dac_val / 0x100) + (1<<6));

		sem_post(&syncADC);

		if (kill_threads == true) {
			// Put it back to 0 when we are done
			int dac_val;
			dac_val = ((2048*0)/10000)+ 2048;
			out8(outLSB, (dac_val & 0xff));
			out8(outMSB, (dac_val / 0x100) + (1<<6));
			break;
		}
	}
	return NULL;
}

void* PID()
{
	double last_position = 0;
	int total_i = 0;
	double ki = .03;
	double kp = 2.5;
	double kd = 2.5;
	double p = 0.0;
	double i = 0.0;
	double d = 0.0;
	while(true)
	{
		sem_wait(&syncPID);
		int error = set_point - millivolts;

		/* Proportional */
		p = kp*error;

		/* Integral */
		total_i = total_i + (double)error;
		if (total_i >100000) {
			total_i=100000;
		}
		else if (total_i<-100000) {
			total_i=-100000;
		}
		i = ki * total_i;

		/* Derivative */
		d = kd * (last_position-error);
		last_position = error;
		Digital_Output = (int)(i+p+d);

		//Limit it to 0V-5V
		if (Digital_Output > 5000) {
			Digital_Output = 5000;
		}
		else if (Digital_Output +5000 < 0) {
			Digital_Output = -5000;
		}
		loop_counter++;
		sem_post(&syncDAC);
		if (kill_threads == true) {
			break;
		}
	}
	return NULL;
}
