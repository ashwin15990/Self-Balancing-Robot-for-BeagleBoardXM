
#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdlib.h>
#include <sched.h>
#include <unistd.h>
#include <time.h>
#include <syslog.h>
#include <math.h>
#include <sys/param.h>
#include "i2c-dev.h"
#include <fcntl.h>
#include <sys/stat.h>

/* ADXL345 Registers */
#define DATA_FORMAT_REG 0x31
#define POWER_CTRL_REG 0x2D
#define FIFO_CTRL_REG 0x38
#define PI 3.14

#define X_CO_L 0x32
#define X_CO_H 0x33

#define Y_CO_L 0x34
#define Y_CO_H 0x35

#define Z_CO_L 0x36
#define Z_CO_H 0x37

/* ITG3200 Registers */
#define DLPF_FS 0x16
#define SMPLRT_DIV 0x15

#define ITG_X_CO_H 0x1D
#define ITG_X_CO_L 0x1E

#define ITG_Y_CO_H 0x1F
#define ITG_Y_CO_L 0x20

#define ITG_Z_CO_H 0x21
#define ITG_Z_CO_L 0x22

int x_a_val,y_a_val,z_a_val;
double x_acc, y_acc, z_acc;

int x_gyro,y_gyro,z_gyro;

pthread_t task1, task2, task3;
pthread_attr_t task1_sched_attr;
pthread_attr_t task2_sched_attr;
pthread_attr_t task3_sched_attr;
pthread_attr_t main_sched_attr;
int rt_max_prio, rt_min_prio;
struct sched_param task1_param;
struct sched_param task2_param;
struct sched_param task3_param;
struct sched_param main_param;

double start = 0;
sem_t sem_t1, sem_t2, sem_t3;
int flag = 1;
int abort_task1, abort_task2, abort_task3;
double pitch;

double readTOD(void)
{
 struct timeval tv;
 double ft=0.0;
 if( gettimeofday (& tv, NULL) != 0)
 {
 perror("readTOD");
 return 0.0;
 }
 else
 {
 ft = ((double)(((double)tv.tv_sec) + (((double)tv.tv_usec) /
1000000.0)));
 }
 return ft;
}

/* ADXL345 Accelerometer functions */
void ADXL345_setup(int file)
{
	i2c_smbus_write_byte_data(file, DATA_FORMAT_REG, 0x0B);

	i2c_smbus_write_byte_data(file, POWER_CTRL_REG, 0x08);		
}

void* ADXL345_i2c(void *threadid) //*********************************************
{
	int file,i;
	int adapter_no = 2;
	char filename[20];
	double stop = 0.0;
	
	int addr = 0x53; /* The I2C address */

	char reg; /*= 0x10;  Device register to access */
  	char res;
  	char buf[10];

	int alpha = 0.5;
	
	snprintf(filename, 19, "/dev/i2c-%d", adapter_no);
	file = open(filename , O_RDWR);

	if (file < 0) 
	{
		perror("NO FILE DESCRIPTOR \n");
    	    	exit(1);
  	}	

  	if (ioctl(file, I2C_SLAVE, addr) < 0) 
	{
    		perror("I2C COMMUNICATION ESTABLISHMENT FAILED");
    		exit(1);
  	}

	ADXL345_setup(file);
	
	/* WRITE TO FIFO_CTL REGISTER */
	i2c_smbus_write_word_data(file, FIFO_CTRL_REG, 0x83);
	
	while(1)
	//for(i=0;i<10;i++)
	{
		/* reading accelerometer data */
		x_a_val = i2c_smbus_read_byte_data(file, X_CO_H)<<8;
		x_a_val |=  i2c_smbus_read_byte_data(file, X_CO_L);

		y_a_val = i2c_smbus_read_byte_data(file, Y_CO_H)<<8; 
		y_a_val |=  i2c_smbus_read_byte_data(file, Y_CO_L);

		z_a_val = i2c_smbus_read_byte_data(file, Z_CO_H)<<8; 
		z_a_val |=  i2c_smbus_read_byte_data(file, Z_CO_L);
		/*-----------------------------*/

	  	if ((x_a_val < 0)||(y_a_val<0)||(z_a_val<0)) 	    		/* ERROR HANDLING: i2c transaction failed */
			printf("\n I2C TRANSACTION FAILED ");
		else 		    		/* res contains the read word */
		{
			x_acc = x_a_val * 0.0039;
			y_acc = y_a_val * 0.0039;
			z_acc = z_a_val * 0.0039;

			x_acc = x_a_val * alpha + (x_acc * (1.0 - alpha));
			y_acc = y_a_val * alpha + (y_acc * (1.0 - alpha));
			z_acc = z_a_val * alpha + (z_acc * (1.0 - alpha));
		
			if(x_acc>=0 && x_acc<=1)
                                pitch = (asin(x_acc)*180.0)/PI;
                        else
                                pitch = -1*(asin(256.0-x_acc)*180.0)/PI;

			//printf("ACCELEROMETER : X co-ordinates : %lf\t Y co-ordinates : %lf\t Z coordinates : %lf\t ANGLE : %lf\n\n",x_acc,y_acc,z_acc,pitch);
		
		
		if(abort_task1 == 1)
		{
			printf("ACCELEROMETER : X co-ordinates : %lf\t Y co-ordinates : %lf\t Z coordinates : %lf\t ANGLE : %lf\n\n",x_acc,y_acc,z_acc,pitch);
			//stop = readTOD();
			//printf("Time elapsed = %lf\n", (double)(stop-start));			
			sem_wait(&sem_t1);
		}
	
		}
	}
	close(file);
}

/* ITG-3200 Gyroscope functions */
void ITG3200_setup(int file)
{
	char id;
	id = i2c_smbus_read_byte_data(file, 0x00); //read from WHO_AM_I Register;

	printf("WHO AM I : %d\n\n",id); 

	//Set the gyroscope scale for the outputs to +/-2000 degrees per second
	i2c_smbus_write_byte_data(file, DLPF_FS, 0x18); 

	//Set the sample rate to 100 hz
	i2c_smbus_write_byte_data(file, SMPLRT_DIV, 0x09);		
}

void* ITG3200_i2c(void *threadid)  //GYROSCOPE
{
	int file,i;
	int adapter_no = 2;
	char filename[20];	
	double stop = 0;

	int addr = 0x69; /* The I2C address */

	snprintf(filename, 19, "/dev/i2c-%d", adapter_no);
	file = open(filename , O_RDWR);

	if (file < 0) 
	{
		perror("NO FILE DESCRIPTOR \n");
    	    	exit(1);
  	}	

  	if (ioctl(file, I2C_SLAVE, addr) < 0) 
	{
    		perror("I2C COMMUNICATION ESTABLISHMENT FAILED");
    		exit(1);
  	}

	ITG3200_setup(file);
	while(1)
	//for(i=0;i<10;i++)
	{		
		x_gyro = i2c_smbus_read_byte_data(file, ITG_X_CO_H)<<8;
		x_gyro |=  i2c_smbus_read_byte_data(file, ITG_X_CO_L);

		y_gyro = i2c_smbus_read_byte_data(file, ITG_Y_CO_H)<<8; 
		y_gyro |=  i2c_smbus_read_byte_data(file, ITG_Y_CO_L);

		z_gyro = i2c_smbus_read_byte_data(file, ITG_Z_CO_H)<<8; 
		z_gyro |=  i2c_smbus_read_byte_data(file, ITG_Z_CO_L);

	  	//if ((x_gyro < 0)||(y_gyro<0)||(z_gyro<0)) 	    		/* ERROR HANDLING: i2c transaction failed */
		//	printf("\n I2C TRANSACTION FAILED ");
		//else 		    		/* res contains the read word */
		//	printf("GYROSCOPE : X co-ordinates : %d\t Y co-ordinates : %d\t Z coordinates : %d\n",x_gyro,y_gyro,z_gyro);
	

		if(abort_task2 == 1)
		{
		    printf("GYROSCOPE : X co-ordinates : %d\t Y co-ordinates : %d\t Z coordinates : %d\n",x_gyro,y_gyro,z_gyro);
		    //stop = readTOD();
		    //printf("Time elapsed = %lf\n", (double)(stop-start));			   
		    sem_wait(&sem_t2);
		}
	}
	close(file);
}

/* DC MOTOR FUNTIONS */
void stop(int file)
{
	int regValue = 0;
	i2c_smbus_write_byte_data(file, 0x00, regValue); 
}

void drive(int file)
{
	int speed;
	char regValue;
	speed = 63;

	i2c_smbus_write_byte_data(file, 0x01, 0x80); // clear the fault status
	
	regValue = (char)abs(speed);
	regValue = regValue<<2;
	
	if(pitch >= 3.0)
            regValue |= 0x02;
        else if(pitch < -9.0)
            regValue |= 0x01;
        else if(pitch > -9.0 && pitch < 3.0)
            regValue = 0x00;

	i2c_smbus_write_byte_data(file, 0x00, regValue);

}

void* DC_motor(void *threadid)
{
	int file,i;
	int adapter_no = 2;
	char filename[20];
	double stop = 0;
	
	int addr = 0x68; /* The I2C address */

	snprintf(filename, 19, "/dev/i2c-%d", adapter_no);
	file = open(filename , O_RDWR);

	if (file < 0) 
	{
		perror("NO FILE DESCRIPTOR \n");
    	    	exit(1);
  	}	

  	if (ioctl(file, I2C_SLAVE, addr) < 0) 
	{
    		perror("I2C COMMUNICATION ESTABLISHMENT FAILED");
    		exit(1);
  	}	

	while(1)
	{
	drive(file);
        
  	stop = readTOD();
  	//printf("Time Elapsed: %lf\n\n", (double) (stop-start));
	
	if(abort_task3 == 1)
	{
	   printf("DC Motor control\n");
	   //stop = readTOD();
	   //printf("Time elapsed = %lf\n", (double)(stop-start));
	   sem_wait(&sem_t3);
	}
	}

}



int main(int argc, char *argv[])
{

 int rc;
 useconds_t t_50,t_500;
 abort_task1 = 1;
 abort_task2 = 1; 
 abort_task3 = 1;
 sem_init (&sem_t1, 0, 0);
 sem_init (&sem_t2, 0, 0);
 sem_init (&sem_t3, 0, 0);

 t_50 = 50000;
 t_500 = 500000;

 pthread_attr_init(&task1_sched_attr);
 pthread_attr_init(&task2_sched_attr);
 pthread_attr_init(&task3_sched_attr);
 pthread_attr_init(&main_sched_attr);
 pthread_attr_setinheritsched(&task1_sched_attr, PTHREAD_EXPLICIT_SCHED);
 pthread_attr_setschedpolicy(&task1_sched_attr, SCHED_FIFO);
 pthread_attr_setinheritsched(&task2_sched_attr, PTHREAD_EXPLICIT_SCHED);
 pthread_attr_setschedpolicy(&task2_sched_attr, SCHED_FIFO);
 pthread_attr_setinheritsched(&task3_sched_attr, PTHREAD_EXPLICIT_SCHED);
 pthread_attr_setschedpolicy(&task3_sched_attr, SCHED_FIFO);
 pthread_attr_setinheritsched(&main_sched_attr, PTHREAD_EXPLICIT_SCHED);
 pthread_attr_setschedpolicy(&main_sched_attr, SCHED_FIFO);
 rt_max_prio = sched_get_priority_max(SCHED_FIFO);
 rt_min_prio = sched_get_priority_min(SCHED_FIFO);

 main_param.sched_priority = rt_max_prio;
 task1_param.sched_priority = rt_max_prio-1;
 task2_param.sched_priority = rt_max_prio-2;
 task3_param.sched_priority = rt_max_prio-3;

 rc=sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
 if (rc)
 {
 printf("ERROR; sched_setscheduler rc is %d\n", rc); perror(NULL); exit(-1);

 }

 pthread_attr_setschedparam(&task1_sched_attr, &task1_param);
 pthread_attr_setschedparam(&task2_sched_attr, &task2_param);
 pthread_attr_setschedparam(&task3_sched_attr, &task3_param);
 pthread_attr_setschedparam(&main_sched_attr, &main_param);

 //start = readTOD();
 rc= pthread_create (&task1, &task1_sched_attr, ADXL345_i2c, ( void *)0 );
 if (rc)
 {
 printf("ERROR; pthread_create() rc is %d\n", rc); perror(NULL); exit(-1);
 }
 rc= pthread_create (&task2, &task2_sched_attr, ITG3200_i2c, ( void *)0 );
 if (rc)
 {
 printf("ERROR; pthread_create() rc is %d\n", rc); perror(NULL); exit(-1);
 }
 rc= pthread_create (&task3, &task3_sched_attr, DC_motor, ( void *)0 );
 if (rc)
 {
 printf("ERROR; pthread_create() rc is %d\n", rc); perror(NULL); exit(-1);
 }
  
 /* Basic sequence of releases after CI */

start = readTOD();
 while(1)
{
 abort_task1 = 0;
 sem_post(&sem_t1);
 usleep(5000);
 abort_task1 = 1;
 
 abort_task2 = 0;
 sem_post (&sem_t2);
 usleep(5000);
 abort_task2 = 1;

 abort_task3 = 0;
 sem_post(&sem_t3);
 usleep(10000);
 abort_task3 = 1;
}


 pthread_join(task1, NULL);
 pthread_join(task2, NULL);
 pthread_join(task3, NULL);
 if(pthread_attr_destroy(&task1_sched_attr) != 0)
 perror("attr destroy");
 if(pthread_attr_destroy(&task2_sched_attr) != 0)
 perror("attr destroy");
 if(pthread_attr_destroy(&task3_sched_attr) != 0)
 perror("attr destroy");

}
