/*
 * MPU6050Thread.c
 *
 *  Created on: Apr 11, 2020
 *      Author: abusous2000
 */
#include "ch.h"
#include "hal.h"
#include "Strust4EmbeddedConf.h"
#include "Strust4Embedded.h"
#include "ButtonLEDs.h"
#include "MQTTClient.h"
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050Thread.h"


static int pitch = 10;
static int role = 10;
static int yaw = 10;
static char  payload[256]	= {0};

enum MPU605_ALGORITHM currentAlgorithm = MPU605_ALGORITHM_RAW;
int16_t XaccelRaw, YaccelRaw, ZaccelRaw, accelXError, tempRaw, accelYError, accelZError;
int16_t XgyroRaw, YgyroRaw, ZgyroRaw, gyroXError, gyroYError , gyroZError;
__IO thread_reference_t 	    dmp6050RawThread 	= NULL;
__IO thread_reference_t 	    mpu6050RawThread	= NULL;
float XaccelAngle, YaccelAngle;
#ifndef STM32F7XX
static const I2CConfig  i2ccfgLow = {
  OPMODE_I2C,
  100000,//400000 can be used however you need external pull-up resistors. But if you can it will be faster
  STD_DUTY_CYCLE,
};
static const I2CConfig i2ccfgHigh = {
  OPMODE_I2C,
  400000,
  FAST_DUTY_CYCLE_2,
};

static const I2CConfig i2ccfgLow2 = {
  OPMODE_I2C,
  100000,
  STD_DUTY_CYCLE,
};
#endif
static uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
static uint8_t dmpReady;
bool sendMsgsToMQTT = true;
static int processed = 0;
static uint8_t packetSize;
extern int8_t MPUGetCurrentFIFOPacket(uint8_t *data, uint8_t length);
static bool_t initMPU6050Raw(void){
	bool_t sts = 0;

	//using the default for gyro and the Accel.:
	/* MPU6050, AD0 is connected to VCC */
	MPU6050(MPU6050_ADDRESS_AD0_LOW);

	/* Test connection */
	sts = MPUtestConnection();
	if (!sts)
		chSysHalt("failed in initMPU6050Raw()");

	MPUreset();
	MPUresetSensors();
	chThdSleepMilliseconds(100);
	MPUinitialize();
	chThdSleepMilliseconds(100);

	MPUgetMotion6(&XaccelRaw, &YaccelRaw, &ZaccelRaw, &XgyroRaw, &YgyroRaw, &ZgyroRaw);

	XaccelAngle = (atan2(YaccelRaw,ZaccelRaw)+M_PI)*RAD_TO_DEG;
	YaccelAngle = (atan2(XaccelRaw,ZaccelRaw)+M_PI)*RAD_TO_DEG;

	return TRUE;
}

void calibrate(int maxSamples){
	gyroXError = gyroYError = gyroZError = 0;
    for (int i=0; i < maxSamples; ++i){
		if ( MPUgetMotion6(&XaccelRaw, &YaccelRaw, &ZaccelRaw, &XgyroRaw, &YgyroRaw, &ZgyroRaw) ){
			accelXError = accelXError + XaccelRaw ;
			accelYError = accelYError + YaccelRaw;
			accelZError = accelZError + ZaccelRaw;

			gyroXError = gyroXError + XgyroRaw;
			gyroYError = gyroYError + YgyroRaw;
			gyroZError = gyroZError + ZgyroRaw;
		}
    }

    accelXError = accelXError/maxSamples;
    accelYError = accelYError/maxSamples;
    accelZError = accelZError/maxSamples;

    gyroXError = gyroXError/maxSamples;
    gyroYError = gyroXError/maxSamples;
    gyroZError = gyroXError/maxSamples;
	dbgprintf("AX: %d\tAY: %d\t,AZ: %d\t,GX: %d\t,GY: %d\t,GZ: %d\t\r\n",accelXError,accelZError,ZaccelRaw,gyroXError,gyroYError,gyroZError);

}
uint8_t initMPU6050DMP(void) {
	MPU6050(MPU6050_ADDRESS_AD0_LOW);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    dbgprintf("Initializing I2C devices...\r\n");
    MPUinitialize();

    // verify connection
    dbgprintf("Testing device connections...");
    dbgprintf(MPUtestConnection() ? "MPU6050 connection successful\r\n": "MPU6050 connection failed\r\n");

    // wait for ready
    dbgprintf("\nSend any character to begin DMP programming and demo: \r\n");
     // load and configure the DMP
    dbgprintf("Initializing DMP...");
    uint8_t devStatus = MPUdmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    MPUsetXGyroOffset(220);
    MPUsetYGyroOffset(76);
    MPUsetZGyroOffset(-85);
    MPUsetZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        MPUCalibrateAccel(6);
        MPUCalibrateGyro(6);
         // turn on the DMP, now that it's ready
        dbgprintf("Enabling DMP...\r\n");
        MPUsetDMPEnabled(true);

        uint8_t mpuIntStatus = MPUgetIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dbgprintf("DMP ready! Waiting for first interrupt...:%d\r\n",mpuIntStatus);
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = MPUdmpGetFIFOPacketSize();
        return true;
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
       dbgprintf("DMP Initialization failed (code %d)\r\n",devStatus);
     }

    chSysHalt("MPUSetup() failed");
    return false;
}


static THD_WORKING_AREA(waMPU6050FromDMPThread, MPU_THD_STACK_SIZE);
/*
 * This approach is a port of this arduino code
 * https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/examples/MPU6050_DMP6/MPU6050_DMP6.ino
 * Your MPU allows a sample rate of 8kHz only for the gyrometer, the accelerometer allows only 1kHz.
 * This logic here... is mostly hidden inside the MPU6050, I believe it is based on Kalman filter or
 * some other kind of DSP filter. It is very good and it can calaculates the Yaw angel accurately.
 * However, it gets updated only 100 times per sec. (that's the highest I was able to get,
 * if you change I2C speed to 400k you might get a better performance)For more details, plz visit this blog
 * https://eggelectricunicycle.bitbucket.io/MicroWorks_30B4_board--Datasheets_30B4--MPU6050_freq.html
  */
static THD_FUNCTION(MPU6050FromDMPThread, arg) {(void)arg;
	Quaternion 		q;           // [w, x, y, z]         quaternion container
	VectorFloat 	gravity;
	float 			euler[3];         // [psi, theta, phi]    Euler angle container
	float  			YawPitchRoll[3];
	uint8_t 		devStatus;      // return status after each device operation (0 = success, !0 = error)
	int 			failed = 0;
	float 			eu1,eu2,eu3;
	systime_t   	processingTime 	= 0;
	systime_t   	deadTime 		= 0;
	systime_t   	queryTime 		= 0;
	uint32_t 		calls = 0;
	uint8_t 		fifoBuffer[64]; // FIFO storage buffer
	bool            init            = false;

	chRegSetThreadName("MPU6050FromDMPThread");
//	initMPU6050DMP();
	while (true) {
		chSysLock();
		msg_t reInit = chThdSuspendS((thread_reference_t *)&dmp6050RawThread);
		chSysUnlock();
		if ( currentAlgorithm != MPU605_ALGORITHM_DMP)
			continue;
		if (!init || reInit){
			dbgprintf("Re-initializing DMP\r\n");
			initMPU6050DMP();
			init = true;
			continue;
		}
		bool 		log 	= calls % 100 == 0 && sendMsgsToMQTT;
		systime_t   start   = chVTGetSystemTimeX();
		++calls;
		int8_t rc = MPUGetCurrentFIFOPacket(fifoBuffer, packetSize);
		queryTime += chTimeI2US( chTimeDiffX(start,chVTGetSystemTimeX()));
		if (i2cGetErrors(&MPU_I2CD) ){
			dbgprintf("Restarting I2C & MPU6050FromDMP:%d %d\r\n",calls,failed++);
			initMPU6050DMP();
			continue;
		}
		if ( rc ){
			  MPUresetFIFO();
			  MPUdmpGetQuaternion(&q, fifoBuffer);
			  MPUdmpGetGravityVect(&gravity,&q);
			  MPUdmpGetEuler(euler, &q);
			  MPUdmpGetYawPitchRoll(YawPitchRoll, &q, &gravity);
			  eu1 = (YawPitchRoll[0] * RAD_TO_DEG);
			  eu2 = (YawPitchRoll[1] * RAD_TO_DEG);
			  eu3 = (YawPitchRoll[2] * RAD_TO_DEG);
			  #if S4E_USE_ETHERNET == 1
			  if ( log ){
				  chsnprintf(payload,sizeof(payload),"{\"algorithm\":\"%s\",\"Pitch\":%.2f,\"Roll\":%2f,\"Yaw\":%2f}\r\n", DMP_ALGPRITHM, eu3, eu2, eu1);
				  sendToDefaultMQTTQueue(payload);
			  }
			  #endif
			  /*
			  ** This is for the teapot demo, see here for details
			  ** https://www.youtube.com/watch?v=kyX9cRxJNdo
			  ** you need the processing Java module to work
			   */
			  teapotPacket[2] = fifoBuffer[0];
			  teapotPacket[3] = fifoBuffer[1];
			  teapotPacket[4] = fifoBuffer[4];
			  teapotPacket[5] = fifoBuffer[5];
			  teapotPacket[6] = fifoBuffer[8];
			  teapotPacket[7] = fifoBuffer[9];
			  teapotPacket[8] = fifoBuffer[12];
			  teapotPacket[9] = fifoBuffer[13];
			  for(int j = 0; j < 14 && log; ++j)
				  dbgprintf("%c",teapotPacket[j]);
			  teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
		}
		if( calls % 1000 == 0 )
			  dbgprintf("Processed:%d\tCalls:%d\tFailed:%d\tYPR:#:X:%.2f Y:%.2f-Z:%.2f\tavg:%d us\tDeadTime:%d\tQueryTime\t%d\r\n", processed ,calls, failed, eu3, eu2, eu1,processingTime/calls,deadTime/calls,queryTime/calls);
	}
}
static THD_WORKING_AREA(waMPU6050FromRawDataThread, MPU_THD_STACK_SIZE);
/*
 * The logic was borrowed from
 * http://www.brokking.net/imu.html
 * https://www.youtube.com/watch?v=j-kE0AMEWy4
 * https://www.youtube.com/watch?v=XCyRXMvVSCw
 * https://www.youtube.com/watch?v=M9lZ5Qy5S2s
 * https://www.researchgate.net/post/Who_have_used_the_Arduino_and_mpu6050_Can_you_tell_me_how_to_set_the_sample_rate_for_mpu6050
 * This algorithm doesn't calaculate the Yaw angel accuretly, however the pitch and roll angels
 * are updated at a rate of 250 times per sec. DMP alogorth is at 100Hz
 * here you will find an excellent lecture on PID controllers for quad copters
 * https://www.youtube.com/watch?v=CHSYgLfhwUo
 */
#define GYRO_PART 0.995
#define ACC_PART  0.005
float 		angle_pitch_output=0, angle_roll_output=0, angle_pitch =0,    angle_roll      =0,
			angle_pitch_acc   =0, angle_roll_acc   =0, angle_yaw   =0,    acc_total_vector=0,  temp= 0;

void mpu6050FormatAngles(char *buff, uint8_t size){
	 chsnprintf(buff, size,"%.1f %.1f %.1f %.1f", angle_pitch_output,angle_roll_output,angle_pitch,temp);

}
static THD_FUNCTION(MPU6050FromRawDataThread, arg) {(void)arg;
	bool 		setGyroAngles     = false;
	uint32_t 	avgTime=0;
	/*
	 ** 1 / (250Hz /65.5) ..we are using MPU6050_GYRO_FS_500=>65.5.
	 ** 250 Hz= .004 sec. is the speed of the loop
	 ** See page 12 for details https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
	 */
	float 		factor1 = (4 * 0.001) / 65.5;
	float 		factor2 = factor1 * M_PI /180.0;
	bool 		init = false;
	chRegSetThreadName("MPU6050FromRawDataThread");

	uint32_t 		i = 0;
	sysinterval_t 	total=0, delta=0;
	systime_t     	end = 0;
	while (TRUE){
		//Suspend if we are not the current algorithm or if we are ordered to pause
		if ( currentAlgorithm != MPU605_ALGORITHM_RAW || !sendMsgsToMQTT ){
			dbgprintf("Suspending MPU6050FromRawDataThread\r\n");
			chSysLock();
			msg_t reinit = chThdSuspendS((thread_reference_t *)&mpu6050RawThread);
			chSysUnlock();
			dbgprintf("MPU6050FromRawDataThread wake up\r\n");
			init = reinit?false:true;
		}
		if ( !init || i2cGetErrors(&MPU_I2CD) ){
			initMPU6050Raw();
			calibrate(200);
			init = true;
			dbgprintf("reinitializing MPU6050\r\n");
		}
		systime_t   start    = chVTGetSystemTimeX();
		++i;
		bool log = i % 1000 == 0 && sendMsgsToMQTT;
		if ( MPUgetMotion6WithTemp(&XaccelRaw, &YaccelRaw, &ZaccelRaw, &XgyroRaw, &YgyroRaw, &ZgyroRaw, &tempRaw) ){
			systime_t   startCPU    = chVTGetSystemTimeX();
			int 		blockedFor = chTimeI2US(chTimeDiffX(start,startCPU));

			XaccelRaw = XaccelRaw - accelXError;
			YaccelRaw = YaccelRaw - accelYError;
			ZaccelRaw = ZaccelRaw - accelXError;
			XgyroRaw  = XgyroRaw  - gyroXError;
			YgyroRaw  = YgyroRaw  - gyroYError;
			ZgyroRaw  = ZgyroRaw  - gyroZError;
			temp= (tempRaw + 12412.0) / 340.0;

			angle_pitch += XgyroRaw * factor1;     //Calculate the traveled pitch angle and add this to the angle_pitch variable
			angle_roll  += YgyroRaw * factor1;    //Calculate the traveled roll angle and add this to the angle_roll variable
			angle_yaw   += ZgyroRaw * factor1;

			angle_pitch += angle_roll  * sin(ZgyroRaw * factor2);               //If the IMU has yawed transfer the roll angle to the pitch angel
			angle_roll  -= angle_pitch * sin(ZgyroRaw * factor2);               //If the IMU has yawed transfer the pitch angle to the roll angel
			//Accelerometer angle calculations
			acc_total_vector = sqrt((XaccelRaw*XaccelRaw)+(YaccelRaw*YaccelRaw)+(ZaccelRaw*ZaccelRaw));  //Calculate the total accelerometer vector
			//57.296 = 180 / M_PI
			angle_pitch_acc = asin((float)YaccelRaw / acc_total_vector) *  RAD_TO_DEG;       //Calculate the pitch angle
			angle_roll_acc  = asin((float)XaccelRaw/ acc_total_vector)  * -RAD_TO_DEG;       //Calculate the roll angle

			//Place the MPU-6050 spirit level and note the values in the following two lines for calibration
			angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
			angle_roll_acc  -= 0.0;                                               //Accelerometer calibration value for roll
			if ( setGyroAngles ){                                                 //If the IMU is already started
				angle_pitch = angle_pitch * GYRO_PART + angle_pitch_acc * ACC_PART;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
				angle_roll  = angle_roll * GYRO_PART  + angle_roll_acc  * ACC_PART;     //Correct the drift of the gyro roll angle with the accelerometer roll angle
			}
			else{                                                               //At first start
				angle_pitch   = angle_pitch_acc;                                //Set the gyro pitch angle equal to the accelerometer pitch angle
				angle_roll    = angle_roll_acc;                                 //Set the gyro roll angle equal to the accelerometer roll angle
				setGyroAngles = true;                                           //Set the IMU started flag
			}

			// retr3 = timer.get();
			//To dampen the pitch and roll angles a complementary filter is used
			angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
			angle_roll_output  = angle_roll_output *  0.9 + angle_roll  * 0.1;    //Take 90% of the output roll value and add 10% of the raw roll value

			end   = chVTGetSystemTimeX();
			delta = chTimeDiffX(start,end);
			total +=  delta ;
			if ( log ){
                avgTime = chTimeI2US(total) / i ;
                int 		cpuSpent = chTimeI2US(chTimeDiffX(startCPU,chVTGetSystemTimeX()));
				#if S4E_USE_ETHERNET == 1
                chsnprintf(payload,sizeof(payload),"{\"algorithm\":\"%s\",\"iteration\":%d,\"XA\": %.2f,\"YA\":%.2f,\"Pitch\":%.2f,\"Roll\":%.2f,\"Yaw\":%.2f,\"Temp\":%.2f,\"AvgTime\": \"%d|%d|%d\"}",
                								   RAW_ALGPRITHM,i, angle_pitch_acc, angle_roll_acc, angle_pitch_output,angle_roll_output,angle_pitch,temp,avgTime,blockedFor,cpuSpent);
                sendToDefaultMQTTQueue(payload);
				#endif
                dbgprintf("i:%d\tXA: %.2f\t,YA: %.2f\tPitch: %.2f\tRoll: %.2f\tYaw:%.2f\tTemp:%.2f\tAvgTime: %d\r\n", i, angle_pitch_acc, angle_roll_acc, angle_pitch_output,angle_roll_output,angle_pitch,temp,avgTime);
                pRedLedPAL->toggle(pRedLedPAL);
                updateScreen();
			}
		}
		else
			dbgprintf("I2C Failed\r\n");
		int toSleep = 4000-chTimeDiffX(start,chVTGetSystemTimeX());
		if ( toSleep > 0){
			chThdSleepMicroseconds(toSleep);
		}
	}

  return;
}

static void cbHandler(void *arg) {(void)arg;
	if (  currentAlgorithm == MPU605_ALGORITHM_DMP && sendMsgsToMQTT){
		chSysLockFromISR();
		chThdResumeI((thread_reference_t*)&dmp6050RawThread,0);
		chSysUnlockFromISR();
		++processed;
	}
}


void initMPU6050Thread(void){
	/*
	 * Port dependencies for interrupts and for I2C
	 * From top to down line coloring: White, brown, black, red.
	 * Green on D2 for interrupt
	 */
	palSetLineMode(LINE_ARD_D2,BUTTON_MODE2);
	palEnableLineEvent(LINE_ARD_D2, PAL_EVENT_MODE_RISING_EDGE);
	palSetLineCallback(LINE_ARD_D2, cbHandler, NULL);

	//Note that I'm using the internal pullup resistors. You can get higher speed by using external resistors
	palSetLineMode(MPU_SCL_PIN, PAL_MODE_ALTERNATE(MPU_I2C_AF) |  PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_HIGHEST|PAL_STM32_PUPDR_PULLUP);//D15-SCL
	palSetLineMode(MPU_SDA_PIN, PAL_MODE_ALTERNATE(MPU_I2C_AF) |  PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_HIGHEST|PAL_STM32_PUPDR_PULLUP);//D14-SDA
	i2cStart(&MPU_I2CD, &i2ccfgLow2);
	chThdSleepMilliseconds(100);

	chThdCreateStatic(waMPU6050FromDMPThread,     sizeof(waMPU6050FromDMPThread),     NORMALPRIO + 11, MPU6050FromDMPThread, NULL);
	chThdCreateStatic(waMPU6050FromRawDataThread, sizeof(waMPU6050FromRawDataThread), NORMALPRIO + 11, MPU6050FromRawDataThread, NULL);
}
void enableSleepModeMPU6050(bool enable){
	MPUsetSleepEnabled(enable);
}
