/*
 * MPU6050Thread.h
 *
 *  Created on: Apr 11, 2020
 *      Author: abusous2000
 */

#ifndef SOURCE_MPU6050_MPU6050THREAD_H_
#define SOURCE_MPU6050_MPU6050THREAD_H_

#include "Strust4EmbeddedConf.h"
#define MPU_THD_STACK_SIZE      1024
#define RAW_ALGPRITHM			"RAW"
#define DMP_ALGPRITHM			"DMP"
#define RAD_TO_DEG 				57.2957786
#ifdef __cplusplus
 extern "C" {
#endif
 enum MPU605_ALGORITHM {
	 MPU605_ALGORITHM_RAW           = 0,
	 MPU605_ALGORITHM_DMP           = 1
 };
void initMPU6050Thread(void);
void enableSleepModeMPU6050(bool enable);
#ifdef __cplusplus
 }
#endif
#endif /* SOURCE_MPU6050_MPU6050THREAD_H_ */
