#ifndef RTECAT_MOBILEMANIPULATOR_CLIENT_H_
#define RTECAT_MOBILEMANIPULATOR_CLIENT_H_

#include <stdio.h>
#include "iostream"
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <queue>
#include <sys/mman.h>

#include <sched.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <rtdm/ipc.h> 

#include "CS_Indy7.h"
#include "CS_hyumm.h"

#undef debug
//QT
// #include <thread>
// #include <pthread.h>
#include <QApplication>
#include <QSplitter>
#include <QTreeView>
#include <QListView>
#include <QTableView>
#include <QStandardItemModel>
#include <QScreen>

#include "DarkStyle.h"
#include "framelesswindow.h"
#include "mainwindow.h"
#include <iostream>

#include <QApplication>
#include <QResource>
#include <QTextCodec>

#include "Ecat_Master.h"
#include "ServoAxis_Motor.h"
#include "ServoAxis_Core.h"
#include <PropertyDefinition.h>
#include <liegroup_robotics.h>

#include "SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
#include "SharedMemory/PhysicsClientSharedMemory_C_API.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3HashMap.h"

#include "Utils/b3Clock.h"
#include <map>
#include <vector>
#include "bullet_hyumm.h"
unsigned long periodBullet = 0;

#define XDDP_PORT 0	/* [0..CONFIG-XENO_OPT_PIPE_NRDEV - 1] */

#define NSEC_PER_SEC 			1000000000
unsigned int cycle_ns = 1000000; // 1 ms
double period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit



double ft_offset[6] = {30.1, -54.8, -45.4, -1.28, -0.233, 4.295};
// double ft_offset[6] = {0.0,};
Twist F_tmp;

// For RT thread management
static int run = 1;

unsigned long periodCycle = 0, worstCycle = 0;
unsigned long periodLoop = 0, worstLoop = 0;
unsigned int overruns = 0;

double act_max[4] = {0.0,};

// Interface to physical axes
// Mobile
NRMKHelper::ServoAxis_Motor Axis_Motor[MOBILE_DRIVE_NUM];
const int 	 zeroPos_mob[MOBILE_DRIVE_NUM] = {0,0,0,0};
const INT32 	 gearRatio_mob[MOBILE_DRIVE_NUM] = {GEAR_RATIO_18,GEAR_RATIO_18,GEAR_RATIO_18,GEAR_RATIO_18};
const int 	 dirQ_mob[MOBILE_DRIVE_NUM] = {1,1,-1,-1};
const int 	 dirTau_mob[MOBILE_DRIVE_NUM] = {1,1,-1,-1};
// Manipulator
NRMKHelper::ServoAxis_Core Axis_Core[NRMK_DRIVE_NUM];
const int 	 zeroPos_arm[NRMK_DRIVE_NUM] = {ZERO_POS_1,ZERO_POS_2,ZERO_POS_3,ZERO_POS_4,ZERO_POS_5,ZERO_POS_6};
const int 	 gearRatio_arm[NRMK_DRIVE_NUM] = {GEAR_RATIO_121,GEAR_RATIO_121,GEAR_RATIO_121,GEAR_RATIO_101,GEAR_RATIO_101,GEAR_RATIO_101};
const int 	 TauADC_arm[NRMK_DRIVE_NUM] = {TORQUE_ADC_500,TORQUE_ADC_500,TORQUE_ADC_200,TORQUE_ADC_100,TORQUE_ADC_100,TORQUE_ADC_100};
const double TauK_arm[NRMK_DRIVE_NUM] = {TORQUE_CONST_500,TORQUE_CONST_500,TORQUE_CONST_200,TORQUE_CONST_100,TORQUE_CONST_100,TORQUE_CONST_100};
const int 	 dirQ_arm[NRMK_DRIVE_NUM] = {-1,-1,1,-1,-1,-1};
const int 	 dirTau_arm[NRMK_DRIVE_NUM] = {-1,-1,1,-1,-1,-1};
const double qdotLimit[NRMK_DRIVE_NUM] = {2*PI, 2*PI, 2*PI, 2*PI, 2*PI, 3*PI};
NRMKHelper::ServoAxis_Motor Axis_MM[MM_DOF_NUM];

// Robotous FT EtherCAT
union DeviceConfig
{
	uint8_t u8Param[4];
	uint32_t u32Param;
};
double force_divider =50.0;
double torque_divider = 2000.0;

// EtherCAT System interface object
Master ecat_master;
Ecat_iServo ecat_iservo[MOBILE_DRIVE_NUM];
EcatNRMK_Drive ecat_drive[NRMK_DRIVE_NUM];
EcatNRMK_Tool ecat_tool[NRMK_TOOL_NUM];

// When all slaves or drives reach OP mode,
// system_ready becomes 1.
int system_ready = 0;

// Global time (beginning from zero)
double gt=0;
double gt_offset = 0;

// Trajectory parameers
double traj_time=0;
int motion=-1;
int modeControl = 0;
int motioncnt = 0;

// Controller Gains
MM_JVec NRIC_Kp_mm;
MM_JVec NRIC_Ki_mm;
MM_JVec NRIC_K_gamma_mm;

MM_JVec Kp_n_mm;
MM_JVec Kd_n_mm;
MM_JVec Ki_n_mm;

Twist Task_Kp;
Twist Task_Kv;
Twist Task_Ki;
MM_JVec Task_K;

// Mobile Jacobian
Mob_pinvJacobian Jinv_mob;
Mob_Jacobian J_mob;


extern const int CONTROL_RATE;
const int CONTROL_RATE = 100;

// Bullet globals
const b3Scalar FIXED_TIMESTEP = 1.0 / ((b3Scalar)CONTROL_RATE);
b3SharedMemoryCommandHandle command;
int statusType, ret;



#endif  // /* RTECAT_MOBILEMANIPULATOR_CLIENT_H */