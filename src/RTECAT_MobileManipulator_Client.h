#ifndef RTECAT_MOBILEMANIPULATOR_CLIENT_H_
#define RTECAT_MOBILEMANIPULATOR_CLIENT_H_

#include <stdio.h>
#include "iostream"
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
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

#define XDDP_PORT 0	/* [0..CONFIG-XENO_OPT_PIPE_NRDEV - 1] */

#define NSEC_PER_SEC 			1000000000
unsigned int cycle_ns = 1000000; // 2 ms
double period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit

#define FT_START_DEVICE 	0x0000000B
#define FT_STOP_DEVICE 		0x0000000C

#define FT_SET_FILTER_500 	0x00010108
#define FT_SET_FILTER_300 	0x00020108
#define FT_SET_FILTER_200 	0x00030108
#define FT_SET_FILTER_150 	0x00040108
#define FT_SET_FILTER_100 	0x00050108
#define FT_SET_FILTER_50 	0x00060108
#define FT_SET_FILTER_40 	0x00070108
#define FT_SET_FILTER_30 	0x00080108
#define FT_SET_FILTER_20 	0x00090108
#define FT_SET_FILTER_10 	0x000A0108

#define FT_SET_BIAS 		0x00000111
#define FT_UNSET_BIAS 		0x00000011

// double ft_offset[6] = {30.25, -51.20, -51.40, -1.084, -0.011, 4.31};
double ft_offset[6] = {0.0,};

// For RT thread management
static int run = 1;

unsigned long periodCycle = 0, worstCycle = 0;
unsigned int overruns = 0;

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

int offset_junction = 1;

// When all slaves or drives reach OP mode,
// system_ready becomes 1.
int system_ready = 0;

// Global time (beginning from zero)
double gt=0;

// Trajectory parameers
double traj_time=0;
int motion=-1;

// Mobile
typedef struct MOB_STATE{
	Mob_JVec q;
	Mob_JVec q_dot;
	Mob_JVec q_ddot;
	Mob_JVec tau;
	Mob_JVec tau_ext;
	Mob_JVec e;
	Mob_JVec eint;
	Mob_JVec G;

	Vector3d x;                           //Task space
	Vector3d x_dot;
	Vector3d x_ddot;
	Vector3d F;
	Vector3d F_CB;
    Vector3d F_ext;
    
    double s_time;
}mob_state;

typedef struct MOB_MOTOR_INFO{
    double torque_const[MOBILE_DRIVE_NUM];
    double gear_ratio[MOBILE_DRIVE_NUM];
    double rate_current[MOBILE_DRIVE_NUM];
}mob_Motor_Info;

typedef struct MOB_ROBOT_INFO{
	int Position;
	int q_inc[MOBILE_DRIVE_NUM];
    int dq_inc[MOBILE_DRIVE_NUM];
	int tau_per[MOBILE_DRIVE_NUM];
	int statusword[MOBILE_DRIVE_NUM];
    int modeofop[MOBILE_DRIVE_NUM];

	Mob_JVec q_target;
	Mob_JVec qdot_target;
	Mob_JVec qddot_target;
	Mob_JVec traj_time;
	unsigned int idx;

	MOB_STATE act;
	MOB_STATE des;
	MOB_STATE nom;

    MOB_MOTOR_INFO motor;
}MOB_ROBOT_INFO;

// Arm
typedef struct ARM_STATE{
	Arm_JVec q;
	Arm_JVec q_dot;
	Arm_JVec q_ddot;
	Arm_JVec tau;
	Arm_JVec tau_fric;
	Arm_JVec tau_ext;
	Arm_JVec tau_aux;
	Arm_JVec e;
	Arm_JVec eint;
	Arm_JVec G;

	Vector6d x;                           //Task space
	Vector6d x_dot;
	Vector6d x_ddot;
	Vector6d F;
	Vector6d F_CB;
    Vector6d F_ext;
    
    double s_time;
}arm_state;

typedef struct ARM_MOTOR_INFO{
    double torque_const[NRMK_DRIVE_NUM];
    double gear_ratio[NRMK_DRIVE_NUM];
    double rate_current[NRMK_DRIVE_NUM];
}arm_Motor_Info;

typedef struct ARM_ROBOT_INFO{
	int Position;
	int q_inc[NRMK_DRIVE_NUM];
    int dq_inc[NRMK_DRIVE_NUM];
	int tau_per[NRMK_DRIVE_NUM];
	int statusword[NRMK_DRIVE_NUM];
    int modeofop[NRMK_DRIVE_NUM];

	Arm_JVec q_target;
	Arm_JVec qdot_target;
	Arm_JVec qddot_target;
	Arm_JVec traj_time;
	unsigned int idx;

	ARM_STATE act;
	ARM_STATE des;
	ARM_STATE nom;
	ARM_STATE sim;

    ARM_MOTOR_INFO motor;
}ARM_ROBOT_INFO;

// MM
typedef struct MM_STATE{
	MM_JVec q;
	MM_JVec q_dot;
	MM_JVec q_ddot;
	MM_JVec tau;
	MM_JVec tau_fric;
	MM_JVec tau_ext;
	MM_JVec tau_aux;
	MM_JVec e;
	MM_JVec eint;
	MM_JVec G;

	Vector6d x;                           //Task space
	Vector6d x_dot;
	Vector6d x_ddot;
	Vector6d F;
	Vector6d F_CB;
    Vector6d F_ext;
    
    double s_time;
}mm_state;

typedef struct MM_MOTOR_INFO{
    double torque_const[NRMK_DRIVE_NUM];
    double gear_ratio[NRMK_DRIVE_NUM];
    double rate_current[NRMK_DRIVE_NUM];
}mm_Motor_Info;

typedef struct MM_ROBOT_INFO{
	int Position;
	int q_inc[NRMK_DRIVE_NUM];
    int dq_inc[NRMK_DRIVE_NUM];
	int tau_per[NRMK_DRIVE_NUM];
	int statusword[NRMK_DRIVE_NUM];
    int modeofop[NRMK_DRIVE_NUM];

	MM_JVec q_target;
	MM_JVec qdot_target;
	MM_JVec qddot_target;
	MM_JVec traj_time;
	unsigned int idx;

	MM_STATE act;
	MM_STATE des;
	MM_STATE nom;
	MM_STATE sim;

    MM_MOTOR_INFO motor;
}MM_ROBOT_INFO;

// Controller Gains
Arm_JVec NRIC_Kp;
Arm_JVec NRIC_Ki;
Arm_JVec NRIC_K_gamma;

Arm_JVec Kp_n;
Arm_JVec Kd_n;
Arm_JVec Ki_n;

// Mobile Jacobian
Mob_pinvJacobian Jinv_mob;
Mob_Jacobian J_mob;

#endif  // /* RTECAT_MOBILEMANIPULATOR_CLIENT_H */