/*! 
 *  @file PropertyDefinition.h
 *  @brief header for property definition
 *  @author Sunhong Kim (tjsghd101@naver.com)
 *  @data Oct. 26. 2023
 *  @Comm
 */


#pragma once

#include <Eigen/Dense>

#define CONTROL_FREQ 1000
#define ROBOT_DOF 4
#define OFFSET_NUM 1
#define MOBILE_DRIVE_NUM 4
#define MOBILE_DOF_NUM 3
#define NRMK_DRIVE_NUM 6
#define NRMK_TOOL_NUM 1
#define MM_DOF_NUM MOBILE_DOF_NUM + NRMK_DRIVE_NUM

// [General Parameters]
// 1. Motor
#define TORQUE_CONST_500 0.0884 		
#define TORQUE_CONST_200 0.087		
#define TORQUE_CONST_100 0.058		

#define TORQUE_ADC_500 48 // Torque ADC for Core 500 [NRMK]
#define TORQUE_ADC_200 96 // Torque ADC for Core 200 [NRMK]
#define TORQUE_ADC_100 96 // Torque ADC for Core 100 [NRMK]

// Indy7
// SN: P11828I07003 [Indy7]
#define ZERO_POS_1 48364
#define ZERO_POS_2 5992
#define ZERO_POS_3 -28718
#define ZERO_POS_4 4754
#define ZERO_POS_5 29969
#define ZERO_POS_6 -3031

#define F_c 20.123, 12.287, 4.5622, 3.1492, 3.4757, 3.4986
#define F_v1 111.32, 70.081, 25.337, 13.131, 8.5082, 9.9523
#define F_v2 0.5193, 0.4824, 0.9098, 1.0961, 0.73829, 1.1475 

#define MAX_CURRENT_1 2.55
#define MAX_CURRENT_2 2.55
#define MAX_CURRENT_3 2.83
#define MAX_CURRENT_4 2.83
#define MAX_CURRENT_5 2.83
#define MAX_CURRENT_6 2.83

#define MAX_TORQUE_1 431.97
#define MAX_TORQUE_2 431.97
#define MAX_TORQUE_3 197.23
#define MAX_TORQUE_4 79.79
#define MAX_TORQUE_5 79.79
#define MAX_TORQUE_6 79.79

#define invL2sqr_1 1000
#define invL2sqr_2 1000
#define invL2sqr_3 800
#define invL2sqr_4 600
#define invL2sqr_5 600
#define invL2sqr_6 600


// 2. Electrical
#define ENC_2048 2048
#define ENC_1024 1024
#define ENC_1000 1000
#define ENC_512 512
#define ABS_ENC_19 524288

#define ENC_CORE_500 65536
#define ENC_CORE_200 65536
#define ENC_CORE_100 65536
#define ENC_CORE 65536

// 3. Mechanical
#define HARMONIC_120 120
#define HARMONIC_100 100
#define HARMONIC_50 50

#define GEAR_RATIO_121 121
#define GEAR_RATIO_101 101
#define GEAR_RATIO_50 50
#define GEAR_RATIO_18 18

#define WHEEL_RADIUS 0.076 // [m]
#define BASE_l 0.41 // [m]
#define BASE_w 0.31 // [m]

#define EFFICIENCY 85.0 // Gear efficiency

// Tool Information [Tip]
#define Ixx 0.00012118
#define Iyy 0.00012118
#define Izz 0.00000861
#define X_com 0.0
#define Y_com 0.0
#define Z_com 0.0355073
#define mass_tool 0.025
// Tool Information [FT Sensor]
#define mass_FT 0.02

// FT Sensor Commands
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

// 4. Type Definition
typedef Eigen::Matrix<double, 4, 4> SE3;
typedef Eigen::Matrix<double, 3, 3> SO3;
typedef Eigen::Matrix<double, 4, 4> se3;
typedef Eigen::Matrix<double, 3, 3> so3;
typedef Eigen::Matrix<double, 6, 1> Twist;
typedef Eigen::Matrix<double, 6, 1> Vector6d;   
typedef Eigen::Matrix<float, 6, 1> Vector6f;  
typedef Eigen::Matrix<double, 3, 1> Vector3d;   
typedef Eigen::Matrix<double, 4, 1> Vector4d;  
typedef Eigen::Matrix<double, 6, 6> Matrix6d;  
typedef Eigen::Matrix<double, 3, 3> Matrix3d;  

typedef Eigen::Matrix<double, MOBILE_DRIVE_NUM, 1> Mob_JVec;
typedef Eigen::Matrix<double, MOBILE_DRIVE_NUM, MOBILE_DRIVE_NUM> Mob_JMat;
typedef Eigen::Matrix<double, 3, MOBILE_DRIVE_NUM> Mob_Jacobian;
typedef Eigen::Matrix<double, MOBILE_DRIVE_NUM,3 > Mob_pinvJacobian;
typedef Eigen::Matrix<double, 3*MOBILE_DRIVE_NUM, MOBILE_DRIVE_NUM> Mob_DerivativeJacobianVec;
typedef Eigen::Matrix<double, 3*MOBILE_DRIVE_NUM, 1> Mob_vecJVec;
typedef Eigen::Matrix<double, MOBILE_DRIVE_NUM, MOBILE_DRIVE_NUM> Mob_Matrixnd;  
typedef Eigen::Matrix<double, 6, MOBILE_DRIVE_NUM> Mob_Matrix6xn;
typedef Eigen::Matrix<double, 6, MOBILE_DRIVE_NUM+1> Mob_Matrix6xn_1;
typedef Eigen::Matrix<double, MOBILE_DRIVE_NUM, MOBILE_DRIVE_NUM> Mob_MassMat;

typedef Eigen::Matrix<double, NRMK_DRIVE_NUM, 1> Arm_JVec;
typedef Eigen::Matrix<double, NRMK_DRIVE_NUM, NRMK_DRIVE_NUM> Arm_JMat;
typedef Eigen::Matrix<double, 6, NRMK_DRIVE_NUM> Arm_ScrewList;
typedef Eigen::Matrix<double, 6, NRMK_DRIVE_NUM> Arm_Jacobian;
typedef Eigen::Matrix<double, NRMK_DRIVE_NUM,6 > Arm_pinvJacobian;
typedef Eigen::Matrix<double, 6*NRMK_DRIVE_NUM, NRMK_DRIVE_NUM> Arm_DerivativeJacobianVec;
typedef Eigen::Matrix<double, 6*NRMK_DRIVE_NUM, 1> Arm_vecJVec;
typedef Eigen::Matrix<double, NRMK_DRIVE_NUM, NRMK_DRIVE_NUM> Arm_Matrixnd;  
typedef Eigen::Matrix<double, 6, NRMK_DRIVE_NUM> Arm_Matrix6xn;
typedef Eigen::Matrix<double, 6, NRMK_DRIVE_NUM+1> Arm_Matrix6xn_1;
typedef Eigen::Matrix<double, NRMK_DRIVE_NUM, NRMK_DRIVE_NUM> Arm_MassMat;

typedef Eigen::Matrix<double, MOBILE_DOF_NUM+NRMK_DRIVE_NUM, 1> MM_JVec;
typedef Eigen::Matrix<double, MOBILE_DOF_NUM+NRMK_DRIVE_NUM, MOBILE_DOF_NUM+NRMK_DRIVE_NUM> MM_JMat;
typedef Eigen::Matrix<double, 6, MOBILE_DOF_NUM+NRMK_DRIVE_NUM> MM_ScrewList;
typedef Eigen::Matrix<double, 6, MOBILE_DOF_NUM+NRMK_DRIVE_NUM> MM_Jacobian;
typedef Eigen::Matrix<double, 3, MOBILE_DOF_NUM+NRMK_DRIVE_NUM> MM_Jacobian_CoM;
typedef Eigen::Matrix<double, MOBILE_DOF_NUM+NRMK_DRIVE_NUM,6 > MM_pinvJacobian;
typedef Eigen::Matrix<double, 6*(MOBILE_DOF_NUM+NRMK_DRIVE_NUM), MOBILE_DOF_NUM+NRMK_DRIVE_NUM> MM_DerivativeJacobianVec;
typedef Eigen::Matrix<double, 6*(MOBILE_DOF_NUM+NRMK_DRIVE_NUM), 1> MM_vecJVec;
typedef Eigen::Matrix<double, MOBILE_DOF_NUM+NRMK_DRIVE_NUM, MOBILE_DOF_NUM+NRMK_DRIVE_NUM> MM_Matrixnd;  
typedef Eigen::Matrix<double, 6, MOBILE_DOF_NUM+NRMK_DRIVE_NUM> MM_Matrix6xn;
typedef Eigen::Matrix<double, 6, MOBILE_DOF_NUM+NRMK_DRIVE_NUM+1> MM_Matrix6xn_1;
typedef Eigen::Matrix<double, MOBILE_DOF_NUM+NRMK_DRIVE_NUM, MOBILE_DOF_NUM+NRMK_DRIVE_NUM> MM_MassMat;

// Robot Struct
// Mobile
typedef struct MOB_STATE{
	Mob_JVec q;
	Mob_JVec q_dot;
	Mob_JVec q_ddot;
	Mob_JVec tau;
	Mob_JVec tau_ext;
	Mob_JVec e;
	Mob_JVec eint;
	Mob_JVec edot;
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

// Mobile Manipulator
typedef struct MM_STATE{
	MM_JVec q;			// x, y, th, q1, q2, q3, q4, q5, q6
	MM_JVec q_dot;		// vx, vy, wz, dq1, dq2, dq3, dq4, dq5, dq6
	MM_JVec q_ddot;
	MM_JVec tau;		// Fx, Fy, Tz, tau1, tau2, tau3, tau4, tau5, tau6
	MM_JVec tau_fric;
	MM_JVec tau_ext;
	MM_JVec tau_aux;
	MM_JVec e;
	MM_JVec eint;
	MM_JVec edot;

	SE3 	 T;                           //Task space
	SO3		 R;
	Vector6d x_dot;
	Vector6d x_ddot;
	Vector6d F;
	Vector6d F_CB;
    Vector6d F_ext;

	Vector3d CoM_x;
	MM_Jacobian_CoM J_com;
    
    double s_time;
}mm_state;

typedef struct MM_ROBOT_INFO{

	MM_JVec q_target;
	MM_JVec qdot_target;
	MM_JVec qddot_target;
	MM_JVec traj_time;
	unsigned int idx;

	MM_STATE act;
	MM_STATE des;
	MM_STATE nom;
	MM_STATE sim;

}MM_ROBOT_INFO;
