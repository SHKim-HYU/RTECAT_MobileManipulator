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
// SN: DD11I7ED001 [Indy7]
#define ZERO_POS_1 20902
#define ZERO_POS_2 28201
#define ZERO_POS_3 292649
#define ZERO_POS_4 61158
#define ZERO_POS_5 2180
#define ZERO_POS_6 31587

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

#define EFFICIENCY 75.0 // Gear efficiency

// Tool Information [Tip]
#define Ixx 0.00012118
#define Iyy 0.00012118
#define Izz 0.00000861
#define X_com 0.0
#define Y_com 0.0
#define Z_com 0.0355073
#define mass_tool 0.06
// Tool Information [FT Sensor]
#define mass_FT 0.02

// 4. Type Definition
typedef Eigen::Matrix<double, 4, 4> SE3;
typedef Eigen::Matrix<double, 3, 3> SO3;
typedef Eigen::Matrix<double, 6, 1> se3;
typedef Eigen::Matrix<double, 3, 3> so3;
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