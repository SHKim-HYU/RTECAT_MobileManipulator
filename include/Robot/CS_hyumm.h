#ifndef CS_HYUMM_H
#define CS_HYUMM_H

#include "iostream"
#include "json_loader.h"
// #include "modern_robotics.h"
#include <Eigen/Dense>
// #include <casadi/casadi.hpp>
#include <dlfcn.h>

#include "liegroup_robotics.h"
#include "PropertyDefinition.h"

typedef long long int casadi_int;
typedef int (*eval_t)(const double**, double**, casadi_int*, double*, int);

using namespace Eigen;
using namespace std;
using namespace lr;

class CS_hyumm  {
public:
	CS_hyumm();


	void CSSetup(const string& _modelPath, double _period);
	void setPIDgain(MM_JVec _Kp, MM_JVec _Kd, MM_JVec _Ki);
	void setHinfgain(MM_JVec _Hinf_Kp, MM_JVec _Hinf_Kd, MM_JVec _Hinf_Ki, MM_JVec _Hinf_K_gamma);
	void setNRICgain(MM_JVec _NRIC_Kp, MM_JVec _NRIC_Ki, MM_JVec _NRIC_K, MM_JVec _NRIC_gamma);
	void setTaskgain(Twist _Kp, Twist _Kv, MM_JVec _K);
	void setTaskImpedancegain(Matrix6d _Kp, Matrix6d _Kv, Matrix6d _Kgamma);

	void updateRobot(MM_JVec _q, MM_JVec _dq);

	MM_JVec addLoadedTorque(MM_JVec _q, Vector3d _com, double _m);

	MM_JVec computeFD(MM_JVec _q, MM_JVec _dq, MM_JVec _tau);
	void computeRK45(MM_JVec _q, MM_JVec _dq, MM_JVec _tau, MM_JVec &_q_nom, MM_JVec &_dq_nom, MM_JVec &_ddq_nom);

	Twist computeF_Tool(Twist _dx, Twist _ddx);
	Twist computeF_Threshold(Twist _F);

	MM_MassMat computeM(MM_JVec _q);
	MM_MassMat computeMinv(MM_JVec _q);
	MM_MassMat computeC(MM_JVec _q, MM_JVec _dq);
	MM_JVec computeG(MM_JVec _q);

	Vector3d computeCoM(MM_JVec _q);

	SE3 computeFK(MM_JVec _q);

	MM_Jacobian_CoM computeJ_com(MM_JVec _q);	
	MM_Jacobian computeJ_b(MM_JVec _q);
	MM_Jacobian computeJ_s(MM_JVec _q);
	MM_Jacobian computeJdot_b(MM_JVec _q, MM_JVec _dq);
	MM_Jacobian computeJdot_s(MM_JVec _q, MM_JVec _dq);


	MM_MassMat getM();
	MM_MassMat getMinv();
	MM_MassMat getC();
	MM_JVec getG();
	Vector3d getCoM();

	SE3 getFK();
	SO3 getRMat();

	MM_Jacobian_CoM getJ_com();
	MM_Jacobian getJ_b();
	MM_Jacobian getJ_s();
	MM_Jacobian getJdot_b();
	MM_Jacobian getJdot_s();

	Twist getBodyTwist();

	MM_JVec FrictionEstimation(MM_JVec dq);

	// Joint Space
	MM_JVec ComputedTorqueControl( MM_JVec q,MM_JVec dq,MM_JVec q_des,MM_JVec dq_des,MM_JVec ddq_des);
	MM_JVec ComputedTorqueControl( MM_JVec q,MM_JVec dq,MM_JVec q_des,MM_JVec dq_des,MM_JVec ddq_des, MM_JVec _tau_ext);
	MM_JVec PassivityInverseDynamicControl( MM_JVec q,MM_JVec dq,MM_JVec q_des,MM_JVec dq_des,MM_JVec ddq_des);
	MM_JVec PassivityInverseDynamicControl( MM_JVec q,MM_JVec dq,MM_JVec q_des,MM_JVec dq_des,MM_JVec ddq_des, MM_JVec _tau_ext);
	MM_JVec HinfControl(MM_JVec q,MM_JVec dq,MM_JVec q_des,MM_JVec dq_des,MM_JVec ddq_des);
	MM_JVec HinfControl(MM_JVec q,MM_JVec dq,MM_JVec q_des,MM_JVec dq_des,MM_JVec ddq_des, MM_JVec _tau_ext);
	
	// Task Space
    MM_JVec TaskInverseDynamicsControl(MM_JVec q_dot, SE3 T_des, Twist V_des, Twist V_dot_des);
	MM_JVec TaskPassivityInverseDynamicsControl(MM_JVec q_dot, SE3 T_des, Twist V_des, Twist V_dot_des);
	MM_JVec TaskImpedanceControl(MM_JVec q_dot, SE3 T_des, Twist V_des, Twist V_dot_des, Twist F_des, Twist F_ext);
	MM_JVec TaskPassivityImpedanceControl(MM_JVec q_dot, SE3 T_des, Twist V_des, Twist V_dot_des, Twist F_des, Twist F_ext);
  	MM_JVec TaskRedundantIDC(MM_JVec q, MM_JVec q_dot, MM_JVec dq, MM_JVec dq_dot, MM_JVec dq_ddot, SE3 T_des, Twist V_des, Twist V_dot_des);
	MM_JVec TaskRedundantImpedanceControl(MM_JVec q, MM_JVec q_dot, MM_JVec dq, MM_JVec dq_dot, MM_JVec dq_ddot, SE3 T_des, Twist V_des, Twist V_dot_des, Twist F_des, Twist F_ext);
    MM_JVec TaskStablePD(SE3 T_des, Twist V_des, Twist V_dot_des, MM_JVec q, MM_JVec q_dot, MM_JVec dq, MM_JVec dq_dot, MM_JVec dq_ddot);

	void resetTaskAdmittance();
	void TaskAdmittance(SE3 T_des, Twist V_des, Twist V_dot_des, SE3 &T_adm, Twist &V_adm, Twist &V_dot_adm, Twist F_des, Twist F_ext);    

	MM_JVec NRIC(MM_JVec q_r, MM_JVec dq_r, MM_JVec q_n, MM_JVec dq_n);

	void computeAlpha(MM_JVec edot, MM_JVec tau_c);
	void saturationMaxTorque(MM_JVec &torque, MM_JVec MAX_TORQUES);

private:
	MM_JVec q, dq, ddq;
	MM_JVec tau, tau_bd, tau_ext, ddq_res;
	MM_JVec e, eint;
	MM_MassMat M, Minv, C;
	MM_JVec G;
	Vector3d CoM;

	SE3 T_M;
	SE3 T_ee;
	SO3 R_ee;
	MM_ScrewList Slist, Blist;
	MM_Jacobian J_b, J_s;
	MM_Jacobian dJ_b, dJ_s;
	MM_Jacobian_CoM J_com;

	Twist V_b, V_s;
	Twist V_dot;
	Twist lambda, lambda_dot, lambda_int;

	SE3 T_ref;
	Twist V_ref, V_dot_ref;

	Twist F_eff;
	Twist gamma, gamma_int;

	Matrix6d A_tool, B_tool;

	Vector3d r_floor;
    Matrix3d r_ceil;
	Vector6d G_tool;
	Vector6d G_FT;
	Vector6d F_FT;

private:
	bool isUpdated = false;
	string robotModel;
	int n_dof;
	double period;

	void* FD_handle;
	void* CoM_x_handle;
	void* M_handle;
	void* Minv_handle;
	void* C_handle;
	void* G_handle;
	void* J_s_handle;
	void* J_b_handle;
	void* dJ_s_handle;
	void* dJ_b_handle;
	void* J_com_handle;
	void* FK_handle;
	
	eval_t FD_eval;
	eval_t CoM_x_eval;
	eval_t M_eval;
	eval_t Minv_eval;
	eval_t C_eval;
	eval_t G_eval;
	eval_t J_s_eval;
	eval_t J_b_eval;
	eval_t dJ_s_eval;
	eval_t dJ_b_eval;
	eval_t J_com_eval;
	eval_t FK_eval;

	// casadi::Function fd_cs, M_cs, Minv_cs, C_cs, G_cs, J_s_cs, J_b_cs, FK_cs;
	Matrix6d Task_Kp;
	Matrix6d Task_Kv;
	Matrix6d Task_Ki;
	MM_JMat Task_K;

	Matrix6d Task_Kp_imp;
	Matrix6d Task_Kv_imp;
	Matrix6d Task_Kgama_imp;
	Matrix6d Task_K_imp;

	Matrix6d A_, D_, K_;
    Matrix6d A_lambda, D_lambda, K_lambda;

    MM_JMat Kp;
    MM_JMat Kv;
    MM_JMat Ki;

    MM_JMat Hinf_Kp;
    MM_JMat Hinf_Kv;
    MM_JMat Hinf_Ki;
    MM_JMat Hinf_K_gamma;

	MM_JMat NRIC_Kp;
    MM_JMat NRIC_Ki;
    MM_JMat NRIC_K_gamma;

	// Friction model parameters
	MM_JVec Fc;
	MM_JVec Fv1;
	MM_JVec Fv2;

	double alpha = 0.0;

};
#endif // CS_HYUMM_H