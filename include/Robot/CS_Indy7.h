#ifndef CS_INDY7_H
#define CS_INDY7_H

#include "iostream"
#include "json_loader.h"
// #include "modern_robotics.h"
#include <Eigen/Dense>
// #include <casadi/casadi.hpp>
#include <dlfcn.h>
#include "PropertyDefinition.h"

typedef long long int casadi_int;
typedef int (*eval_t)(const double**, double**, casadi_int*, double*, int);

using namespace Eigen;
using namespace std;

class CS_Indy7 {
public:
	CS_Indy7();
	
	// ~CS_Indy7(){
	// 	// Free the handle
    // 	dlclose(fd_handle);
    // 	dlclose(M_handle);
    // 	dlclose(Minv_handle);
    // 	dlclose(C_handle);
    // 	dlclose(G_handle);
    // 	dlclose(J_s_handle);
    // 	dlclose(J_b_handle);
    // 	dlclose(FK_handle);
	// };

	// JsonLoader loader_;

	void CSSetup(const string& _modelPath, double _period);
	void setPIDgain(Arm_JVec _Kp, Arm_JVec _Kd, Arm_JVec _Ki);
	void setHinfgain(Arm_JVec _Hinf_Kp, Arm_JVec _Hinf_Kd, Arm_JVec _Hinf_Ki, Arm_JVec _Hinf_K_gamma);
	void setNRICgain(Arm_JVec _NRIC_Kp, Arm_JVec _NRIC_Ki, Arm_JVec _NRIC_K_gamma);

	void updateRobot(Arm_JVec _q, Arm_JVec _dq);

	Arm_JVec computeFD(Arm_JVec _q, Arm_JVec _dq, Arm_JVec _tau);
	void computeRK45(Arm_JVec _q, Arm_JVec _dq, Arm_JVec _tau, Arm_JVec &_q_nom, Arm_JVec &_dq_nom);

	Arm_MassMat computeM(Arm_JVec _q);
	Arm_MassMat computeMinv(Arm_JVec _q);
	Arm_MassMat computeC(Arm_JVec _q, Arm_JVec _dq);
	Arm_JVec computeG(Arm_JVec _q);

	SE3 computeFK(Arm_JVec _q);

	Arm_Jacobian computeJ_b(Arm_JVec _q);
	Arm_Jacobian computeJ_s(Arm_JVec _q);

	Arm_MassMat getM();
	Arm_MassMat getMinv();
	Arm_MassMat getC();
	Arm_JVec getG();

	SE3 getFK();

	Arm_Jacobian getJ_b();
	Arm_Jacobian getJ_s();

	Arm_JVec FrictionEstimation(Arm_JVec dq);

	Arm_JVec ComputedTorqueControl( Arm_JVec q,Arm_JVec dq,Arm_JVec q_des,Arm_JVec dq_des,Arm_JVec ddq_des);
	Arm_JVec ComputedTorqueControl( Arm_JVec q,Arm_JVec dq,Arm_JVec q_des,Arm_JVec dq_des,Arm_JVec ddq_des, Arm_JVec tau_ext);
    void saturationMaxTorque(Arm_JVec &torque, Arm_JVec MAX_TORQUES);
    
    Arm_JVec HinfControl(Arm_JVec q,Arm_JVec dq,Arm_JVec q_des,Arm_JVec dq_des,Arm_JVec ddq_des);
	Arm_JVec HinfControl(Arm_JVec q,Arm_JVec dq,Arm_JVec q_des,Arm_JVec dq_des,Arm_JVec ddq_des, Arm_JVec tau_ext);
	Arm_JVec NRIC(Arm_JVec q_r, Arm_JVec dq_r, Arm_JVec q_n, Arm_JVec dq_n);

private:
	Arm_JVec q, dq, ddq;
	Arm_JVec tau, ddq_res;
	Arm_JVec e, eint;
	Arm_MassMat M, Minv, C;
	Arm_JVec G;

	SE3 T_ee;
	Arm_Jacobian J_b, J_s;

private:
	bool isUpdated = false;
	string robotModel;
	int n_dof;
	double period;

	void* FD_handle;
	void* M_handle;
	void* Minv_handle;
	void* C_handle;
	void* G_handle;
	void* J_s_handle;
	void* J_b_handle;
	void* FK_handle;
	
	eval_t FD_eval;
	eval_t M_eval;
	eval_t Minv_eval;
	eval_t C_eval;
	eval_t G_eval;
	eval_t J_s_eval;
	eval_t J_b_eval;
	eval_t FK_eval;

	// casadi::Function fd_cs, M_cs, Minv_cs, C_cs, G_cs, J_s_cs, J_b_cs, FK_cs;

	Arm_JMat M_imp;
    Arm_JMat B_imp;
    Arm_JMat K_imp;

    Arm_JMat Kp;
    Arm_JMat Kv;
    Arm_JMat Ki;

    Arm_JMat Hinf_Kp;
    Arm_JMat Hinf_Kv;
    Arm_JMat Hinf_Ki;
    Arm_JMat Hinf_K_gamma;

	Arm_JMat NRIC_Kp;
    Arm_JMat NRIC_Ki;
    Arm_JMat NRIC_K_gamma;

	// Friction model parameters
	Arm_JVec Fc;
	Arm_JVec Fv1;
	Arm_JVec Fv2;

};
#endif // CS_INDY7_H