#include "CS_hyumm.h"


CS_hyumm::CS_hyumm()
{
    robotModel = "hyumm";
    n_dof = 9;

    Task_Kp = Matrix6d::Zero();
    Task_Kv = Matrix6d::Zero();
    Task_Ki = Matrix6d::Zero();
    Task_K = MM_JMat::Zero();

    Hinf_Kp = MM_JMat::Zero();
    Hinf_Kv = MM_JMat::Zero();
    Hinf_Ki = MM_JMat::Zero();
    Hinf_K_gamma = MM_JMat::Zero();

    NRIC_Kp = MM_JMat::Zero();
    NRIC_Ki = MM_JMat::Zero();
    NRIC_K_gamma = MM_JMat::Zero();

    Kp = MM_JMat::Zero();
    Kv = MM_JMat::Zero();
    Ki = MM_JMat::Zero();

    e = MM_JVec::Zero();
    eint = MM_JVec::Zero();

    r_floor << X_com, Y_com, Z_com;
    r_ceil = VecToso3(r_floor); 
    G_tool << 0, 0, -mass_tool*9.8, 0, 0, 0;

    for (int i=0; i<this->n_dof; ++i)
    {
        switch(i)
        {
        case 0:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 70+1.0/invL2sqr_1 ;
            break;
        case 1:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 70+1.0/invL2sqr_2 ;

            break;
        case 2:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 70.0+1.0/invL2sqr_3 ;

            break;
        case 3:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 70.0+1.0/invL2sqr_4 ;

            break;
        case 4:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 70.0+1.0/invL2sqr_5 ;

            break;
        case 5:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 70.0+1.0/invL2sqr_6 ;

            break;
#ifdef __RP__
        case 6:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 70.0+1.0/invL2sqr_7 ;

            break;
#endif
        }
    }
   for (int i=0; i<this->n_dof; ++i)
    {
        switch(i)
        {
        case 0:
            Kp(i,i) = 70.0;
            Kv(i,i) = 55.0;
            Ki(i,i)=10.0;
            break;
        case 1:
            Kp(i,i) = 75.0;
            Kv(i,i) = 55.0;
            Ki(i,i)=10.0;
            break;
        case 2:
            Kp(i,i) = 40.0;
            Kv(i,i) = 30.0;
            Ki(i,i)=5.0;
            break;
        case 3:
            Kp(i,i) = 25.0;
            Kv(i,i) = 15.0;
            Ki(i,i)=3.0;
            break;
        case 4:
            Kp(i,i) = 25.0;
            Kv(i,i) = 15.0;
            Ki(i,i)=3.0;
            break;
        case 5:
            Kp(i,i) = 18.0;
            Kv(i,i) = 3.0;
            Ki(i,i)=1.0;
            break;
        case 6:
            Kp(i,i) = 18.0;
            Kv(i,i) = 3.0;
            Ki(i,i)=1.0;
            break;
        }
    }
}

void CS_hyumm::CSSetup(const string& _modelPath, double _period)// : loader_(_modelPath), period[sec]
{
	JsonLoader loader_ = JsonLoader(_modelPath);

	robotModel = loader_.getValue("name").asString();
	n_dof = std::stoi(loader_.getValue("n_dof").asString());
    period = _period;

    T_M << 1,0,0,0.3,
			0,1,0,-0.1865,
			0,0,1,1.7255,
			0,0,0,1;

	Slist << 1, 0, 0, 0, 0.5275, 0.9775, -0.0035, 1.3275, -0.1865,
				0, 1, -0, -0.3, 1.17129e-16, 2.17049e-16, -0.3, 6.56142e-16, -0.3,
				0, 0, 0, 0, -0.3, -0.3, -6.66134e-17, -0.3, -1.33227e-16,
				0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, -1, -1, 0, -1, 0,
				0, 0, 1, 1, 0, 0, 1, 0, 1;
	Blist = Ad(TransInv(T_M)) * Slist;

    lambda_int = Twist::Zero();
    gamma_int = Twist::Zero();

    r_floor << X_com, Y_com, Z_com;
    r_ceil = VecToso3(r_floor); 
    G_tool << 0, 0, -mass_tool*9.8, 0, 0, 0;
	G_FT << 0, 0, -mass_FT*9.8, 0, 0, 0;

    A_tool=Matrix6d::Zero(); B_tool=Matrix6d::Zero();
    A_tool(0,0) = mass_tool; A_tool(1,1) = mass_tool; A_tool(2,2) = mass_tool;
    A_tool(3,3) = Ixx; A_tool(4,4) = Iyy; A_tool(5,5) = Izz; 

    for (int i=0; i<this->n_dof; ++i)
    {
        switch(i)
        {
        case 0:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 60.0+1.0/invL2sqr_1 ;
            break;
        case 1:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 60.0+1.0/invL2sqr_2 ;

            break;
        case 2:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 60.0+1.0/invL2sqr_3 ;

            break;
        case 3:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 60.0+1.0/invL2sqr_4 ;

            break;
        case 4:
              Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 60.0+1.0/invL2sqr_5 ;

            break;
        case 5:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 60.0+1.0/invL2sqr_6 ;

            break;
#ifdef __RP__
        case 6:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 60.0+1.0/invL2sqr_7 ;

            break;
#endif
        }
    }
   for (int i=0; i<this->n_dof; ++i)
    {
        switch(i)
        {
        case 0:
            Kp(i,i) = 70.0;
            Kv(i,i) = 55.0;
            Ki(i,i)=10.0;
            break;
        case 1:
            Kp(i,i) = 75.0;
            Kv(i,i) = 55.0;
            Ki(i,i)=10.0;
            break;
        case 2:
            Kp(i,i) = 40.0;
            Kv(i,i) = 30.0;
            Ki(i,i)=5.0;
            break;
        case 3:
            Kp(i,i) = 25.0;
            Kv(i,i) = 15.0;
            Ki(i,i)=3.0;
            break;
        case 4:
            Kp(i,i) = 25.0;
            Kv(i,i) = 15.0;
            Ki(i,i)=3.0;
            break;
        case 5:
            Kp(i,i) = 25.0;
            Kv(i,i) = 15.0;
            Ki(i,i)=1.0;
            break;
        case 6:
            Kp(i,i) = 25.0;
            Kv(i,i) = 15.0;
            Ki(i,i)=1.0;
            break;
        }
    }
	
	// Friction Parameters
    Fc << F_c;
    Fv1 << F_v1;
    Fv2 << F_v2; 

	// Load the shared library
    string casadi_path = "../lib/URDF2CASADI/";
    casadi_path = casadi_path + robotModel + "/" + robotModel;

    string func_path = casadi_path + "_fd.so";
    FD_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (FD_handle == 0) {
        throw std::runtime_error("Cannot open hyumm_fd.so");
    }
    func_path = casadi_path + "_CoM_x.so";
    CoM_x_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (CoM_x_handle == 0) {
        throw std::runtime_error("Cannot open hyumm_CoM_x.so");
    }
    func_path = casadi_path + "_M.so";
    M_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (M_handle == 0) {
        throw std::runtime_error("Cannot open hyumm_M.so");
    }
    func_path = casadi_path + "_Minv.so";
    Minv_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (Minv_handle == 0) {
        throw std::runtime_error("Cannot open hyumm_Minv.so");
    }
    func_path = casadi_path + "_C.so";
    C_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (C_handle == 0) {
        throw std::runtime_error("Cannot open hyumm_C.so");
    }
    func_path = casadi_path + "_G.so";
    G_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (G_handle == 0) {
        throw std::runtime_error("Cannot open hyumm_G.so");
    }
    func_path = casadi_path + "_fk_ee.so";
    FK_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (FK_handle == 0) {
        throw std::runtime_error("Cannot open hyumm_fk_ee.so");
    }
    func_path = casadi_path + "_J_b.so";
    J_b_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (J_b_handle == 0) {
        throw std::runtime_error("Cannot open hyumm_J_b.so");
    }
    func_path = casadi_path + "_J_s.so";
    J_s_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (J_s_handle == 0) {
        throw std::runtime_error("Cannot open hyumm_J_s.so");
    }
    func_path = casadi_path + "_dJ_b.so";
    dJ_b_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (J_b_handle == 0) {
        throw std::runtime_error("Cannot open hyumm_dJ_b.so");
    }
    func_path = casadi_path + "_dJ_s.so";
    dJ_s_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (J_s_handle == 0) {
        throw std::runtime_error("Cannot open hyumm_dJ_s.so");
    }
    func_path = casadi_path + "_J_com.so";
    J_com_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (J_com_handle == 0) {
        throw std::runtime_error("Cannot open hyumm_J_com.so");
    }

    // Reset error
    dlerror();
    // Function evaluation
    FD_eval = (eval_t)dlsym(FD_handle, "aba");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    CoM_x_eval = (eval_t)dlsym(CoM_x_handle, "centerOfMass");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    M_eval = (eval_t)dlsym(M_handle, "M");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    Minv_eval = (eval_t)dlsym(Minv_handle, "Minv");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }

    C_eval = (eval_t)dlsym(C_handle, "coriolis");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    G_eval = (eval_t)dlsym(G_handle, "generalized_gravity");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    FK_eval = (eval_t)dlsym(FK_handle, "fk_T");
    if (dlerror()) {
        throw std::runtime_error("Failed to retrieve \"fk_T\" function.\n");
    }
    J_b_eval = (eval_t)dlsym(J_b_handle, "J_b");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    J_s_eval = (eval_t)dlsym(J_s_handle, "J_s");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    dJ_b_eval = (eval_t)dlsym(dJ_b_handle, "dJ_b");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    dJ_s_eval = (eval_t)dlsym(dJ_s_handle, "dJ_s");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    J_com_eval = (eval_t)dlsym(J_com_handle, "jacobianCenterOfMass");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }

}

void CS_hyumm::setPIDgain(MM_JVec _Kp, MM_JVec _Kd, MM_JVec _Ki)
{
    Kp = _Kp.asDiagonal();
    Kv = _Kd.asDiagonal();
    Ki = _Ki.asDiagonal();
}

void CS_hyumm::setHinfgain(MM_JVec _Hinf_Kp, MM_JVec _Hinf_Kd, MM_JVec _Hinf_Ki, MM_JVec _Hinf_K_gamma)
{
    Hinf_Kp = _Hinf_Kp.asDiagonal();
    Hinf_Kv = _Hinf_Kd.asDiagonal();
    Hinf_Ki = _Hinf_Ki.asDiagonal();
    Hinf_K_gamma = _Hinf_K_gamma.asDiagonal();
}

void CS_hyumm::setNRICgain(MM_JVec _NRIC_Kp, MM_JVec _NRIC_Ki, MM_JVec _NRIC_K, MM_JVec _NRIC_gamma)
{
    for (int i=0; i<this->n_dof; ++i)
    {
        NRIC_Kp(i,i) = _NRIC_Kp(i);
        NRIC_Ki(i,i) = _NRIC_Ki(i);
        NRIC_K_gamma(i,i) = _NRIC_K(i)+1/ _NRIC_gamma(i);
    }
}

void CS_hyumm::setTaskgain(Twist _Kp, Twist _Kv, MM_JVec _K)
{
    Task_Kp = _Kp.asDiagonal();
    Task_Kv = _Kv.asDiagonal();
    Task_K = _K.asDiagonal();
}

void CS_hyumm::setTaskImpedancegain(Matrix6d _A, Matrix6d _D, Matrix6d _K)
{
    A_ = _A;
    D_ = _D;
    K_ = _K;
}


void CS_hyumm::updateRobot(MM_JVec _q, MM_JVec _dq)
{
    M = computeM(_q);
    Minv = computeMinv(_q);
    C = computeC(_q, _dq);
    G = computeG(_q); 

    J_b = computeJ_b(_q);
    dJ_b = computeJdot_b(_q, _dq);

    T_ee = computeFK(_q);

    CoM = computeCoM(_q);
    J_com = computeJ_com(_q);

    V_b = J_b*_dq;

    isUpdated=true;   
}

MM_JVec CS_hyumm::addLoadedTorque(MM_JVec _q, Vector3d _com, double _m)
{
    MM_JVec _tau;
    J_b = computeJ_b(_q);

    r_floor << _com(0), _com(1), _com(2);
    r_ceil = VecToso3(r_floor); 
    G_tool << 0, 0, _m*9.8, 0, 0, 0;

    Matrix6d AdT = Matrix6d::Zero(); 
    AdT.block<3,3>(0,0) = R_ee.transpose();
    AdT.block<3,3>(3,3) = R_ee.transpose();
    AdT.block<3,3>(3,0) = -R_ee.transpose()*r_ceil;
    

    Twist loadtool = AdT*G_tool;
    
    _tau = J_b.transpose()*loadtool;
    
    return _tau;
}

MM_JVec CS_hyumm::computeFD(MM_JVec _q, MM_JVec _dq, MM_JVec _tau)
{
    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = n_dof;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[3*sz_arg];
    double* res[sz_res*sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_pos[sz_arg];
    double input_vel[sz_arg];
    double input_tau[sz_arg];

    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_pos[i] = _q(i);
        input_vel[i] = _dq(i);
        input_tau[i] = _tau(i);
        arg[3*i] = &input_pos[i];
        arg[3*i+1] = &input_vel[i];
        arg[3*i+2] = &input_tau[i];
    }

    // Set output buffers
    double output_values[sz_res]; // 6x1 Vector
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (FD_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        // if(!isnan(output_values[i]))
            ddq_res(i) = output_values[i];
        // else
        //     cout<<[ERROR NaN]<<endl;
    }

    // // fd method
    // Minv = computeMinv(_q);
    // C = computeC(_q,_dq);
    // G = computeG(_q);

    // ddq_res = Minv*(_tau-C*_dq-G);
    
    return ddq_res;

}

void CS_hyumm::computeRK45(MM_JVec _q, MM_JVec _dq, MM_JVec _tau, MM_JVec &_q_nom, MM_JVec &_dq_nom, MM_JVec &_ddq_nom)
{
    MM_JVec k1, k2, k3, k4;

    // state update
    MM_JVec _q0 = _q;
    // MM_JVec _q_dot = info.act.q_dot;
    MM_JVec _q_dot0 = _dq;
    // MM_JVec _tau = _tau;

    // 1st stage
    k1 = computeFD(_q, _q_dot0, _tau);
    MM_JVec _q1 = _q + 0.5 * period * _dq;		// period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit
    MM_JVec _q_dot1 = _dq + 0.5 * period * k1; // k2(q_dot)
    
    // 2nd stage
    k2 = computeFD(_q1, _q_dot1, _tau);
    MM_JVec _q2 = _q + 0.5 * period * _q_dot1;
    MM_JVec _q_dot2 = _dq + 0.5 * period * k2;
    
    // 3th stage
    k3 = computeFD(_q2, _q_dot2, _tau);
    MM_JVec _q3 = _q + period * _q_dot2;
    MM_JVec _q_dot3 = _dq + period * k3;
    
    // 4th stage
    k4 = computeFD(_q3, _q_dot3, _tau);
    _q_nom = _q + (period / 6.0) * (_dq + 2 * (_q_dot1 + _q_dot2) + _q_dot3);
    _dq_nom = _dq + (period / 6.0) * (k1 + 2 * (k2 + k3) + k4);
    _ddq_nom = k1;
}

Twist CS_hyumm::computeF_Tool(Twist _dx, Twist _ddx)
{
    Twist res;
    Matrix6d adj = ad(_dx);
    Matrix6d AdT = Matrix6d::Zero(); 

	AdT.block<3,3>(0,0) = R_ee.transpose();
    F_FT = AdT * G_FT;
    AdT.block<3,3>(3,3) = R_ee.transpose();
    AdT.block<3,3>(3,0) = -R_ee.transpose()*r_ceil;
    
    B_tool = A_tool*adj - adj.transpose()*A_tool;

    res = A_tool*_ddx + B_tool*_dx + F_FT + AdT*G_tool;
    return res;
}

Twist CS_hyumm::computeF_Threshold(Twist _F)
{
    Twist res;

    for(int i=0; i<6; i++)
    {
		if(i<3)
		{
			if(0.8>abs(_F(i)))
			{
				res(i)=0.0;
			}
			else
			{
				res(i)=_F(i);
			}
		}
		else if(i==5)
		{
			if(0.005>abs(_F(i)))
			{
				res(i)=0.0;
			}
			else
			{
				res(i)=0.1*_F(i);
				// res(i)=0.0;
			}
		}
		else 
		{
			if(0.05>abs(_F(i)))
			{
				res(i)=0.0;
			}
			else
			{
				res(i)=1*_F(i);
			}
		}
    }
    return res;
}

MM_MassMat CS_hyumm::computeM(MM_JVec _q)
{
    // casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    // vector<casadi::DM> arg = {q_dm};
    // vector<casadi::DM> M_res = M_cs(arg);

    // return M;
    
    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = n_dof;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[sz_arg];
    double* res[sz_res*sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_values[sz_arg];
    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_values[i] = _q(i);
        arg[i] = &input_values[i];
    }

    // Set output buffers
    double output_values[sz_res*sz_res]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (M_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < sz_res; ++j) {   
            M(j,i) = output_values[i * sz_res + j];
        }
    }

    return M;
}

MM_MassMat CS_hyumm::computeMinv(MM_JVec _q)
{
    // casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    // vector<casadi::DM> arg = {q_dm};
    // vector<casadi::DM> Minv_res = Minv_cs(arg);

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = n_dof;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[sz_arg];
    double* res[sz_res*sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_values[sz_arg];
    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_values[i] = _q(i);
        arg[i] = &input_values[i];
    }

    // Set output buffers
    double output_values[sz_res*sz_res]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (Minv_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < sz_res; ++j) {   
            Minv(j,i) = output_values[i * sz_res + j];
        }
    }

    return Minv;
}

MM_MassMat CS_hyumm::computeC(MM_JVec _q, MM_JVec _dq)
{
    // casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    // casadi::DM dq_dm = casadi::DM(vector<double>(_dq.data(), _dq.data() + _dq.size()));
    // map<string, casadi::DM> arg;
    // arg["q"] = q_dm; arg["dq"] = dq_dm;
    // map<string, casadi::DM> C_res = C_cs(arg);

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = n_dof;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[2*sz_arg];
    double* res[sz_res*sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_pos[sz_arg];
    double input_vel[sz_arg];
    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_pos[i] = _q(i);
        input_vel[i] = _dq(i);
        arg[2*i] = &input_pos[i];
        arg[2*i+1] = &input_vel[i];
    }

    // Set output buffers
    double output_values[sz_res*sz_res]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (C_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < sz_res; ++j) {   
            C(j,i) = output_values[i * sz_res + j];
        }
    }

    return C;
}

MM_JVec CS_hyumm::computeG(MM_JVec _q)
{
    // casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    // vector<casadi::DM> arg = {q_dm};
    // vector<casadi::DM> G_res = G_cs(arg);

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = n_dof;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[sz_arg];
    double* res[sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_values[sz_arg];
    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_values[i] = _q(i);
        arg[i] = &input_values[i];
    }

    // Set output buffers
    double output_values[sz_res]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (G_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        G(i) = output_values[i];
    }

    return G;
}

Vector3d CS_hyumm::computeCoM(MM_JVec _q)
{
    // casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    // vector<casadi::DM> arg = {q_dm};
    // vector<casadi::DM> G_res = G_cs(arg);

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = 3;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[sz_arg];
    double* res[sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_values[sz_arg];
    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_values[i] = _q(i);
        arg[i] = &input_values[i];
    }

    // Set output buffers
    double output_values[sz_res]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (CoM_x_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        CoM(i) = output_values[i];
    }

    return CoM;
}

SE3 CS_hyumm::computeFK(MM_JVec _q)
{
    // casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    // vector<casadi::DM> arg = {q_dm};
    // vector<casadi::DM> FK_res = FK_cs(arg);

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = 4;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[sz_arg];
    double* res[sz_res*sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_values[sz_arg];
    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_values[i] = _q(i);
        arg[i] = &input_values[i];
    }

    // Set output buffers
    double output_values[sz_res*sz_res]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (FK_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < sz_res; ++j) {   
            T_ee(j,i) = output_values[i * sz_res + j];
        }
    }

    R_ee = T_ee.block<3,3>(0,0);

    return T_ee;
}

MM_Jacobian_CoM CS_hyumm::computeJ_com(MM_JVec _q)
{
    // casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    // vector<casadi::DM> arg = {q_dm};
    // vector<casadi::DM> J_com_res = Minv_cs(arg);

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = 3;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[sz_arg];
    double* res[sz_res*n_dof];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_values[sz_arg];
    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_values[i] = _q(i);
        arg[i] = &input_values[i];
    }

    // Set output buffers
    double output_values[sz_res*n_dof]; // 3xn_dof matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (J_com_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < n_dof; ++i) {
        for (casadi_int j = 0; j < sz_res; ++j) {   
            J_com(j,i) = output_values[i * sz_res + j];
        }
    }

    return J_com;
}

MM_Jacobian CS_hyumm::computeJ_b(MM_JVec _q)
{
    // casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    // vector<casadi::DM> arg = {q_dm};
    // vector<casadi::DM> J_b_res = J_b_cs(arg);

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = n_dof;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[sz_arg];
    double* res[6*sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_values[sz_arg];
    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_values[i] = _q(i);
        arg[i] = &input_values[i];
    }

    // Set output buffers
    double output_values[6*sz_res]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (J_b_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < 6; ++j) {   
            J_b(j,i) = output_values[i * 6 + j];
        }
    }

    return J_b;
}


MM_Jacobian CS_hyumm::computeJdot_b(MM_JVec _q, MM_JVec _dq)
{
    dJ_b = dJacobianBody(T_M, Blist, _q, _dq);

    return dJ_b;
}

MM_Jacobian CS_hyumm::computeJ_s(MM_JVec _q)
{
    // casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    // vector<casadi::DM> arg = {q_dm};
    // vector<casadi::DM> J_s_res = J_s_cs(arg);

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = n_dof;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[sz_arg];
    double* res[6*sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_values[sz_arg];
    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_values[i] = _q(i);
        arg[i] = &input_values[i];
    }

    // Set output buffers
    double output_values[6*sz_res]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (J_s_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < 6; ++j) {   
            J_s(j,i) = output_values[i * 6 + j];
        }
    }

    return J_s;
}

MM_Jacobian CS_hyumm::computeJdot_s(MM_JVec _q, MM_JVec _dq)
{
    // casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    // vector<casadi::DM> arg = {q_dm};
    // vector<casadi::DM> dJ_s_res = dJ_s_cs(arg);

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = n_dof;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[2*sz_arg];
    double* res[6*sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_pos[sz_arg];
    double input_vel[sz_arg];
    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_pos[i] = _q(i);
        input_vel[i] = _dq(i);
        arg[2*i] = &input_pos[i];
        arg[2*i+1] = &input_vel[i];
    }

    // Set output buffers
    double output_values[6*sz_res]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (dJ_s_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < 6; ++j) {   
            dJ_s(j,i) = output_values[i * 6 + j];
        }
    }

    return dJ_s;
}

MM_MassMat CS_hyumm::getM()
{
    return M;
}
MM_MassMat CS_hyumm::getMinv()
{
    return Minv;
}
MM_MassMat CS_hyumm::getC()
{
    return C;
}
MM_JVec CS_hyumm::getG()
{
    return G;
}
Vector3d CS_hyumm::getCoM()
{
    return CoM;
}
SE3 CS_hyumm::getFK()
{
    return T_ee;
}
SO3 CS_hyumm::getRMat()
{
    return R_ee;
}
MM_Jacobian_CoM CS_hyumm::getJ_com()
{
    return J_com;
}
MM_Jacobian CS_hyumm::getJ_b()
{
    return J_b;
}
MM_Jacobian CS_hyumm::getJ_s()
{
    return J_s;
}
MM_Jacobian CS_hyumm::getJdot_b()
{
    return dJ_b;
}
MM_Jacobian CS_hyumm::getJdot_s()
{
    return dJ_s;
}
Twist CS_hyumm::getBodyTwist()
{
    return V_b;
}


MM_JVec CS_hyumm::FrictionEstimation(MM_JVec dq)
{
    MM_JVec tau_fric;
    
    for (int i; i<NRMK_DRIVE_NUM; i++)
	{
		if(dq(i)>0.0)
			tau_fric(i) = Fc(i)+Fv1(i)*(1-exp(-fabs(dq(i)/Fv2(i))));
		else if(dq(i)<0.0)
			tau_fric(i) = -(Fc(i)+Fv1(i)*(1-exp(-fabs(dq(i)/Fv2(i)))));
		else
			tau_fric(i) = 0.0;
	}

    return tau_fric;
}

MM_JVec CS_hyumm::ComputedTorqueControl( MM_JVec q,MM_JVec dq,MM_JVec q_des,MM_JVec dq_des,MM_JVec ddq_des)
{
    MM_JVec e = q_des-q;
    MM_JVec edot = dq_des-dq;
    
    eint = eint + e*period;	
    
    if(isUpdated)
    {
        MM_JVec ddq_ref = ddq_des + Kv*edot + Kp*e;
        tau = M*ddq_ref + C*dq + G;
        isUpdated = false;
    }
    else
    {
        M = computeM(q);
        C = computeC(q, dq);
        G = computeG(q);
        MM_JVec ddq_ref = ddq_des + Kv*edot + Kp*e;
        tau = M*ddq_ref+C*dq+G;
    }
    return tau;   
}

MM_JVec CS_hyumm::ComputedTorqueControl( MM_JVec q,MM_JVec dq,MM_JVec q_des,MM_JVec dq_des,MM_JVec ddq_des, MM_JVec _tau_ext)
{
    MM_JVec e = q_des-q;
    MM_JVec edot = dq_des-dq;

    // apperent inertia, spring, damper
    MM_MassMat m_ = MM_MassMat::Zero();
    MM_JVec k_, d_;

    m_.diagonal()<< 20, 20, 20, 5, 2.5, 1.25, 0.25, 0.25, 0.25;
    k_ << 800.0, 800.0, 800.0, 500.0, 500.0, 300.0, 200.0, 200.0, 200.0;
    d_ << 155.0, 155.0, 155.0, 50.0, 50.0, 30.0, 20.0, 20.0, 20.0;
    
    
    tau_ext = _tau_ext;
    eint = eint + e*period;	
    
    if(isUpdated)
    {
        MM_JVec ddq_ref = ddq_des + m_.inverse()*(d_.cwiseProduct(edot) + k_.cwiseProduct(e) + tau_ext);
        // tau = M*ddq_ref + C*dq + G + tau_ext - tau_bd;
        tau = M*ddq_ref + C*dq + G;// + tau_ext - tau_bd;
        isUpdated = false;
    }
    else
    {
        M = computeM(q);
        C = computeC(q, dq);
        G = computeG(q);
        MM_JVec ddq_ref = ddq_des + m_.inverse()*(d_.cwiseProduct(edot) + k_.cwiseProduct(e) + tau_ext);
        // tau = M*ddq_ref+C*dq+G + tau_ext - tau_bd;
        tau = M*ddq_ref+C*dq+G;// + tau_ext - tau_bd;
    }
    // tau = K.cwiseProduct(tau);
    return tau;   
}

MM_JVec CS_hyumm::PassivityInverseDynamicControl( MM_JVec q,MM_JVec dq,MM_JVec q_des,MM_JVec dq_des,MM_JVec ddq_des)
{
    MM_JVec e = q_des-q;
    MM_JVec edot = dq_des-dq;
    
    eint = eint + e*period;	
    
    if(isUpdated)
    {
        MM_JVec ddq_ref = ddq_des + Kv*edot + Kp*e;
        MM_JVec dq_ref = dq_des + Kv*e + Kp*eint;

        tau = M*ddq_ref + C*dq + G;
        isUpdated = false;
    }
    else
    {
        M = computeM(q);
        C = computeC(q, dq);
        G = computeG(q);
        MM_JVec ddq_ref = ddq_des + Kv*edot + Kp*e;
        tau = M*ddq_ref+C*dq+G;
    }
    return tau;   
}

MM_JVec CS_hyumm::TaskInverseDynamicsControl(MM_JVec q_dot, SE3 T_des, Twist V_des, Twist V_dot_des)
{
    SE3 T_err = TransInv(T_ee)*T_des;
    SE3 invT_err = TransInv(T_err);
    
    Twist V_err = V_des - Ad(invT_err) * V_b;
    
    lambda = se3ToVec(MatrixLog6(T_err));
    lambda_dot = dlog6(-lambda) * V_err;

    Twist lambda_ddot_ref = Task_Kv * lambda_dot + Task_Kp * lambda;

    Twist V_dot_ref = Ad(T_err) * (V_dot_des + (dexp6(-lambda) * lambda_ddot_ref) + ad(V_err) * V_des - (ddexp6(-lambda, -lambda_dot) * lambda_dot));
    
    MM_pinvJacobian invJb = J_b.transpose() * (J_b * J_b.transpose()).inverse();
    MM_JVec qddot_ref = invJb * (V_dot_ref - dJ_b * q_dot);
    MM_JVec torques = M * qddot_ref + C * q_dot + G;

    return torques;
}

MM_JVec CS_hyumm::TaskPassivityInverseDynamicsControl(MM_JVec q_dot, SE3 T_des, Twist V_des, Twist V_dot_des)
{
    SE3 T_err = TransInv(T_ee)*T_des;
    SE3 invT_err = TransInv(T_err);
    
    Twist V_err = V_des - Ad(invT_err) * V_b;
    
    lambda = se3ToVec(MatrixLog6(T_err));
    lambda_int += lambda * period;

    lambda_dot = dlog6(-lambda) * V_err;

    Twist lambda_dot_ref = Task_Kv * lambda + Task_Kp * lambda_int;
    Twist lambda_ddot_ref = Task_Kv * lambda_dot + Task_Kp * lambda;

    Twist V_ref = Ad(T_err) * (V_des + dexp6(-lambda) * (lambda_dot_ref));
    Twist V_dot_ref = Ad(T_err) * (V_dot_des + (dexp6(-lambda) * lambda_ddot_ref) + ad(V_err) * V_des - (ddexp6(-lambda, -lambda_dot) * lambda_dot_ref));
    
    MM_pinvJacobian invJb = J_b.transpose() * (J_b * J_b.transpose()).inverse();
    MM_JVec q_dot_ref = invJb * V_ref;
    MM_JVec qddot_ref = invJb * (V_dot_ref - dJ_b * q_dot_ref);
    MM_JVec edot = q_dot_ref - q_dot;
    MM_JVec tau_ref = Task_K * edot;
    
    MM_JVec torques = M * qddot_ref + C * q_dot_ref + G + tau_ref;

    return torques;
}

MM_JVec CS_hyumm::TaskImpedanceControl(MM_JVec q_dot, SE3 T_des, Twist V_des, Twist V_dot_des, Twist F_des, Twist F_ext)
{
    SE3 T_err = TransInv(T_ee)*T_des;
    SE3 invT_err = TransInv(T_err);
    
    Twist V_err = V_des - Ad(invT_err) * V_b;
    
    lambda = se3ToVec(MatrixLog6(T_err));
    lambda_int += lambda * period;

    lambda_dot = dlog6(-lambda) * V_err;

    F_eff = F_des - Ad(T_err).transpose()*F_ext;
    gamma = dexp6(-lambda).transpose()*F_eff;
    gamma_int += gamma *period;

    A_lambda = dexp6(-lambda).transpose() * A_ * dexp6(-lambda);
    D_lambda = dexp6(-lambda).transpose() * (D_ * dexp6(-lambda) + A_*ddexp6(-lambda, -lambda_dot));
    K_lambda = dexp6(-lambda).transpose() * K_ * dexp6(-lambda);

    Task_Kv_imp = dlog6(-lambda)*(A_.inverse()*D_*dexp6(-lambda) + ddexp6(-lambda, -lambda_dot));
    Task_Kp_imp = dlog6(-lambda)*A_.inverse()*K_*dexp6(-lambda);
    Task_Kgama_imp = dlog6(-lambda)*A_.inverse()*dlog6(-lambda).transpose();


    Twist lambda_ddot_ref = -Task_Kv_imp * lambda_dot - Task_Kp_imp * lambda + Task_Kgama_imp * gamma;

    Twist V_dot_ref = Ad(T_err) * (V_dot_des - (dexp6(-lambda) * lambda_ddot_ref) + ad(V_err) * V_des - (ddexp6(-lambda, -lambda_dot) * lambda_dot));
    
    MM_pinvJacobian invJb = J_b.transpose() * (J_b * J_b.transpose()).inverse();
    MM_JVec qddot_ref = invJb * (V_dot_ref - dJ_b * q_dot);
    
    // MM_JVec torques = M * qddot_ref + C * q_dot + G + J_b.transpose()*F_ext;
    MM_JVec torques = M * qddot_ref + C * q_dot + G;
    

    return torques;
}

MM_JVec CS_hyumm::TaskPassivityImpedanceControl(MM_JVec q_dot, SE3 T_des, Twist V_des, Twist V_dot_des, Twist F_des, Twist F_ext)
{
    A_.diagonal() << 3, 3, 3, 0.5, 0.5, 0.5;
    D_.diagonal() << 10, 10, 10, 1, 1, 1;
    K_.diagonal() << 1000, 1000, 1000, 100, 100, 100;

    SE3 T_err = TransInv(T_ee)*T_des;
    SE3 invT_err = TransInv(T_err);
    
    Twist V_err = V_des - Ad(invT_err) * V_b;
    
    lambda = se3ToVec(MatrixLog6(T_err));
    lambda_int += lambda * period;

    lambda_dot = dlog6(-lambda) * V_err;

    F_eff = F_des - Ad(T_err).transpose()*F_ext;
    gamma = dexp6(-lambda).transpose()*F_eff;
    gamma_int += gamma *period;

    A_lambda = dexp6(-lambda).transpose() * A_ * dexp6(-lambda);
    D_lambda = dexp6(-lambda).transpose() * (D_ * dexp6(-lambda) + A_*ddexp6(-lambda, -lambda_dot));
    K_lambda = dexp6(-lambda).transpose() * K_ * dexp6(-lambda);

    Task_Kv_imp = dlog6(-lambda)*(A_.inverse()*D_*dexp6(-lambda) + ddexp6(-lambda, -lambda_dot));
    Task_Kp_imp = dlog6(-lambda)*A_.inverse()*K_*dexp6(-lambda);
    Task_Kgama_imp = dlog6(-lambda)*A_.inverse()*dlog6(-lambda).transpose();


    Twist lambda_dot_ref = -Task_Kv_imp * lambda - Task_Kp_imp * lambda_int + Task_Kgama_imp * gamma_int;
    Twist lambda_ddot_ref = -Task_Kv_imp * lambda_dot - Task_Kp_imp * lambda + Task_Kgama_imp * gamma;

    Twist V_ref = Ad(T_err) * (V_des + dexp6(-lambda) * (lambda_dot_ref));
    Twist V_dot_ref = Ad(T_err) * (V_dot_des + (dexp6(-lambda) * lambda_ddot_ref) + ad(V_err) * V_des - (ddexp6(-lambda, -lambda_dot) * lambda_dot_ref));
    
    MM_pinvJacobian invJb = J_b.transpose() * (J_b * J_b.transpose()).inverse();
    MM_JVec q_dot_ref = invJb * V_ref;
    MM_JVec qddot_ref = invJb * (V_dot_ref - dJ_b * q_dot_ref);

    Twist lambda_dot_tilde_ref = lambda_dot_ref - lambda_dot;
    Twist V_tilde_ref = -Ad(T_err)*dexp6(-lambda)*lambda_dot_tilde_ref;
    MM_JVec tau_ref = J_b.transpose()*(Task_K_imp*V_tilde_ref - F_ext);
    
    MM_JVec torques = M * qddot_ref + C * q_dot_ref + G + tau_ref;

    return torques;
}

MM_JVec CS_hyumm::TaskRedundantIDC(MM_JVec q, MM_JVec q_dot, MM_JVec dq, MM_JVec dq_dot, MM_JVec dq_ddot, SE3 T_des, Twist V_des, Twist V_dot_des)
{
    SE3 T_err = TransInv(T_ee)*T_des;
    SE3 invT_err = TransInv(T_err);
    
    Twist V_err = V_des - Ad(invT_err) * V_b;
    
    lambda = se3ToVec(MatrixLog6(T_err));
    lambda_dot = dlog6(-lambda) * V_err;

    Twist lambda_ddot_ref = Task_Kv * lambda_dot + Task_Kp * lambda;

    Twist V_dot_ref = Ad(T_err) * (V_dot_des + (dexp6(-lambda) * lambda_ddot_ref) + ad(V_err) * V_des - (ddexp6(-lambda, -lambda_dot) * lambda_dot));

    MM_pinvJacobian invJb, invJw;
    invJb = J_b.transpose() * (J_b * J_b.transpose()).inverse();

    MM_JVec jointMatrix;
    jointMatrix << 1,1,1,0,0,0,0,0,0;

    JacobiSVD<MM_Jacobian, ColPivHouseholderQRPreconditioner> svd(J_b, ComputeFullV);
    MM_JMat V = svd.matrixV().transpose();

    Matrix<double, MOBILE_DOF_NUM, MM_DOF_NUM> Z, Z_dot, ZW;
    Matrix<double, MM_DOF_NUM, MOBILE_DOF_NUM> invZ, invZ_dot, invZw;
    Matrix3d Zn, invZn;
    Vector3d Zw_dot_ref;
    MM_JMat W, W_dot, invW;
    MM_JMat Wdiag = MM_JMat::Zero();
    W_dot = MM_JMat::Zero();

    Z = V.block<MOBILE_DOF_NUM,MM_DOF_NUM>(6,0);
  
    invZ = Z.transpose() * (Z*Z.transpose()).inverse();
    Zn = Z.block<MOBILE_DOF_NUM, MOBILE_DOF_NUM>(0,0);
    invZn = Zn.inverse();

    // Method 1 M scale
    // Z = Minv.block<MOBILE_DOF_NUM,MOBILE_DOF_NUM>(0,0) * invZ.block<MOBILE_DOF_NUM,MOBILE_DOF_NUM>(0,0) * Z;
    // invZ = Z.transpose() * (Z*Z.transpose()).inverse();
    // W = M * jointMatrix.asDiagonal() + J_b.transpose()*J_b;

    // // Method 2 ZZ
    Wdiag.block<MOBILE_DOF_NUM,MOBILE_DOF_NUM>(0,0) = invZn;
    W = Wdiag + J_b.transpose()*J_b;
    
    Z_dot = (-invJb * dJ_b * Z.transpose()).transpose();
    invZ_dot = -invZ * Z_dot * invZ;
    
    invW = W.transpose() * (W*W.transpose()).inverse();
    W_dot.block<MOBILE_DOF_NUM,MOBILE_DOF_NUM>(0,0) = invZ_dot.block<3,3>(0,0);
    W_dot += dJ_b.transpose()*J_b + J_b.transpose()*dJ_b;

    invJw = invW * J_b.transpose() * (J_b * invW * J_b.transpose()).inverse();
    invZw = Z.transpose() * (Z * W * Z.transpose()).inverse();

    Zw_dot_ref = dq_ddot.head<3>() + 100*(dq.head<3>()-q.head<3>()) + 10*(dq_dot.head<3>() - q_dot.head<3>());
    MM_JVec qddot_ref = invJw * (V_dot_ref - dJ_b * q_dot) + invZw* (Zw_dot_ref - (Z_dot*W+Z*W_dot)*q_dot);
    MM_JVec torques = M * qddot_ref + C * q_dot + G;

    return torques;
}

MM_JVec CS_hyumm::TaskRedundantImpedanceControl(MM_JVec q, MM_JVec q_dot, MM_JVec dq, MM_JVec dq_dot, MM_JVec dq_ddot, SE3 T_des, Twist V_des, Twist V_dot_des, Twist F_des, Twist F_ext)
{
    SE3 T_err = TransInv(T_ee)*T_des;
    SE3 invT_err = TransInv(T_err);
    
    Twist V_err = V_des - Ad(invT_err) * V_b;
    
    lambda = se3ToVec(MatrixLog6(T_err));
    lambda_int += lambda * period;

    lambda_dot = dlog6(-lambda) * V_err;

    F_eff = F_des - Ad(T_err).transpose()*F_ext;
    gamma = dexp6(-lambda).transpose()*F_eff;
    gamma_int += gamma *period;

    A_lambda = dexp6(-lambda).transpose() * A_ * dexp6(-lambda);
    D_lambda = dexp6(-lambda).transpose() * (D_ * dexp6(-lambda) + A_*ddexp6(-lambda, -lambda_dot));
    K_lambda = dexp6(-lambda).transpose() * K_ * dexp6(-lambda);

    Task_Kv_imp = dlog6(-lambda)*(A_.inverse()*D_*dexp6(-lambda) + ddexp6(-lambda, -lambda_dot));
    Task_Kp_imp = dlog6(-lambda)*A_.inverse()*K_*dexp6(-lambda);
    Task_Kgama_imp = dlog6(-lambda)*A_.inverse()*dlog6(-lambda).transpose();


    Twist lambda_ddot_ref = -Task_Kv_imp * lambda_dot - Task_Kp_imp * lambda + Task_Kgama_imp * gamma;

    Twist V_dot_ref = Ad(T_err) * (V_dot_des - (dexp6(-lambda) * lambda_ddot_ref) + ad(V_err) * V_des - (ddexp6(-lambda, -lambda_dot) * lambda_dot));
    
    MM_pinvJacobian invJb, invJw;
    invJb = J_b.transpose() * (J_b * J_b.transpose()).inverse();

    MM_JVec jointMatrix;
    jointMatrix << 1,1,1,0,0,0,0,0,0;

    JacobiSVD<MM_Jacobian, ColPivHouseholderQRPreconditioner> svd(J_b, ComputeFullV);
    MM_JMat V = svd.matrixV().transpose();

    Matrix<double, MOBILE_DOF_NUM, MM_DOF_NUM> Z, Z_dot, ZW;
    Matrix<double, MM_DOF_NUM, MOBILE_DOF_NUM> invZ, invZ_dot, invZw;
    Matrix3d Zn, invZn;
    Vector3d Zw_dot_ref;
    MM_JMat W, W_dot, invW;
    MM_JMat Wdiag = MM_JMat::Zero();
    W_dot = MM_JMat::Zero();

    Z = V.block<MOBILE_DOF_NUM,MM_DOF_NUM>(6,0);
  
    invZ = Z.transpose() * (Z*Z.transpose()).inverse();
    Zn = Z.block<MOBILE_DOF_NUM, MOBILE_DOF_NUM>(0,0);
    invZn = Zn.inverse();

    // Method 1 M scale
    // Z = Minv.block<MOBILE_DOF_NUM,MOBILE_DOF_NUM>(0,0) * invZ.block<MOBILE_DOF_NUM,MOBILE_DOF_NUM>(0,0) * Z;
    // invZ = Z.transpose() * (Z*Z.transpose()).inverse();
    // W = M * jointMatrix.asDiagonal() + J_b.transpose()*J_b;

    // // Method 2 ZZ
    Wdiag.block<MOBILE_DOF_NUM,MOBILE_DOF_NUM>(0,0) = invZn;
    W = Wdiag + J_b.transpose()*J_b;
    
    Z_dot = (-invJb * dJ_b * Z.transpose()).transpose();
    invZ_dot = -invZ * Z_dot * invZ;
    
    invW = W.transpose() * (W*W.transpose()).inverse();
    W_dot.block<MOBILE_DOF_NUM,MOBILE_DOF_NUM>(0,0) = invZ_dot.block<3,3>(0,0);
    W_dot += dJ_b.transpose()*J_b + J_b.transpose()*dJ_b;

    invJw = invW * J_b.transpose() * (J_b * invW * J_b.transpose()).inverse();
    invZw = Z.transpose() * (Z * W * Z.transpose()).inverse();

    // apperent inertia, spring, damper
    Matrix3d m_ = Matrix3d::Zero();
    Vector3d k_, d_;

    m_.diagonal()<< 4, 4, 4;
    k_ << 120.0, 120.0, 120.0;
    d_ << 80.0, 80.0, 80.0;
    // for(int i=0; i<3; i++)
    //     d_(i) = 2*sqrt(k_(i)*m_(i,i));

    double a0=1.0/100.0;
    double b0=1.0;
    double b = b0*(1-exp(-pow(J_b.block<6,6>(0,3).determinant(),2)/pow(a0,2)));
    // printf("b: %lf\n", b);
    
    Zw_dot_ref = dq_ddot.head<3>() + m_.inverse()*(k_.cwiseProduct(dq.head<3>()-q.head<3>()) + d_.cwiseProduct(dq_dot.head<3>() - q_dot.head<3>()) + J_b.block<6,3>(0,0).transpose()*F_ext);
    MM_JVec qddot_ref = invJb * (V_dot_ref - dJ_b * q_dot) + b* invZw* (Zw_dot_ref - (Z_dot*W+Z*W_dot)*q_dot);
    MM_JVec torques = M * qddot_ref + C * q_dot + G;// + J_b.transpose()*F_ext;

    return torques;
}

MM_JVec CS_hyumm::TaskStablePD(SE3 T_des, Twist V_des, Twist V_dot_des, MM_JVec q, MM_JVec q_dot, MM_JVec dq, MM_JVec dq_dot, MM_JVec dq_ddot)
{
    SE3 T_err = TransInv(T_ee)*T_des;
    SE3 invT_err = TransInv(T_err);
    Matrix6d AdInvT = Ad(invT_err);

    Twist V_err = V_des - AdInvT * V_b;
    lambda = se3ToVec(MatrixLog6(T_err));
    lambda_dot = dlog6(-lambda) * V_err;

    MM_pinvJacobian JT = J_b.transpose();
    Matrix6d Kv_bar = Task_Kv*period*dlog6(-lambda);

    double a0=1/2.50;
    double b0=75.0;
    double b = b0*exp(-pow(J_b.determinant(),2)/pow(a0,2));
    Twist Ke = Task_Kp*(lambda+lambda_dot*period);
    double fcut = 10.0;
    MM_JVec qddot;


    MM_pinvJacobian invJb, invJw;
    invJb = J_b.transpose() * (J_b * J_b.transpose()).inverse();

    MM_JVec jointMatrix;
    jointMatrix << 1,1,1,0,0,0,0,0,0;

    JacobiSVD<MM_Jacobian, ColPivHouseholderQRPreconditioner> svd(J_b, ComputeFullV);
    MM_JMat V = svd.matrixV().transpose();

    Matrix<double, MOBILE_DOF_NUM, MM_DOF_NUM> Z, Z_dot, ZW;
    Matrix<double, MM_DOF_NUM, MOBILE_DOF_NUM> invZ, invZ_dot, invZw;
    Matrix3d Zn, invZn;
    Vector3d Zw_dot_ref;
    MM_JMat W, W_dot, invW;
    MM_JMat Wdiag = MM_JMat::Zero();
    W_dot = MM_JMat::Zero();

    Z = V.block<MOBILE_DOF_NUM,MM_DOF_NUM>(6,0);
  
    invZ = Z.transpose() * (Z*Z.transpose()).inverse();
    Zn = Z.block<MOBILE_DOF_NUM, MOBILE_DOF_NUM>(0,0);
    invZn = Zn.inverse();

    // Method 1 M scale
    // Z = Minv.block<MOBILE_DOF_NUM,MOBILE_DOF_NUM>(0,0) * invZ.block<MOBILE_DOF_NUM,MOBILE_DOF_NUM>(0,0) * Z;
    // invZ = Z.transpose() * (Z*Z.transpose()).inverse();
    // W = M * jointMatrix.asDiagonal() + J_b.transpose()*J_b;

    // // Method 2 ZZ
    Wdiag.block<MOBILE_DOF_NUM,MOBILE_DOF_NUM>(0,0) = invZn;
    W = Wdiag + J_b.transpose()*J_b;
    
    Z_dot = (-invJb * dJ_b * Z.transpose()).transpose();
    invZ_dot = -invZ * Z_dot * invZ;
    
    invW = W.transpose() * (W*W.transpose()).inverse();
    W_dot.block<MOBILE_DOF_NUM,MOBILE_DOF_NUM>(0,0) = invZ_dot.block<3,3>(0,0);
    W_dot += dJ_b.transpose()*J_b + J_b.transpose()*dJ_b;

    invJw = invW * J_b.transpose() * (J_b * invW * J_b.transpose()).inverse();
    invZw = Z.transpose() * (Z * W * Z.transpose()).inverse();

    // apperent inertia, spring, damper
    Matrix3d m_ = Matrix3d::Zero();
    Vector3d k_, d_;

    m_.diagonal()<< 4, 4, 4;
    k_ << 120.0, 120.0, 120.0;
    d_ << 80.0, 80.0, 80.0;
    // for(int i=0; i<3; i++)
    //     d_(i) = 2*sqrt(k_(i)*m_(i,i));
    
    Zw_dot_ref = dq_ddot.head<3>() + m_.inverse()*(k_.cwiseProduct(dq.head<3>()-q.head<3>()) + d_.cwiseProduct(dq_dot.head<3>() - q_dot.head<3>()));

    // if (Ke.norm()<fcut)
    // {
        qddot = (M+JT*Kv_bar*AdInvT*J_b).inverse()*(JT*(Task_Kp*(lambda+lambda_dot*period)+Task_Kv*lambda_dot
                    +Kv_bar*(V_dot_des-AdInvT*dJ_b*dq + ad(V_err)*V_des-ddexp6(-lambda,-lambda_dot)*lambda_dot)) -b*dq -C*dq) + invZw* (Zw_dot_ref - (Z_dot*W+Z*W_dot)*q_dot);
    // }
    // else
    // {
    //     JVec err_tmp = lambda+lambda_dot*period;
    //     qddot = (M+JT*Kv_bar*AdInvT*J_b).inverse()*(JT*(fcut*err_tmp/err_tmp.norm()+Task_Kv*lambda_dot
    //                 +Kv_bar*(V_dot_des-AdInvT*dJ_b*dq + ad(V_err)*V_des-ddexp6(-lambda,-lambda_dot)*lambda_dot)) -b*dq -C*dq);
    // }

    return qddot;
}


void CS_hyumm::TaskAdmittance(SE3 T_des, Twist V_des, Twist V_dot_des, SE3 &T_adm, Twist &V_adm, Twist &V_dot_adm, Twist F_des, Twist F_ext)
{
    SE3 T_err = TransInv(T_ref)*T_des;
    SE3 invT_err = TransInv(T_err);
    Matrix6d AdInvT = Ad(invT_err);

    Twist V_err = V_des - AdInvT * V_ref;
    lambda = se3ToVec(MatrixLog6(T_err));
    lambda_dot = dlog6(-lambda) * V_err;

    F_eff = F_des - Ad(T_err).transpose()*F_ext;
    gamma = dexp6(-lambda).transpose()*F_eff;
    gamma_int += gamma *period;

    A_lambda = dexp6(-lambda).transpose() * A_ * dexp6(-lambda);
    D_lambda = dexp6(-lambda).transpose() * (D_ * dexp6(-lambda) + A_*ddexp6(-lambda, -lambda_dot));
    K_lambda = dexp6(-lambda).transpose() * K_ * dexp6(-lambda);

    Task_Kv_imp = dlog6(-lambda)*(A_.inverse()*D_*dexp6(-lambda) + ddexp6(-lambda, -lambda_dot));
    Task_Kp_imp = dlog6(-lambda)*A_.inverse()*K_*dexp6(-lambda);
    Task_Kgama_imp = dlog6(-lambda)*A_.inverse()*dlog6(-lambda).transpose();


    Twist lambda_ddot_ref = -Task_Kv_imp * lambda_dot - Task_Kp_imp * lambda + Task_Kgama_imp * gamma;
    V_dot_ref = Ad(T_err) * (V_dot_des - (dexp6(-lambda) * lambda_ddot_ref) + ad(V_err) * V_des - (ddexp6(-lambda, -lambda_dot) * lambda_dot));
    V_ref += V_dot_ref*period;
    lambda_dot = dlog6(-lambda)*V_ref*period;
    lambda += lambda_dot;
    T_ref = T_ref*MatrixExp6(VecTose3(lambda_dot));

    V_dot_adm =  V_dot_ref;
    V_adm = V_ref;
    T_adm = T_ref;
}

void CS_hyumm::resetTaskAdmittance()
{
    V_dot_ref = Twist::Zero();
    V_ref = V_b;
    T_ref = T_ee;
}

void CS_hyumm::saturationMaxTorque(MM_JVec &torque, MM_JVec MAX_TORQUES)
{
    for(int i =0;i<NRMK_DRIVE_NUM;i++){
        if(abs(torque(i))> MAX_TORQUES(i)){
            if(torque(i)>0) torque(i) = MAX_TORQUES(i);
            else torque(i) = -MAX_TORQUES(i);
        }
    }
}

MM_JVec CS_hyumm::HinfControl( MM_JVec q,MM_JVec dq,MM_JVec q_des,MM_JVec dq_des,MM_JVec ddq_des)
{
    MM_JVec e = q_des-q;
    MM_JVec edot = dq_des-dq;
    
    eint = eint + e*period;	
    
    if(isUpdated)
    {
        MM_JVec ddq_ref = ddq_des+Hinf_Kv*edot+Hinf_Kp*e;
        MM_JVec dq_ref = dq_des;
        tau = M*ddq_ref+C*dq_ref+G+(Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint);
        isUpdated = false;
    }
    else
    {
        M = computeM(q);
        C = computeC(q, dq);
        G = computeG(q);
        MM_JVec ddq_ref = ddq_des+Hinf_Kv*edot+Hinf_Kp*e;
        MM_JVec dq_ref = dq_des;
        tau = M*ddq_ref+C*dq_ref+G+(Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint);
    }
    return tau;
}

MM_JVec CS_hyumm::HinfControl( MM_JVec q,MM_JVec dq,MM_JVec q_des,MM_JVec dq_des,MM_JVec ddq_des, MM_JVec _tau_ext)
{
    MM_JVec e = q_des-q;
    MM_JVec edot = dq_des-dq;
    
    eint = eint + e*period;	
    tau_ext = _tau_ext;
    
    if(isUpdated)
    {
        MM_JVec ddq_ref = ddq_des+Hinf_Kv*edot+Hinf_Kp*e;
        MM_JVec dq_ref = dq_des;
        tau = M*ddq_ref+C*dq_ref+G+(Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint);
        isUpdated = false;
    }
    else
    {
        M = computeM(q);
        C = computeC(q, dq);
        G = computeG(q);
        MM_JVec ddq_ref = ddq_des+Hinf_Kv*edot+Hinf_Kp*e;
        MM_JVec dq_ref = dq_des;
        tau = M*ddq_ref+C*dq_ref+G+(Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint);
    }
    return tau;
}

MM_JVec CS_hyumm::NRIC(MM_JVec q_r, MM_JVec dq_r, MM_JVec q_n, MM_JVec dq_n)
{
    MM_JVec e = q_r-q_n;
    MM_JVec edot = dq_r - dq_n;
    eint = eint + e*period;	
    
    tau = NRIC_K_gamma * (edot + NRIC_Kp*e + NRIC_Ki*eint);
    computeAlpha(edot, tau);
    tau_bd = alpha*tau;
    tau = (1-alpha)*tau;


    return tau;
}
void CS_hyumm::computeAlpha(MM_JVec edot, MM_JVec tau_c)
{
    double edotc, edotx;
    edotc = edot.transpose() * tau_c;
    edotx = edot.transpose() * tau_ext;
    if(edotc>0 && edotx >0)
    {
        if(edotc<edotx)
            alpha=1.0;
        else
            alpha = edotx/edotc;
    }
    else
        alpha = 0;
}