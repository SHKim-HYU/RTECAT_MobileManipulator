#include "CS_hyumm.h"


CS_hyumm::CS_hyumm()
{
    robotModel = "hyumm";
    n_dof = 6;

    this->q.resize(this->n_dof);
    this->dq.resize(this->n_dof);
    this->ddq.resize(this->n_dof);
    this->M.resize(this->n_dof, this->n_dof);
    this->Minv.resize(this->n_dof, this->n_dof);
    this->C.resize(this->n_dof, this->n_dof);
    this->G.resize(this->n_dof);
    this->J_b.resize(6, this->n_dof);
    this->J_s.resize(6, this->n_dof);

    this->Kp.resize(this->n_dof, this->n_dof);
    this->Kv.resize(this->n_dof, this->n_dof);
    this->Ki.resize(this->n_dof, this->n_dof);

    this->Hinf_Kp.resize( this->n_dof, this->n_dof);
    this->Hinf_Kv.resize( this->n_dof, this->n_dof);
    this->Hinf_Ki.resize( this->n_dof, this->n_dof);
    this->Hinf_K_gamma.resize( this->n_dof, this->n_dof);

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

    this->q.resize(this->n_dof);
    this->dq.resize(this->n_dof);
    this->ddq.resize(this->n_dof);
    this->M.resize(this->n_dof, this->n_dof);
    this->Minv.resize(this->n_dof, this->n_dof);
    this->C.resize(this->n_dof, this->n_dof);
    this->G.resize(this->n_dof);
    this->J_b.resize(6, this->n_dof);
    this->J_s.resize(6, this->n_dof);

    this->Kp.resize(this->n_dof, this->n_dof);
    this->Kv.resize(this->n_dof, this->n_dof);
    this->Ki.resize(this->n_dof, this->n_dof);

    this->Hinf_Kp.resize( this->n_dof, this->n_dof);
    this->Hinf_Kv.resize( this->n_dof, this->n_dof);
    this->Hinf_Ki.resize( this->n_dof, this->n_dof);
    this->Hinf_K_gamma.resize( this->n_dof, this->n_dof);

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

    // string FD_path = loader_.getValue("forward_dynamics_path").asString();
    // string M_path = loader_.getValue("mass_matrix_path").asString();
    // string Minv_path = loader_.getValue("mass_inverse_matrix_path").asString();
    // string C_path = loader_.getValue("coriolis_path").asString();
    // string G_path = loader_.getValue("gravity_path").asString();
    // string FK_path = loader_.getValue("forward_kinematics_path").asString();
    // string Js_path = loader_.getValue("MM_Jacobian_space_path").asString();
    // string Jb_path = loader_.getValue("MM_Jacobian_body_path").asString();

    // fd_cs = casadi::Function::load(FD_path);
    // M_cs = casadi::Function::load(M_path);
    // Minv_cs = casadi::Function::load(Minv_path);
    // C_cs = casadi::Function::load(C_path);
    // G_cs = casadi::Function::load(G_path);
    // J_s_cs = casadi::Function::load(FK_path);
    // J_b_cs = casadi::Function::load(Js_path);
    // FK_cs = casadi::Function::load(Jb_path);

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
    J_com_eval = (eval_t)dlsym(J_com_handle, "jacobianCenterOfMass");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }

}

void CS_hyumm::setPIDgain(MM_JVec _Kp, MM_JVec _Kd, MM_JVec _Ki)
{
    for (int i=0; i<this->n_dof; ++i)
    {
        Kp(i,i) = _Kp(i);
        Kv(i,i) = _Kd(i);
        Ki(i,i) = _Ki(i);
    }
}

void CS_hyumm::setHinfgain(MM_JVec _Hinf_Kp, MM_JVec _Hinf_Kd, MM_JVec _Hinf_Ki, MM_JVec _Hinf_K_gamma)
{
    for (int i=0; i<this->n_dof; ++i)
    {
        Hinf_Kp(i,i) = _Hinf_Kp(i);
        Hinf_Kv(i,i) = _Hinf_Kd(i);
        Hinf_Ki(i,i) = _Hinf_Ki(i);
        Hinf_K_gamma(i,i) = _Hinf_K_gamma(i);
    }
}

void CS_hyumm::setNRICgain(MM_JVec _NRIC_Kp, MM_JVec _NRIC_Ki, MM_JVec _NRIC_K_gamma)
{
    for (int i=0; i<this->n_dof; ++i)
    {
        NRIC_Kp(i,i) = _NRIC_Kp(i);
        NRIC_Ki(i,i) = _NRIC_Ki(i);
        NRIC_K_gamma(i,i) = _NRIC_K_gamma(i);
    }
}

void CS_hyumm::updateRobot(MM_JVec _q, MM_JVec _dq)
{
    M = computeM(_q);
    Minv = computeMinv(_q);
    C = computeC(_q, _dq);
    G = computeG(_q); 

    J_b = computeJ_b(_q);

    T_ee = computeFK(_q);


    isUpdated=true;   
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

void CS_hyumm::computeRK45(MM_JVec _q, MM_JVec _dq, MM_JVec _tau, MM_JVec &_q_nom, MM_JVec &_dq_nom)
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

    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < sz_res; ++j) {   
            J_com(j,i) = output_values[i * sz_res + j];
            cout<<J_com(j,i)<<"  ";
        }
        cout<<endl;
    }

    return J_com;
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

    return T_ee;
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
    
    if (J_b_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < sz_res; ++j) {   
            J_b(j,i) = output_values[i * sz_res + j];
        }
    }

    return J_b;
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
    
    if (J_s_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < sz_res; ++j) {   
            J_s(j,i) = output_values[i * sz_res + j];
        }
    }

    return J_s;
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
SE3 CS_hyumm::getFK()
{
    return T_ee;
}
MM_Jacobian CS_hyumm::getJ_b()
{
    return J_b;
}
MM_Jacobian CS_hyumm::getJ_s()
{
    return J_s;
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

MM_JVec CS_hyumm::ComputedTorqueControl( MM_JVec q,MM_JVec dq,MM_JVec q_des,MM_JVec dq_des,MM_JVec ddq_des, MM_JVec tau_ext)
{
    MM_JVec e = q_des-q;
    MM_JVec edot = dq_des-dq;
    double m_gam = 1;
    
    eint = eint + e*period;	
    
    if(isUpdated)
    {
        MM_JVec ddq_ref = ddq_des + m_gam*Kv*edot + m_gam*Kp*e + m_gam*tau_ext;
        tau = M*ddq_ref + C*dq + G + tau_ext;
        isUpdated = false;
    }
    else
    {
        M = computeM(q);
        C = computeC(q, dq);
        G = computeG(q);
        MM_JVec ddq_ref = ddq_des + m_gam*Kv*edot + m_gam*Kp*e + m_gam*tau_ext;
        tau = M*ddq_ref+C*dq+G + tau_ext;
    }
    return tau;   
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

MM_JVec CS_hyumm::HinfControl( MM_JVec q,MM_JVec dq,MM_JVec q_des,MM_JVec dq_des,MM_JVec ddq_des, MM_JVec tau_ext)
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

MM_JVec CS_hyumm::NRIC(MM_JVec q_r, MM_JVec dq_r, MM_JVec q_n, MM_JVec dq_n)
{
    MM_JVec e = q_r-q_n;
    MM_JVec edot = dq_r - dq_n;
    eint = eint + e*period;	
    
    tau = NRIC_K_gamma * (edot + NRIC_Kp*e + NRIC_Ki*eint);

    return tau;
}