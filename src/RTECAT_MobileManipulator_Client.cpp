#include "RTECAT_MobileManipulator_Client.h"

MOB_ROBOT_INFO info_mob;
MM_ROBOT_INFO info_mm;

CS_Indy7 cs_indy7;
CS_hyumm cs_hyumm;
CS_hyumm cs_nom_hyumm;

RT_TASK safety_task;
RT_TASK motor_task;
RT_TASK bullet_task;
RT_TASK print_task;
RT_TASK odom_writer;
RT_TASK cmd_vel_listener;

using namespace std;
using namespace lr;


static void fail(const char *reason)
{
	perror(reason);
	exit(EXIT_FAILURE);
}

bool isSlaveInit()
{
    // Mobile Drive Servo on
    for(int i=0; i<MOBILE_DRIVE_NUM; i++)
    {
        if(!ecat_iservo[i].isSystemReady())
            return false;
    }

    // Indy Drive Servo on
    for(int i=0; i<NRMK_DRIVE_NUM; i++)
    {
        if(!ecat_drive[i].isSystemReady())
            return false;
    }


    return true;
}

int initAxes()
{
    // Mobile Drive Init Axes
	for (int i = 0; i < MOBILE_DRIVE_NUM; i++)
	{	
		Axis_Motor[i].setGearRatio(gearRatio_mob[i]);
		Axis_Motor[i].setGearEfficiency(EFFICIENCY);
		Axis_Motor[i].setPulsePerRevolution(ecat_master.SDO_ENCODER_RESOLUTION(i+OFFSET_NUM));
		Axis_Motor[i].setTauRateCur(((double)ecat_master.SDO_RATE_CURRENT(i+OFFSET_NUM))/1000.0);
		Axis_Motor[i].setTauK(((double)ecat_master.SDO_TORQUE_CONSTANT(i+OFFSET_NUM))/1000000.0);
		Axis_Motor[i].setZeroPos(zeroPos_mob[i]);

		Axis_Motor[i].setDirQ(dirQ_mob[i]);
		Axis_Motor[i].setDirTau(dirTau_mob[i]);

		Axis_Motor[i].setConversionConstants();

		Axis_Motor[i].setTrajPeriod(period);
		
		Axis_Motor[i].setTarVelInRPM(0);
		Axis_Motor[i].setTarTorInCnt(0);

        info_mob.des.e = Mob_JVec::Zero();
        info_mob.des.eint = Mob_JVec::Zero();
		des_int = MM_JVec::Zero();
	}

    // Arm Drive Init Axes
	for (int i = 0; i < NRMK_DRIVE_NUM; i++)
	{	
		Axis_Core[i].setGearRatio(gearRatio_arm[i]);
		Axis_Core[i].setGearEfficiency(EFFICIENCY);
		Axis_Core[i].setPulsePerRevolution(ENC_CORE);
		Axis_Core[i].setTauADC(TauADC_arm[i]);
		Axis_Core[i].setTauK(TauK_arm[i]);
		Axis_Core[i].setZeroPos(zeroPos_arm[i]);
		Axis_Core[i].setVelLimits(qdotLimit[i], -qdotLimit[i]);

		Axis_Core[i].setDirQ(dirQ_arm[i]);
		Axis_Core[i].setDirTau(dirTau_arm[i]);

		Axis_Core[i].setConversionConstants();

		Axis_Core[i].setTrajPeriod(period);
		
		Axis_Core[i].setTarVelInCnt(0);
		Axis_Core[i].setTarTorInCnt(0);
	}

	// Mobile Manipulator Init Axes
	for (int i = 0; i < MM_DOF_NUM; i++)
	{	
		Axis_MM[i].setTrajPeriod(period);
		
		Axis_MM[i].setTarVelInCnt(0);
		Axis_MM[i].setTarTorInCnt(0);
	}

	// Mecanum Mobile Base Jacobian
	Jinv_mob << 1.0, 1.0, -BASE_l-BASE_w,
				1.0, -1.0, -BASE_l-BASE_w,
				1.0, 1.0, BASE_l+BASE_w, 
				1.0, -1.0, BASE_l+BASE_w;
	Jinv_mob = Jinv_mob/WHEEL_RADIUS;

	J_mob << 	1.0, 1.0, 1.0, 1.0,
				1.0, -1.0, 1.0, -1.0,
				 1.0/(-BASE_l-BASE_w), 1.0/(-BASE_l-BASE_w), 1.0/(BASE_l+BASE_w), 1.0/(BASE_l+BASE_w);
	J_mob = J_mob*WHEEL_RADIUS/4.0;


	return 1;
}

void readData()
{
    // ecat_master.Motor_STATE(info_mob.q_inc, info_mob.dq_inc, info_mob.tau_per, info_mob.statusword, info_mob.modeofop);
    
	ecat_master.TxUpdate();
    for(int i=0; i<MOBILE_DRIVE_NUM;i++)
    {

        Axis_Motor[i].setCurrentPosInCnt(ecat_iservo[i].position_);
        Axis_Motor[i].setCurrentVelInRPM(ecat_iservo[i].velocity_);
        Axis_Motor[i].setCurrentTorInCnt(ecat_iservo[i].torque_);
        
        Axis_Motor[i].setCurrentTime(gt);

        info_mob.act.q(i) = Axis_Motor[i].getCurrPosInRad();
		if (fabs(Axis_Motor[i].getCurrVelInRad())<10.0)
		{
			info_mob.act.q_dot(i) = Axis_Motor[i].getCurrVelInRad();
		}
		else
		{
			act_max[i] = fabs(Axis_Motor[i].getCurrVelInRad());
		}
        info_mob.act.tau(i) = Axis_Motor[i].getCurrTorInNm();

        // For Inital target
        if(!system_ready)
        {
            Axis_Motor[i].setTarPosInRad(info_mob.act.q(i));
            Axis_Motor[i].setDesPosInRad(info_mob.act.q(i));
        }

    }
    for(int i=0; i<NRMK_DRIVE_NUM;i++)
	{

		Axis_Core[i].setCurrentPosInCnt(ecat_drive[i].position_);
		Axis_Core[i].setCurrentVelInCnt(ecat_drive[i].velocity_);
		Axis_Core[i].setCurrentTorInCnt(ecat_drive[i].torque_);
		
		Axis_Core[i].setCurrentTime(gt);

		info_mm.act.q(MOBILE_DOF_NUM+i) = Axis_Core[i].getCurrPosInRad();
		info_mm.act.q_dot(MOBILE_DOF_NUM+i) = Axis_Core[i].getCurrVelInRad();
		info_mm.act.tau(MOBILE_DOF_NUM+i) = Axis_Core[i].getCurrTorInNm();

		Axis_MM[MOBILE_DOF_NUM+i].setCurrentPosInConf(info_mm.act.q(MOBILE_DOF_NUM+i));
		Axis_MM[MOBILE_DOF_NUM+i].setCurrentVelInConf(info_mm.act.q_dot(MOBILE_DOF_NUM+i));
		Axis_MM[MOBILE_DOF_NUM+i].setCurrentTime(gt);
		
		if(!system_ready)
		{
			Axis_Core[i].setTarPosInRad(info_mm.act.q(MOBILE_DOF_NUM+i));
			Axis_Core[i].setDesPosInRad(info_mm.act.q(MOBILE_DOF_NUM+i));
			info_mm.nom.q(MOBILE_DOF_NUM+i) = info_mm.act.q(MOBILE_DOF_NUM+i);
			info_mm.nom.q_dot(MOBILE_DOF_NUM+i) = info_mm.act.q_dot(MOBILE_DOF_NUM+i);
			info_mm.nom.tau(MOBILE_DOF_NUM+i) = info_mm.act.tau(MOBILE_DOF_NUM+i);
		}

	}

	// Mapping status to Mobile Manipulator
	info_mob.act.x_dot = Einv_mob * J_mob * info_mob.act.q_dot;
	info_mob.act.x += info_mob.act.x_dot*period;

	info_mm.act.q_dot.segment<MOBILE_DOF_NUM>(0) = info_mob.act.x_dot;
	info_mm.act.q.segment<MOBILE_DOF_NUM>(0) = info_mob.act.x;

	for(int i=0; i<MOBILE_DOF_NUM+NRMK_DRIVE_NUM; i++)
	{
		Axis_MM[i].setCurrentPosInConf(info_mm.act.q(i));
		Axis_MM[i].setCurrentVelInConf(info_mm.act.q_dot(i));

		Axis_MM[i].setCurrentTime(gt);
		
		if(!system_ready)
		{
			Axis_MM[i].setTarPosInRad(info_mm.act.q(i));
			Axis_MM[i].setDesPosInRad(info_mm.act.q(i));
			info_mm.nom.q = info_mm.act.q;
			info_mm.nom.q_dot = info_mm.act.q_dot;
			info_mm.nom.tau = info_mm.act.tau;
		}
	}

	for(int i=0; i<NRMK_TOOL_NUM; i++)
	{
		// Update RFT data
		if (ecat_tool[i].FT_Raw_F[0] ==0 && ecat_tool[i].FT_Raw_F[1] ==0 && ecat_tool[i].FT_Raw_F[2] ==0)
		{
			F_tmp = Twist::Zero();
		}
		else{
			for(int j=0; j<3; j++)
			{
				F_tmp(j) = (double)ecat_tool[i].FT_Raw_F[j] / force_divider - ft_offset[j];
				F_tmp(j+3) = (double)ecat_tool[i].FT_Raw_T[j] / torque_divider - ft_offset[j+3];
			}
		}
	}
}

/****************************************************************************/
void trajectory_generation(){
	/////////////Trajectory for Joint Space//////////////
    if(!Axis_MM[0].trajInitialized())
    {
	    switch(motion)
	    {
	    case 1:
			info_mm.q_target(0)=0.0; info_mm.q_target(1)=0.0; info_mm.q_target(2)=0.0;
			info_mm.q_target(3)=0.0; info_mm.q_target(4)=0.707; info_mm.q_target(5)=-1.5709;
			info_mm.q_target(6)=0.0; info_mm.q_target(7)=-0.707; info_mm.q_target(8)=0.0;
			info_mm.q_target(3)=0.0; info_mm.q_target(4)=0.0; info_mm.q_target(5)=-1.5709;
			info_mm.q_target(6)=0.0; info_mm.q_target(7)=-1.5709; info_mm.q_target(8)=0.0;
			// info_mm.q_target(3)=0.0; info_mm.q_target(4)=-0.56; info_mm.q_target(5)=-2.14;
			// info_mm.q_target(6)=0.0; info_mm.q_target(7)=1.13; info_mm.q_target(8)=0.0;
			// info_mm.q_target(3)=0.0; info_mm.q_target(4)=0.0; info_mm.q_target(5)=0.0;
			// info_mm.q_target(6)=0.0; info_mm.q_target(7)=0.0; info_mm.q_target(8)=0.0;
	    	traj_time = 3;
			modeControl =1;
	    	motion++;
			// motion=1;
	        break;

		case 2:
			cs_nom_hyumm.resetTaskAdmittance();
	    	info_mm.T_init = info_mm.nom.T;
			info_mm.T_target << 0.0, -0, 1.0, 1.072989,
							0.0, 1.0, 0, -0.177496,
							-1.0, 0.0, 0.0, 0.67077,
							0, 0, 0, 1;
			// info_mm.T_target << -1, 1.40483e-21, 1.26536e-05, 0.350005, 
 			// 				1.40484e-21, 1, 1.40483e-21, -0.1865, 
 			// 				-1.26536e-05, 1.40484e-21, -1, 0.502002, 
 			// 				0, 0, 0, 1;
			traj_time = 10;	
			modeControl=2;
			motioncnt = 0;
			// motion++;
			motion=4;
			break;
		case 3:
	    	info_mm.T_init = info_mm.nom.T;
			info_mm.T_target << 0.0, -0, 1.0, 0.872989,
							0.0, 1.0, 0, -0.177496,
							-1.0, 0.0, 0.0, 0.87077,
							0, 0, 0, 1;
			info_mm.T_target << -1, 1.40483e-21, 1.26536e-05, 0.350005, 
 							1.40484e-21, 1, 1.40483e-21, -0.1865, 
 							-1.26536e-05, 1.40484e-21, -1, 0.502002, 
 							0, 0, 0, 1;
			traj_time = 10;	
			modeControl=2;
			motioncnt = 0;
			motion=2;
			break;

		case 4: // Hold on Task motion
	    	
			break;	    

		default:
			info_mm.q_target(0)=info_mm.act.q(0); info_mm.q_target(1)=info_mm.act.q(1); info_mm.q_target(2)=info_mm.act.q(2);
			info_mm.q_target(3)=info_mm.act.q(3); info_mm.q_target(4)=info_mm.act.q(4); info_mm.q_target(5)=info_mm.act.q(5);
			info_mm.q_target(6)=info_mm.act.q(6); info_mm.q_target(7)=info_mm.act.q(7); info_mm.q_target(8)=info_mm.act.q(8);

	    	motion=1;
	    	break;
	    }
	}

	for(int i=0;i<MM_DOF_NUM;i++)
	{
		if(!Axis_MM[i].trajInitialized())
		{
			Axis_MM[i].setTrajInitialQuintic();
			Axis_MM[i].setTarPosInRad(info_mm.q_target(i));
			Axis_MM[i].setTarVelInRad(0);
			Axis_MM[i].setTrajTargetQuintic(traj_time);
		}

		Axis_MM[i].TrajQuintic();
		if(modeControl==1)
		{
			info_mm.des.q(i)=Axis_MM[i].getDesPosInRad();
			info_mm.des.q_dot(i)=Axis_MM[i].getDesVelInRad();
			info_mm.des.q_ddot(i)=Axis_MM[i].getDesAccInRad();
		}
	}

	// 3. Lie Screw Trajectory --> [ToDo] It will be replaced to liescrewtraj function
	if(!motioncnt) 
	{			
		gt_offset = gt;
		motioncnt++;
	}
	LieScrewTrajectory(info_mm.T_init, info_mm.T_target, info_mm.V_init, info_mm.V_target, info_mm.V_init, info_mm.V_target, traj_time, gt-gt_offset, info_mm.des.T, info_mm.des.x_dot, info_mm.des.x_ddot);
}

void compute()
{
	// Update Mobile Manipulator
	cs_hyumm.updateRobot(info_mm.act.q, info_mm.act.q_dot);
	// Update nominal
	cs_nom_hyumm.updateRobot(info_mm.nom.q , info_mm.nom.q_dot);
	// Update Indy7
	cs_indy7.updateRobot(info_mm.nom.q.tail(6), info_mm.nom.q_dot.tail(6));
	
	info_mm.act.CoM_x = cs_hyumm.getCoM();
	info_mm.nom.J_com = cs_nom_hyumm.getJ_com();
	Vector3d v_com = info_mm.nom.J_com*info_mm.nom.q_dot;
	// rt_printf("v_com: %lf, %lf, %lf\n", v_com(0), v_com(1), v_com(2));
	info_mm.act.T = cs_hyumm.getFK();
	info_mm.act.R = cs_hyumm.getRMat();
	info_mm.nom.T = cs_nom_hyumm.getFK();
	info_mm.nom.R = cs_nom_hyumm.getRMat();

	MM_Jacobian J_b_mm = cs_hyumm.getJ_b();
	MM_Jacobian dJ_b_mm = cs_hyumm.getJdot_b();
	info_mm.act.x_dot = cs_hyumm.getBodyTwist();
	info_mm.act.x_ddot = dJ_b_mm*info_mm.nom.q_dot + J_b_mm*info_mm.nom.q_ddot;
		
	Twist F_mometum = cs_hyumm.computeF_Tool(info_mm.act.x_dot, info_mm.act.x_ddot);
	
	// info_mm.act.F = F_tmp-F_mometum;
	info_mm.act.F = cs_hyumm.computeF_Threshold(F_tmp);
	info_mm.act.tau_ext = J_b_mm.transpose()*(info_mm.act.F);

	info_mm.act.tau_fric = cs_hyumm.FrictionEstimation(info_mm.act.q_dot);
	// info_mm.act.tau_fric = JVec::Zero();

	E_mob << cos(info_mm.nom.q(2)), sin(info_mm.nom.q(2)), 0,
			-sin(info_mm.nom.q(2)), cos(info_mm.nom.q(2)), 0,
			0, 0, 1;

	Einv_mob = E_mob.transpose();

	manipulability = cs_indy7.getManipulability();
}

void control()
{	
	if(modeControl==0)
	{
		// info_mm.des.tau = cs_hyumm.computeG(info_mm.act.q);
		MM_JVec G_damp = {2, 2, 2, 0.5, 0.5, 0.5, 1.0, 1.0, 0.05};
		info_mm.nom.tau = cs_nom_hyumm.computeG(info_mm.nom.q) - G_damp.cwiseProduct(info_mm.nom.q_dot) + info_mm.act.tau_ext;
		// [Simulation]
		cs_nom_hyumm.computeRK45(info_mm.nom.q, info_mm.nom.q_dot, info_mm.nom.tau, info_mm.nom.q, info_mm.nom.q_dot, info_mm.nom.q_ddot);
			
		// [NRIC]
		info_mm.act.tau_aux = cs_hyumm.NRIC(info_mm.act.q, info_mm.act.q_dot, info_mm.nom.q, info_mm.nom.q_dot);
		// info_mm.des.tau = info_mm.nom.tau - info_mm.act.tau_aux + info_mm.act.tau_ext;
		info_mm.des.tau = info_mm.nom.tau - info_mm.act.tau_aux;
	}
	else if(modeControl==1)
	{
		// [Joint Space Nominal Controller]
		// info_mm.nom.tau = cs_nom_hyumm.ComputedTorqueControl(info_mm.nom.q, info_mm.nom.q_dot, info_mm.des.q, info_mm.des.q_dot, info_mm.des.q_ddot);
    	info_mm.nom.tau = cs_nom_hyumm.ComputedTorqueControl(info_mm.nom.q, info_mm.nom.q_dot, info_mm.des.q, info_mm.des.q_dot, info_mm.des.q_ddot, info_mm.act.tau_ext);
		// info_mm.nom.tau = cs_nom_hyumm.PassivityInverseDynamicControl(info_mm.nom.q, info_mm.nom.q_dot, info_mm.des.q, info_mm.des.q_dot, info_mm.des.q_ddot);

		// [Simulation]
		cs_nom_hyumm.computeRK45(info_mm.nom.q, info_mm.nom.q_dot, info_mm.nom.tau, info_mm.nom.q, info_mm.nom.q_dot, info_mm.nom.q_ddot);
			
		// [NRIC]
		info_mm.act.tau_aux = cs_hyumm.NRIC(info_mm.act.q, info_mm.act.q_dot, info_mm.nom.q, info_mm.nom.q_dot);
		info_mm.des.tau = info_mm.nom.tau - info_mm.act.tau_aux;
	}
	else if(modeControl==2)
	{
		des_int += info_mm.des.q_dot*period;
		info_mm.des.q = des_int;

		// [Task Space Nominal Controller]
				
		// info_mm.nom.tau = cs_nom_hyumm.TaskInverseDynamicsControl(info_mm.nom.q_dot, info_mm.des.T, info_mm.des.x_dot, info_mm.des.x_ddot);
		// info_mm.nom.tau = cs_nom_hyumm.TaskPassivityInverseDynamicsControl(info_mm.nom.q_dot, info_mm.des.T, info_mm.des.x_dot, info_mm.des.x_ddot);
		// info_mm.nom.tau = cs_nom_hyumm.TaskImpedanceControl(info_mm.nom.q_dot, info_mm.des.T, info_mm.des.x_dot, info_mm.des.x_ddot, Twist::Zero(), info_mm.act.F);
		// info_mm.nom.tau = cs_nom_hyumm.TaskPassivityImpedanceControl(info_mm.nom.q_dot, info_mm.des.T, info_mm.des.x_dot, info_mm.des.x_ddot, Twist::Zero(), info_mm.act.F);

		info_mm.nom.tau = cs_nom_hyumm.TaskRedundantImpedanceControl(info_mm.nom.q, info_mm.nom.q_dot, info_mm.des.q, info_mm.des.q_dot, info_mm.des.q_ddot, info_mm.des.T, info_mm.des.x_dot, info_mm.des.x_ddot, Twist::Zero(), info_mm.act.F);
		// info_mm.nom.tau = cs_nom_hyumm.TaskRedundantIDC(info_mm.nom.q, info_mm.nom.q_dot, info_mm.des.q, info_mm.des.q_dot, info_mm.des.q_ddot, info_mm.des.T, info_mm.des.x_dot, info_mm.des.x_ddot);

		// [Simulation]
		cs_nom_hyumm.computeRK45(info_mm.nom.q, info_mm.nom.q_dot, info_mm.nom.tau, info_mm.nom.q, info_mm.nom.q_dot, info_mm.nom.q_ddot);
			
		// [NRIC]
		info_mm.act.tau_aux = cs_hyumm.NRIC(info_mm.act.q, info_mm.act.q_dot, info_mm.nom.q, info_mm.nom.q_dot);
		info_mm.des.tau = info_mm.nom.tau - info_mm.act.tau_aux;
		
	}
	else if(modeControl==3)
	{
		// [Task Space Nominal Controller]
		if(gt>5)
			// info_mm.des.F << 0, 0, -10, 0, 0, 0;
			info_mm.des.F << 0, 0, 0, 0, 0, 0;
		// info_mm.act.F << 0, 0, 0, 0, 0, 0;

		cs_nom_hyumm.TaskAdmittance(info_mm.des.T, info_mm.des.x_dot, info_mm.des.x_ddot, info_mm.sim.T, info_mm.sim.x_dot, info_mm.sim.x_ddot, info_mm.des.F, info_mm.act.F);
		info_mm.nom.q_ddot = cs_nom_hyumm.TaskStablePD(info_mm.sim.T, info_mm.sim.x_dot, info_mm.sim.x_ddot, info_mm.nom.q, info_mm.nom.q_dot, info_mm.des.q, info_mm.des.q_dot, info_mm.des.q_ddot);

		// info_mm.nom.q_ddot = cs_nom_hyumm.TaskStablePD(info_mm.des.T, info_mm.des.x_dot, info_mm.des.x_ddot);
		// info_mm.nom.q_ddot = cs_nom_hyumm.TaskStablePDImpedance(info_mm.des.T, info_mm.des.x_dot, info_mm.des.x_ddot, info_mm.des.F, info_mm.act.F);
		info_mm.nom.q_dot += info_mm.nom.q_ddot*period;
		info_mm.nom.q += info_mm.nom.q_dot*period;
			
		// [NRIC]
		info_mm.act.tau_aux = cs_hyumm.NRIC(info_mm.act.q, info_mm.act.q_dot, info_mm.nom.q, info_mm.nom.q_dot);
		// info_mm.des.tau = info_mm.nom.tau - info_mm.act.tau_aux + info_mm.act.tau_ext;
		info_mm.des.tau = info_mm.nom.tau - info_mm.act.tau_aux;
		// info_mm.des.tau = info_mm.act.tau_aux;
	}

	

	// [Gravity Compensator]
	// Vector3d _com = {0,0,-0.124};
	// double _m = 2.5;
	// MM_JVec _addtor;
	// _addtor = cs_hyumm.addLoadedTorque(info_mm.act.q, _com, _m);
	// info_mm.des.tau = cs_hyumm.computeG(info_mm.act.q) + _addtor;
	// info_mm.des.tau = cs_hyumm.computeG(info_mm.act.q);

	double Kp = 0.1;
    double Kd = 0.01;
	double Ki = 0;
	double Mb = 100;

	// Mobile Base Controller
    for (int i = 0; i<MOBILE_DOF_NUM; i++)
    {
		info_mm.des.e(i) = info_mm.nom.q(i)-info_mm.act.q(i);
		info_mm.des.edot(i) = info_mm.nom.q_dot(i)-info_mm.act.q_dot(i);
		info_mm.des.eint(i) = info_mm.des.eint(i) + info_mm.des.e(i)*period;

		info_mm.des.tau(i) = info_mm.nom.q_dot(i) + Kd*info_mm.des.edot(i) + Kp*info_mm.des.e(i) + Ki*info_mm.des.eint(i);
    }
	info_mob.des.tau = Jinv_mob * E_mob * info_mm.des.tau.head(3);
	info_mob.des.q_dot = info_mob.des.tau;
}

void writeData()
{
    for(int i=1;i<=MOBILE_DRIVE_NUM;i++){
        if (ecat_iservo[i-1].mode_of_operation_ == ecat_iservo[i-1].MODE_CYCLIC_SYNC_TORQUE)
        {
            Axis_Motor[i-1].setDesTorInNm(info_mob.des.tau(i-1));
                
            INT16 temp = Axis_Motor[i-1].getDesTorInPer();
            // rt_printf("temp: %d\n", temp);
            if (temp > 2000)
            {
                temp = 2000;            
            }
            else if (temp<-2000)
            {
                temp = -2000;
            }
            ecat_iservo[i-1].writeTorque(temp);
        }
        else if (ecat_iservo[i-1].mode_of_operation_ == ecat_iservo[i-1].MODE_CYCLIC_SYNC_VELOCITY)
        {
            ecat_iservo[i-1].writeVelocity(Axis_Motor[i-1].getDesVelInRPM(info_mob.des.tau(i-1)));
        }
		ecat_master.RxUpdate();
	}

    for(int i=0;i<NRMK_DRIVE_NUM;i++){

        Axis_Core[i].setDesTorInNm(info_mm.des.tau(i+MOBILE_DOF_NUM));
            
        INT16 temp = Axis_Core[i].getDesTorInCnt();

        ecat_drive[i].writeTorque(temp);

        ecat_master.RxUpdate();
	}
    ecat_master.SyncEcatMaster(rt_timer_read());
}

void motor_run(void *arg)
{
    RTIME beginCycle, endCycle;
	RTIME beginCyclebuf;

	beginCyclebuf = 0;
   
    memset(&info_mob, 0, sizeof(MOB_ROBOT_INFO));
	memset(&info_mm, 0, sizeof(MM_ROBOT_INFO));

	int ft_init_cnt = 0;

	info_mm.des.q = MM_JVec::Zero();
	info_mm.des.q_dot = MM_JVec::Zero();
	info_mm.des.q_ddot = MM_JVec::Zero();
	info_mm.des.F = Vector6d::Zero();
	info_mm.des.F_CB = Vector6d::Zero();
	info_mm.act.tau_aux = MM_JVec::Zero();


	// Real
	NRIC_Kp << 50.0, 50.0, 50.0, 20.0, 25.0, 10.0, 3.0, 3.0, 1.5;
	NRIC_Ki << 8.0, 8.0, 8.0, 4.0, 5.0, 2.0, 0.6, 0.6, 0.3;
	// NRIC_K << 100, 100, 100, 80.0, 80.0, 50.0, 15.0, 15.0, 15.0;
	NRIC_K << 700.0, 700.0, 700.0, 550.0, 600.0, 450.0, 250.0, 250.0, 175.0;
	NRIC_gamma << 700.0, 700.0, 700.0, 550.0, 600.0, 450.0, 250.0, 250.0, 175.0;
	
	cs_hyumm.setNRICgain(NRIC_Kp, NRIC_Ki, NRIC_K,  NRIC_gamma);
	
	// nominal
	// for IDC
	Kp_n << 80.0, 80.0, 80.0, 50.0, 50.0, 30.0, 20.0, 20.0, 20.0;
	Kd_n << 55.0, 55.0, 55.0, 5.0, 5.0, 3.0, 2.0, 2.0, 2.0;
	K_n << 1.5,1.5,1.5,1,1,0.5,0.3,0.2,0.2;
	cs_nom_hyumm.setPIDgain(Kp_n, Kd_n, K_n);

	// Task
	// for IDC
	// Task_Kp << 8000, 8000, 8000, 1000, 1000, 1000;
	// Task_Kv << 1000, 1000, 1000, 500, 500, 500;
	// Task_K << 1,1,0.5,0.3,0.2,0.2;
	// for pIDC
	// Task_Kp << 200000, 200000, 200000, 200000, 200000, 200000;
	// Task_Kv << 1000, 1000, 1000, 1000, 1000, 1000;
	Task_K << 1.5,1.5,1.5,1,1,0.5,0.3,0.2,0.2;
	// for SPD
	Task_Kp << 1, 1, 1, 0.033, 0.033, 0.033;
	// Task_Kp << 1, 1, 1, 1, 1, 1;
	Task_Kp = Task_Kp * 1e6;
	Task_Kv = Task_Kp*period*30.0;
	cs_nom_hyumm.setTaskgain(Task_Kp, Task_Kv, Task_K);
	
	// for Impedance model
	A_.diagonal() << 4, 4, 4, 4, 4, 4;
	// A_.diagonal() << 5, 5, 5, 5, 5, 5;
    K_.diagonal() << 10, 10, 10, 30000, 30000, 30000;
	for(int i=0;i<6;i++)
		D_(i,i) = 2*1.0*sqrt(A_(i,i)*K_(i,i));
	// D_(0,0) = 2; D_(1,1) = 2; D_(2,2) = 2; 
	cs_nom_hyumm.setTaskImpedancegain(A_,D_,K_);


    for(int j=0; j<MOBILE_DRIVE_NUM; ++j)
	{
		ecat_master.addSlaveiServo(0, j+OFFSET_NUM, &ecat_iservo[j]);
		ecat_iservo[j].mode_of_operation_ = ecat_iservo[j].MODE_CYCLIC_SYNC_VELOCITY;
	}
    for(int j=0; j<NRMK_DRIVE_NUM; ++j)
	{
		ecat_master.addSlaveNRMKdrive(0, j+OFFSET_NUM+MOBILE_DRIVE_NUM, &ecat_drive[j]);
		ecat_drive[j].mode_of_operation_ = ecat_drive[j].MODE_CYCLIC_SYNC_TORQUE;
	}
    for(int j=0; j<NRMK_TOOL_NUM; ++j)
	{
		ecat_master.addSlaveNRMKtool(0, j+OFFSET_NUM+MOBILE_DRIVE_NUM+NRMK_DRIVE_NUM, &ecat_tool[j]);
	}

    initAxes();

    // Print Motor parameters
    for(int i=0; i<MOBILE_DRIVE_NUM; i++)
    {
        rt_printf("rate current: %lf\n", Axis_Motor[i].getTauRateCur());
        rt_printf("torque constant: %lf\n", Axis_Motor[i].getTauK());
        rt_printf("encoder resol: %d\n", Axis_Motor[i].getPulsePerRevolution());
        rt_printf("motor direction: %d\n", Axis_Motor[i].getDirQ());
    }

    ecat_master.activateWithDC(0, cycle_ns);
    
    for (int i=0; i<MOBILE_DRIVE_NUM; i++)
        ecat_iservo[i].setServoOn();
    for (int i=0; i<NRMK_DRIVE_NUM; i++)
        ecat_drive[i].setServoOn();
    
    rt_task_set_periodic(NULL, TM_NOW, cycle_ns);
    while (1) {
        beginCycle = rt_timer_read();
        // Read Joints Data
        readData();
        if(system_ready)
        {
            // Trajectory Generation
            trajectory_generation();
            
            // Compute KDL
            compute();	

            
            // Controller
            control();
                    
        }
		else
		{
			// Vector3d _com = {0,0,-0.124};
			// double _m = 2.5;
			// MM_JVec _addtor;
			// _addtor = cs_hyumm.addLoadedTorque(info_mm.act.q, _com, _m);
			// info_mm.des.tau = cs_hyumm.computeG(info_mm.act.q) + _addtor;
			info_mm.des.tau = cs_hyumm.computeG(info_mm.act.q);
		}
        // Write Joint Data
        writeData();
        
        endCycle = rt_timer_read();
		periodCycle = (unsigned long) endCycle - beginCycle;
		periodLoop = (unsigned long) beginCycle - beginCyclebuf;

        if(isSlaveInit())
		{
			if(ft_init_cnt==0)
			{
				// Stop FT Sensor
				UINT32 FTConfigParam=FT_STOP_DEVICE;
				ecat_tool[0].writeFTconfig(FTConfigParam);			
        		ecat_master.RxUpdate();
				ft_init_cnt++;
			}
			else if(ft_init_cnt==1)
			{
				// Start
				UINT32 FTConfigParam=FT_START_DEVICE;
				ecat_tool[0].writeFTconfig(FTConfigParam);			
        		ecat_master.RxUpdate();
				ft_init_cnt++;
			}
			else if(ft_init_cnt==2)
			{
				// Set bias
				UINT32 FTConfigParam=FT_SET_BIAS;
				ecat_tool[0].writeFTconfig(FTConfigParam);			
        		ecat_master.RxUpdate();
				ft_init_cnt++;
			}
			else if(ft_init_cnt==3)
			{
				// Set Filter 10Hz
				UINT32 FTConfigParam=FT_SET_FILTER_500;
				ecat_tool[0].writeFTconfig(FTConfigParam);			
        		ecat_master.RxUpdate();
				ft_init_cnt++;
			}
			else
				system_ready=true;;	//all drives have been done
		} 
            
        
		if(system_ready)
		{
			gt+= period;
			if (periodCycle > cycle_ns) overruns++;
			if (periodLoop > worstLoop) worstLoop = periodLoop;
		}

        beginCyclebuf = beginCycle;
		rt_task_wait_period(NULL); //wait for next cycle
    }
}

void runQtApplication(int argc, char* argv[]) {
  QApplication a(argc, argv);
  // style our application with custom dark style
  QApplication::setStyle(new DarkStyle);

  // create frameless window (and set windowState or title)
  FramelessWindow framelessWindow;

  // create our mainwindow instance
  MainWindow *mainWindow = new MainWindow;
  // add the mainwindow to our custom frameless window
  framelessWindow.resize(1600,600);
  framelessWindow.setContent(mainWindow);
  framelessWindow.show();
  a.exec();
}

// Safety task
void safety_run(void *arg)
{
	RTIME now, previous=0;
	int i;
	unsigned long itime=0, step;
	long stick=0;
	int count=0;
	unsigned int NumSlaves=0, masterState=0, slaveState[NRMK_DRIVE_NUM]={0,};

	rt_task_set_periodic(NULL, TM_NOW, 10*cycle_ns);
	
	while (1)
	{
		rt_task_wait_period(NULL); //wait for next cycle
		
		if (system_ready)
		{
			for(int i=0;i<NRMK_DRIVE_NUM;i++)
			{
				if(Axis_Core[i].isLimitReached())
				{
					ecat_drive[i].setServoOff();
					rt_printf("Servo Off!!\n");
					// break;
				}
			}
			if(info_mm.nom.q.array().isNaN().any())
			{
				rt_printf("nan occured\n");
				for(int i=0; i<NRMK_DRIVE_NUM; i++)
					ecat_drive[i].setServoOff();
				for(int i=0; i<MOBILE_DRIVE_NUM; i++)
					ecat_iservo[i].setServoOff();
			}

		}
	}
}

void print_run(void *arg)
{
	RTIME now, previous=0;
	int i;
	unsigned long itime=0, step;
	long stick=0;
	int count=0;
		
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 100ms = 0.1s)
	 */
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns*1);
	/*
	// Joint data log
	string filename1 = "joint_log.csv";
	ifstream checkFile1(filename1);

	if (checkFile1.is_open())
	{
		checkFile1.close();
		remove(filename1.c_str());
	}
	ofstream newFile1(filename1);
	if(newFile1.is_open())
	{
		newFile1<<"Time, q_r1, q_r2, q_r3, q_r4, q_r5, q_r6, q_r7, q_r8, q_r9, dq_r1, dq_r2, dq_r3, dq_r4, dq_r5, dq_r6, dq_r7, dq_r8, dq_r9, q_n1, q_n2, q_n3, q_n4, q_n5, q_n6, q_n7, q_n8, q_n9, dq_n1, dq_n2, dq_n3, dq_n4, dq_n5, dq_n6, dq_n7, dq_n8, dq_n9\n";
		newFile1.close();
	}
	ofstream csvFile1(filename1, ios_base::app);

	// Task data log
	string filename2 = "task_log.csv";
	ifstream checkFile2(filename2);

	if (checkFile2.is_open())
	{
		checkFile2.close();
		remove(filename2.c_str());
	}

	ofstream newFile2(filename2);
	if(newFile2.is_open())
	{
		newFile2<<"Time, xr, yr, zr, q0r, q1r, q2r, q3r, xn, yn, zn, q0n, q1n, q2n, q3n\n";
		newFile2.close();
	}
	ofstream csvFile2(filename2, ios_base::app);
	*/
	while (1)
	{
		rt_task_wait_period(NULL); //wait for next cycle
		if (++count==10)
		{
			++stick;
			count=0;
		}
		
		if (system_ready)
		{
			now = rt_timer_read();
			step=(unsigned long)(now - previous) / 1000000;
			itime+=step;
			previous=now;

			rt_printf("Time=%0.3lfs, cycle_dt=%lius, worst_cycle=%lius, overrun=%d\n", gt, periodCycle/1000, worstLoop/1000, overruns);
			// /*
            rt_printf("Mobile Data\n");
			rt_printf("Des_x: %lf, Des_y: %lf, Des_th: %lf\n", info_mm.des.q(0), info_mm.des.q(1), info_mm.des.q(2));
			rt_printf("Act_x: %lf, Act_y: %lf, Act_th: %lf\n", info_mm.act.q(0), info_mm.act.q(1), info_mm.act.q(2));
			rt_printf("Nom_x: %lf, Nom_y: %lf, Nom_th: %lf\n", info_mm.nom.q(0), info_mm.nom.q(1), info_mm.nom.q(2));
			rt_printf("Des_Vx: %lf, Des_Vy: %lf, Des_Wz: %lf\n", info_mm.des.q_dot(0), info_mm.des.q_dot(1), info_mm.des.q_dot(2));
			rt_printf("Act_Vx: %lf, Act_Vy: %lf, Act_Wz: %lf\n", info_mm.act.q_dot(0), info_mm.act.q_dot(1), info_mm.act.q_dot(2));
			rt_printf("Nom_Vx: %lf, Nom_Vy: %lf, Nom_Wz: %lf\n", info_mm.nom.q_dot(0), info_mm.nom.q_dot(1), info_mm.nom.q_dot(2));
			
			// for(int j=0; j<MOBILE_DRIVE_NUM; ++j){
			// 	rt_printf("ID: %d", j);
			// 	rt_printf("\t DesPos: %lf, DesVel :%lf, DesAcc :%lf\n",info_mob.des.q[j],info_mob.des.q_dot[j],info_mob.des.q_ddot[j]);
			// 	rt_printf("\t ActPos: %lf, ActVel: %lf \n",info_mob.act.q(j), info_mob.act.q_dot(j));
			// 	rt_printf("\t NomPos: %lf, NomVel: %lf, NomAcc :%lf\n",info_mm.nom.q(j), info_mm.nom.q_dot(j), info_mm.nom.q_ddot(j));
			// 	rt_printf("\t TarTor: %lf, ActTor: %lf, ExtTor: %lf \n", info_mob.des.tau(j), info_mob.act.tau(j), info_mob.act.tau_ext(j));
			// }
            rt_printf("Arm Data\n");
			for(int j=MOBILE_DOF_NUM; j<MM_DOF_NUM; ++j){
				rt_printf("ID: %d", j+2);
				rt_printf("\t DesPos: %lf, DesVel :%lf, DesAcc :%lf\n",info_mm.des.q[j],info_mm.des.q_dot[j],info_mm.des.q_ddot[j]);
				rt_printf("\t ActPos: %lf, ActVel: %lf \n",info_mm.act.q(j), info_mm.act.q_dot(j));
				rt_printf("\t NomPos: %lf, NomVel: %lf, NomAcc :%lf\n",info_mm.nom.q(j), info_mm.nom.q_dot(j), info_mm.nom.q_ddot(j));
				rt_printf("\t TarTor: %lf, ActTor: %lf, NomTor: %lf, ExtTor: %lf \n", info_mm.des.tau(j), info_mm.act.tau(j), info_mm.nom.tau(j), info_mm.act.tau_ext(j));
			}
			// rt_printf("V: %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", info_mm.act.x_dot(0),info_mm.act.x_dot(1),info_mm.act.x_dot(2),info_mm.act.x_dot(3),info_mm.act.x_dot(4),info_mm.act.x_dot(5),info_mm.act.x_dot(6),info_mm.act.x_dot(7),info_mm.act.x_dot(8));
			// rt_printf("dV: %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", info_mm.act.x_ddot(0),info_mm.act.x_ddot(1),info_mm.act.x_ddot(2),info_mm.act.x_ddot(3),info_mm.act.x_ddot(4),info_mm.act.x_ddot(5),info_mm.act.x_ddot(6),info_mm.act.x_ddot(7),info_mm.act.x_ddot(8));
			rt_printf("readFT: %lf, %lf, %lf, %lf, %lf, %lf\n", F_tmp(0),F_tmp(1),F_tmp(2),F_tmp(3),F_tmp(4),F_tmp(5));
			rt_printf("resFT: %lf, %lf, %lf, %lf, %lf, %lf\n", info_mm.act.F(0),info_mm.act.F(1),info_mm.act.F(2),info_mm.act.F(3),info_mm.act.F(4),info_mm.act.F(5));
			rt_printf("tau_ext: %lf, %lf, %lf\n", info_mm.act.tau_ext(0), info_mm.act.tau_ext(1), info_mm.act.tau_ext(2));
			rt_printf("manipulability: %lf\n",manipulability);
			rt_printf("Tdes: \t%lf, %lf, %lf, %lf\n", info_mm.des.T(0,0), info_mm.des.T(0,1), info_mm.des.T(0,2), info_mm.des.T(0,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info_mm.des.T(1,0), info_mm.des.T(1,1), info_mm.des.T(1,2), info_mm.des.T(1,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info_mm.des.T(2,0), info_mm.des.T(2,1), info_mm.des.T(2,2), info_mm.des.T(2,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info_mm.des.T(3,0), info_mm.des.T(3,1), info_mm.des.T(3,2), info_mm.des.T(3,3));
			rt_printf("T: \t%lf, %lf, %lf, %lf\n", info_mm.nom.T(0,0), info_mm.nom.T(0,1), info_mm.nom.T(0,2), info_mm.nom.T(0,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info_mm.nom.T(1,0), info_mm.nom.T(1,1), info_mm.nom.T(1,2), info_mm.nom.T(1,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info_mm.nom.T(2,0), info_mm.nom.T(2,1), info_mm.nom.T(2,2), info_mm.nom.T(2,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info_mm.nom.T(3,0), info_mm.nom.T(3,1), info_mm.nom.T(3,2), info_mm.nom.T(3,3));
			rt_printf("T: \t%lf, %lf, %lf, %lf\n", info_mm.act.T(0,0), info_mm.act.T(0,1), info_mm.act.T(0,2), info_mm.act.T(0,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info_mm.act.T(1,0), info_mm.act.T(1,1), info_mm.act.T(1,2), info_mm.act.T(1,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info_mm.act.T(2,0), info_mm.act.T(2,1), info_mm.act.T(2,2), info_mm.act.T(2,3));
			rt_printf("\t %lf, %lf, %lf, %lf\n", info_mm.act.T(3,0), info_mm.act.T(3,1), info_mm.act.T(3,2), info_mm.act.T(3,3));
			rt_printf("\n");
			rt_printf("\n");
			// */
			/*
			if(csvFile1.is_open())
			{
				csvFile1<<gt<<", ";
				for (int i = 0; i < MM_DOF_NUM; ++i) csvFile1<<info_mm.act.q(i) << ", ";
				for (int i = 0; i < MM_DOF_NUM; ++i) csvFile1<<info_mm.act.q_dot(i) << ", ";
				for (int i = 0; i < MM_DOF_NUM; ++i) csvFile1<<info_mm.nom.q(i) << ", ";
				for (int i = 0; i < MM_DOF_NUM; ++i) csvFile1<<info_mm.nom.q_dot(i) << ", ";
				csvFile1<<"\n";
			}
			if(csvFile2.is_open())
			{
				Quaterniond quat_r(info_mm.act.R);
				Quaterniond quat_n(info_mm.nom.R);
				
				csvFile2<<gt<<", ";
				for (int i = 0; i < 3; ++i) csvFile2<<info_mm.act.T(i,3) << ", "; // xr,yr,zr
				for (int i = 0; i < 4; ++i) csvFile2<<quat_r.vec()(i) << ", ";
				for (int i = 0; i < 3; ++i) csvFile2<<info_mm.nom.T(i,3) << ", ";
				for (int i = 0; i < 4; ++i) csvFile2<<quat_n.vec()(i) << ", ";
				csvFile2<<"\n";
			}
			*/
		}
		else
		{
			if (count==0){
				rt_printf("%i", stick);
				for(i=0; i<stick; ++i)
					rt_printf(".");
				rt_printf("\n");
			}
		}
	}
	/*
	csvFile1.close();
	csvFile2.close();
	*/
}

// Bullet task
void bullet_run(void *arg)
{
	struct sockaddr_ipc addr_nom, addr_act;
	uint addr_nom_len = sizeof(addr_nom);
	uint addr_act_len = sizeof(addr_act);
	int socket_nom, socket_act;
	int ret_nom, ret_act;
	struct timespec ts;
	size_t poolsz;
	size_t BUFLEN = sizeof(packet::JointState);

	struct packet::JointState *bullet_nom_msg = (packet::JointState *)malloc(BUFLEN);
	struct packet::JointState *bullet_act_msg = (packet::JointState *)malloc(BUFLEN);
	
    rt_task_set_periodic(NULL, TM_NOW, 100*cycle_ns); // 100ms

	socket_nom = __cobalt_socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP);
	if (socket_nom < 0) {
		perror("socket_nom");
		exit(EXIT_FAILURE);
	}
	socket_act = __cobalt_socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP);
	if (socket_act < 0) {
		perror("socket_act");
		exit(EXIT_FAILURE);
	}

	poolsz = 16384; /* bytes */
	if( __cobalt_setsockopt(socket_nom, SOL_XDDP, XDDP_POOLSZ, &poolsz, sizeof(poolsz))==-1)
		fail("setsockopt");

	memset(&addr_nom, 0, sizeof(addr_nom));
	addr_nom.sipc_family = AF_RTIPC;
	addr_nom.sipc_port = XDDP_PORT_SIM;

	if( __cobalt_setsockopt(socket_act, SOL_XDDP, XDDP_POOLSZ, &poolsz, sizeof(poolsz))==-1)
		fail("setsockopt");

	memset(&addr_act, 0, sizeof(addr_act));
	addr_act.sipc_family = AF_RTIPC;
	addr_act.sipc_port = XDDP_PORT_ACT;

	if(__cobalt_bind(socket_nom, (struct sockaddr *)&addr_nom, sizeof(addr_nom)) == -1)
		fail("bind");
	if(__cobalt_bind(socket_act, (struct sockaddr *)&addr_act, sizeof(addr_act)) == -1)
		fail("bind");

    while(1) 
    {
        rt_task_wait_period(NULL); //wait for next cycle
		if(system_ready)
		{
			Quaterniond quaternion(cs_nom_hyumm.getRMat());		
			for(int i=0; i<MM_DOF_NUM; i++)
			{
				bullet_nom_msg->position[i] = info_mm.nom.q(i);
				bullet_act_msg->position[i] = info_mm.act.q(i);
			}
			ret_nom = __cobalt_sendto(socket_nom, bullet_nom_msg, BUFLEN, 0, (struct sockaddr *) &addr_nom, addr_nom_len);
			ret_act = __cobalt_sendto(socket_act, bullet_act_msg, BUFLEN, 0, (struct sockaddr *) &addr_act, addr_act_len);
		}
	}
	close(socket_nom);
	close(socket_act);

	return;
}

void odom_run(void *arg) {
    struct sockaddr_ipc addr;
	uint addrlen = sizeof(addr);
	int socket;
	int ret, bufsize = 0;
	struct timespec ts;
	size_t poolsz;
	char buf[128];
	size_t BUFLEN = sizeof(packet::Odometry);
	Vector3d V_mob;
	
	struct packet::Odometry *odom_msg = (packet::Odometry *)malloc(BUFLEN);
	
    rt_task_set_periodic(NULL, TM_NOW, 1*cycle_ns); // 100ms

	socket = __cobalt_socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP);
	if (socket < 0) {
		perror("socket");
		exit(EXIT_FAILURE);
	}

	poolsz = 16384; /* bytes */
	if( __cobalt_setsockopt(socket, SOL_XDDP, XDDP_POOLSZ, &poolsz, sizeof(poolsz))==-1)
		fail("setsockopt");

	memset(&addr, 0, sizeof(addr));
	addr.sipc_family = AF_RTIPC;
	addr.sipc_port = XDDP_PORT_ODOM;

	if(__cobalt_bind(socket, (struct sockaddr *)&addr, sizeof(addr)) == -1)
		fail("bind");

    while(1) 
    {
        rt_task_wait_period(NULL); //wait for next cycle
		if(system_ready)
		{
			Quaterniond quaternion;
			quaternion = AngleAxisd(info_mm.nom.q(2), Eigen::Vector3d::UnitZ());
			V_mob = E_mob*info_mm.nom.q_dot.head(3);

			odom_msg->pose.position.x = info_mm.nom.q(0);
			odom_msg->pose.position.y = info_mm.nom.q(1);
			odom_msg->pose.orientation.x = quaternion.x();
			odom_msg->pose.orientation.y = quaternion.y();
			odom_msg->pose.orientation.z = quaternion.z();
			odom_msg->pose.orientation.w = quaternion.w();
			odom_msg->twist.linear.x = V_mob(0);
			odom_msg->twist.linear.y = V_mob(1);
			odom_msg->twist.angular.z = V_mob(2);
			
			ret = __cobalt_sendto(socket, odom_msg, BUFLEN, 0, (struct sockaddr *) &addr, addrlen);
			// ret = __cobalt_sendto(socket, odom_msg, BUFLEN, 0, NULL, 0);
			// if (ret != BUFLEN)
			// {
			// 	fail("sendto");		
			// }
		}
	}
	close(socket);

	return;
}

void cmd_vel_run(void *arg) {
    struct sockaddr_ipc addr;
	uint addrlen = sizeof(addr);
	int socket;
	int ret, n = 0, len;
	struct timespec ts;
	size_t poolsz;
	char buf[128];
	size_t BUFLEN = sizeof(packet::Twist);
	Vector3d V_mob;
	
	struct packet::Twist *twist_msg = (packet::Twist *)malloc(BUFLEN);

    rt_task_set_periodic(NULL, TM_NOW, 1*cycle_ns); // 100ms

	socket = __cobalt_socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP);
	if (socket < 0) {
		perror("socket");
		exit(EXIT_FAILURE);
	}

	poolsz = 16384; /* bytes */
	if(__cobalt_setsockopt(socket, SOL_XDDP, XDDP_POOLSZ, &poolsz, sizeof(poolsz))==-1)
		fail("setsockopt");

	memset(&addr, 0, sizeof(addr));
	addr.sipc_family = AF_RTIPC;
	addr.sipc_port = XDDP_PORT_CMD_VEL;

	if( __cobalt_bind(socket, (struct sockaddr *)&addr, sizeof(addr)) == -1)
		fail("bind");

    while(1) 
    {
        rt_task_wait_period(NULL); //wait for next cycle


		ret = __cobalt_recvfrom(socket, twist_msg, BUFLEN, 0, NULL, 0);

		if(ret >0)
		{
			
			V_mob << twist_msg->linear.x , twist_msg->linear.y, twist_msg->angular.z;
			info_mm.des.q_dot.head(3) = Einv_mob * V_mob;
		}
		
	}

	close(socket);

	return;
}

void signal_handler(int signum)
{
    rt_task_delete(&motor_task);
	rt_task_delete(&bullet_task);
    rt_task_delete(&print_task);
    rt_task_delete(&odom_writer);
	rt_task_delete(&cmd_vel_listener);
    
    for(int i=0; i<MOBILE_DRIVE_NUM; i++)
        ecat_iservo[i].setServoOff();
        
    ecat_master.deactivate();

    printf("\n\n");
	if(signum==SIGINT)
		printf("╔════════════════[SIGNAL INPUT SIGINT]═══════════════╗\n");
	else if(signum==SIGTERM)
		printf("╔═══════════════[SIGNAL INPUT SIGTERM]═══════════════╗\n");	
	else if(signum==SIGTSTP)
		printf("╔═══════════════[SIGNAL INPUT SIGTSTP]══════════════╗\n");
    printf("║                Servo drives Stopped!               ║\n");
	printf("╚════════════════════════════════════════════════════╝\n");	

    exit(1);
}

int main(int argc, char *argv[])
{
    // Perform auto-init of rt_print buffers if the task doesn't do so
    rt_print_init(0, NULL);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
	signal(SIGTSTP, signal_handler);

    /* Avoids memory swapping for this program */
    mlockall(MCL_CURRENT|MCL_FUTURE);

    cpu_set_t cpuset_qt, cpuset_rt1, cpuset_rt2;
    CPU_ZERO(&cpuset_qt);
    CPU_ZERO(&cpuset_rt1);  
    CPU_ZERO(&cpuset_rt2);  

    CPU_SET(6, &cpuset_qt);  
    CPU_SET(7, &cpuset_rt1);  
    CPU_SET(5, &cpuset_rt2);  

	cs_hyumm=CS_hyumm();
	cs_hyumm.CSSetup("../lib/URDF2CASADI/hyumm/hyumm.json", period);
	cs_nom_hyumm=CS_hyumm();
	cs_nom_hyumm.CSSetup("../lib/URDF2CASADI/hyumm/hyumm.json", period);
	cs_indy7=CS_Indy7();
	cs_indy7.CSSetup("../lib/URDF2CASADI/indy7/indy7.json", period);

    
    // std::thread qtThread(runQtApplication, argc, argv);
    // pthread_t pthread = qtThread.native_handle();
    // int rc = pthread_setaffinity_np(pthread, sizeof(cpu_set_t), &cpuset_qt);

    rt_task_create(&safety_task, "safety_task", 0, 93, 0);
    rt_task_set_affinity(&safety_task, &cpuset_rt1);
	rt_task_start(&safety_task, &safety_run, NULL);

	rt_task_create(&odom_writer, "odom_writer", 0, 80, 0);
    // rt_task_set_affinity(&odom_writer, &cpuset_rt1);
	rt_task_start(&odom_writer, &odom_run, NULL);

	rt_task_create(&cmd_vel_listener, "cmd_vel_listener", 0, 90, 0);
    // rt_task_set_affinity(&cmd_vel_listener, &cpuset_rt1);
	rt_task_start(&cmd_vel_listener, &cmd_vel_run, NULL);

    rt_task_create(&motor_task, "motor_task", 0, 99, 0);
    rt_task_set_affinity(&motor_task, &cpuset_rt2);
    rt_task_start(&motor_task, &motor_run, NULL);

	rt_task_create(&bullet_task, "bullet_task", 0, 60, 0);
    // rt_task_set_affinity(&bullet_task, &cpuset_rt1);
    rt_task_start(&bullet_task, &bullet_run, NULL);

    rt_task_create(&print_task, "print_task", 0, 70, 0);
    rt_task_set_affinity(&print_task, &cpuset_rt1);
    rt_task_start(&print_task, &print_run, NULL);

    // Must pause here
    pause();
    // qtThread.join();

    // Finalize
    signal_handler(0);

    return 0;
}

