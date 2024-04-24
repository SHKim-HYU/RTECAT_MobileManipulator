#include "RTECAT_MobileManipulator_Client.h"

MOB_ROBOT_INFO info_mob;
ARM_ROBOT_INFO info_arm;
MM_ROBOT_INFO info_mm;

CS_Indy7 cs_indy7;
CS_Indy7 cs_nom_indy7;
CS_Indy7 cs_sim_indy7;

CS_hyumm cs_hyumm;

RT_TASK safety_task;
RT_TASK motor_task;
RT_TASK print_task;
RT_TASK xddp_writer;

using namespace std;


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
		Axis_Motor[i].setPulsePerRevolution(ecat_master.SDO_ENCODER_RESOLUTION(i+offset_junction));
		Axis_Motor[i].setTauRateCur(((double)ecat_master.SDO_RATE_CURRENT(i+offset_junction))/1000.0);
		Axis_Motor[i].setTauK(((double)ecat_master.SDO_TORQUE_CONSTANT(i+offset_junction))/1000000.0);
		Axis_Motor[i].setZeroPos(zeroPos_mob[i]);

		Axis_Motor[i].setDirQ(dirQ_mob[i]);
		Axis_Motor[i].setDirTau(dirTau_mob[i]);

		Axis_Motor[i].setConversionConstants();

		Axis_Motor[i].setTrajPeriod(period);
		
		Axis_Motor[i].setTarVelInRPM(0);
		Axis_Motor[i].setTarTorInCnt(0);

        info_mob.des.e = Mob_JVec::Zero();
        info_mob.des.eint = Mob_JVec::Zero();
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

		info_arm.act.q(i) = Axis_Core[i].getCurrPosInRad();
		info_arm.act.q_dot(i) = Axis_Core[i].getCurrVelInRad();
		info_arm.act.tau(i) = Axis_Core[i].getCurrTorInNm();

		Axis_MM[MOBILE_DOF_NUM+i].setCurrentPosInConf(info_arm.act.q(i));
		Axis_MM[MOBILE_DOF_NUM+i].setCurrentVelInConf(info_arm.act.q_dot(i));
		Axis_MM[MOBILE_DOF_NUM+i].setCurrentTime(gt);
		
		if(!system_ready)
		{
			Axis_Core[i].setTarPosInRad(info_arm.act.q(i));
			Axis_Core[i].setDesPosInRad(info_arm.act.q(i));
			info_arm.nom.q = info_arm.act.q;
			info_arm.nom.q_dot = info_arm.act.q_dot;
			info_arm.nom.tau = info_arm.act.tau;
		}

	}

	// Mapping status to Mobile Manipulator
	info_mob.act.x_dot = J_mob * info_mob.act.q_dot;
	info_mob.act.x += info_mob.act.x_dot*period;

	info_mm.act.q_dot.segment<MOBILE_DOF_NUM>(0) = info_mob.act.x_dot;
	info_mm.act.q.segment<MOBILE_DOF_NUM>(0) = info_mob.act.x;
	info_mm.act.q_dot.segment<NRMK_DRIVE_NUM>(MOBILE_DOF_NUM) = info_arm.act.q_dot;
	info_mm.act.q.segment<NRMK_DRIVE_NUM>(MOBILE_DOF_NUM) = info_arm.act.q;

	for(int i=0; i<MOBILE_DOF_NUM; i++)
	{
		Axis_MM[i].setCurrentPosInConf(info_mm.act.q(i));
		Axis_MM[i].setCurrentVelInConf(info_mm.act.q_dot(i));

		Axis_MM[i].setCurrentTime(gt);
	}

	for(int i=0; i<NRMK_TOOL_NUM; i++)
	{
		// Update RFT data
		if (ecat_tool[i].FT_Raw_Fx_==0 && ecat_tool[i].FT_Raw_Fy_==0 && ecat_tool[i].FT_Raw_Fz_==0)
		{
			info_arm.act.F(0) = 0.0;
			info_arm.act.F(1) = 0.0;
			info_arm.act.F(2) = 0.0;
			info_arm.act.F(3) = 0.0;
			info_arm.act.F(4) = 0.0;
			info_arm.act.F(5) = 0.0;
		}
		else{
			info_arm.act.F(0) = (double)ecat_tool[i].FT_Raw_Fx_ / force_divider - ft_offset[0];
			info_arm.act.F(1) = (double)ecat_tool[i].FT_Raw_Fy_ / force_divider - ft_offset[1];
			info_arm.act.F(2) = (double)ecat_tool[i].FT_Raw_Fz_ / force_divider - ft_offset[2];
			info_arm.act.F(3) = (double)ecat_tool[i].FT_Raw_Tx_ / torque_divider - ft_offset[3];
			info_arm.act.F(4) = (double)ecat_tool[i].FT_Raw_Ty_ / torque_divider - ft_offset[4];
			info_arm.act.F(5) = (double)ecat_tool[i].FT_Raw_Tz_ / torque_divider - ft_offset[5];

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
	    	traj_time = 10;
	    	motion++;
	        break;
	    case 2:
			info_mm.q_target(0)=0.0; info_mm.q_target(1)=0.0; info_mm.q_target(2)=0.0;
			info_mm.q_target(3)=0.0; info_mm.q_target(4)=0.0; info_mm.q_target(5)=0.0;
			info_mm.q_target(6)=0.0; info_mm.q_target(7)=0.0; info_mm.q_target(8)=0.0;
	    	traj_time = 10;
	    	motion++;
	        break;
	    case 3:
			info_mm.q_target(0)=0.0; info_mm.q_target(1)=0.0; info_mm.q_target(2)=0.0;
			info_mm.q_target(3)=-1.5709; info_mm.q_target(4)=0.4071; info_mm.q_target(5)=-0.4071;
			info_mm.q_target(6)=-1.5709; info_mm.q_target(7)=-1.5709; info_mm.q_target(8)=-1.5709;
	    	traj_time = 10;
	    	motion++;
	        break;
	    case 4:
			info_mm.q_target(0)=0.0; info_mm.q_target(1)=0.0; info_mm.q_target(2)=0.0;
			info_mm.q_target(3)=0.0; info_mm.q_target(4)=0.0; info_mm.q_target(5)=0.0;
			info_mm.q_target(6)=0.0; info_mm.q_target(7)=0.0; info_mm.q_target(8)=0.0;
	    	traj_time = 10;
			motion=1;
	    	break;
		// case 5:
		// 	// info_mob.q_target(0)=-1.5709; info_mob.q_target(1)=-1.5709; info_mob.q_target(2)=-1.5709; info_mob.q_target(3)=-1.5709; //
        //     info_arm.q_target(0)=0.0; 	info_arm.q_target(1)=-1.5709; 	info_arm.q_target(2)=0.0;
	    // 	info_arm.q_target(3)=0.0; 	info_arm.q_target(4)=0.0; 	info_arm.q_target(5)=0.0;
	    // 	traj_time = 4;
	    // 	motion++;
	    // 	break;
	    // case 6:
		// 	// info_mob.q_target(0)=-1.5709; info_mob.q_target(1)=-1.5709; info_mob.q_target(2)=-1.5709; info_mob.q_target(3)=-1.5709; //
        //     info_arm.q_target(0)=-1.5709; 	info_arm.q_target(1)=-1.5709; 	info_arm.q_target(2)=0.0;
	    // 	info_arm.q_target(3)=0.0; 	info_arm.q_target(4)=0.0; 	info_arm.q_target(5)=0.0;
	    // 	traj_time = 4;
	    // 	motion=1;
	    // 	break;
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

		info_mm.des.q(i)=Axis_MM[i].getDesPosInRad();
		info_mm.des.q_dot(i)=Axis_MM[i].getDesVelInRad();
		info_mm.des.q_ddot(i)=Axis_MM[i].getDesAccInRad();
	}
	info_arm.des.q = info_mm.des.q.segment<NRMK_DRIVE_NUM>(MOBILE_DOF_NUM);
	info_arm.des.q_dot = info_mm.des.q_dot.segment<NRMK_DRIVE_NUM>(MOBILE_DOF_NUM);
	info_arm.des.q_ddot = info_mm.des.q_ddot.segment<NRMK_DRIVE_NUM>(MOBILE_DOF_NUM);
}

void compute()
{
    // Update Robot
	cs_indy7.updateRobot(info_arm.act.q , info_arm.act.q_dot);
	// Update nominal
	cs_nom_indy7.updateRobot(info_arm.nom.q , info_arm.nom.q_dot);
	// Update Mobile Manipulator
	cs_hyumm.updateRobot(info_mm.act.q, info_mm.act.q_dot);
	
	info_mm.act.CoM_x = cs_hyumm.getCoM();
	info_mm.act.J_com = cs_hyumm.getJ_com();
	
	info_arm.act.T = cs_indy7.getFK();
	info_mm.act.T = cs_hyumm.getFK();

	Arm_Jacobian J_b = cs_indy7.getJ_b();
	info_arm.act.x_dot = J_b*info_arm.act.q_dot;

	Arm_JVec F_tmp = Arm_JVec::Zero();
	for(int i=0; i<6; i++)
    {
		if(i<3)
		{
			if(1.0>abs(info_arm.act.F(i)))
			{
				F_tmp[i]=0.0;
			}
			else
			{
				F_tmp[i]=info_arm.act.F(i);
			}
		}
		else if(i==5)
		{
			if(0.05>abs(info_arm.act.F(i)))
			{
				F_tmp[i]=0.0;
			}
			else
			{
				// F_tmp[i]=info_arm.act.F(i);
				F_tmp[i]=0.0;
			}
		}
		else 
		{
			if(0.1>abs(info_arm.act.F(i)))
			{
				F_tmp[i]=0.0;
			}
			else
			{
				F_tmp[i]=info_arm.act.F(i);
			}
		}
    }
	info_arm.act.tau_ext = J_b.transpose()*F_tmp;
	
	// rt_printf("tau_ext:\n");
	// rt_printf("%lf, %lf, %lf, %lf, %lf, %lf\n\n", info.act.tau_ext(0), info.act.tau_ext(1), info.act.tau_ext(2), info.act.tau_ext(3), info.act.tau_ext(4), info.act.tau_ext(5));

	info_arm.act.tau_fric = cs_indy7.FrictionEstimation(info_arm.act.q_dot);
	// info.act.tau_fric = JVec::Zero();
}

void control()
{

    double Kp = 15;
    double Kd = 5;

    for (int i = 0; i<MOBILE_DRIVE_NUM; i++)
    {
        if (ecat_iservo[i].mode_of_operation_ == ecat_iservo[i].MODE_CYCLIC_SYNC_TORQUE)
        {
            info_mob.des.e(i) = info_mob.des.q(i)-info_mob.act.q(i);
            info_mob.des.eint(i) = info_mob.des.eint(i) + info_mob.des.e(i)*period;
            info_mob.des.tau(i) = Kp*info_mob.des.e(i)+Kd*(info_mob.des.q_dot(i)-info_mob.act.q_dot(i));
        }
        else if (ecat_iservo[i].mode_of_operation_ == ecat_iservo[i].MODE_CYCLIC_SYNC_VELOCITY)
        {
            info_mm.des.e(i) = info_mm.des.q(i)-info_mm.act.q(i);
			info_mm.des.edot(i) = info_mm.des.q_dot(i)-info_mm.act.q_dot(i);
            info_mm.des.tau(i) = info_mm.des.q_dot(i) + 0.1*info_mm.des.edot(i) + 1*info_mm.des.e(i);
        }
    }
	info_mob.des.tau = Jinv_mob * info_mm.des.tau.segment<MOBILE_DOF_NUM>(0);
	info_mob.des.q_dot = info_mob.des.tau;

	// info_arm.nom.tau = cs_nom_indy7.ComputedTorqueControl(info_arm.nom.q, info_arm.nom.q_dot, info_arm.des.q, info_arm.des.q_dot, info_arm.des.q_ddot);
    info_arm.nom.tau = cs_nom_indy7.ComputedTorqueControl(info_arm.nom.q, info_arm.nom.q_dot, info_arm.des.q, info_arm.des.q_dot, info_arm.des.q_ddot, info_arm.act.tau_ext);
    info_arm.act.tau_aux = cs_indy7.NRIC(info_arm.act.q, info_arm.act.q_dot, info_arm.nom.q, info_arm.nom.q_dot);
    info_arm.des.tau = info_arm.nom.tau - info_arm.act.tau_aux;

    cs_nom_indy7.computeRK45(info_arm.nom.q, info_arm.nom.q_dot, info_arm.nom.tau, info_arm.nom.q, info_arm.nom.q_dot);
	// info_arm.des.tau = cs_indy7.computeG(info_arm.act.q);
}

void writeData()
{
    for(int i=1;i<=MOBILE_DRIVE_NUM;i++){
        if (ecat_iservo[i-1].mode_of_operation_ == ecat_iservo[i-1].MODE_CYCLIC_SYNC_TORQUE)
        {
            Axis_Motor[i-1].setDesTorInNm(info_mob.des.tau(i-1));
                
            INT16 temp = Axis_Motor[i-1].getDesTorInPer();
            // rt_printf("temp: %d\n", temp);
            if (temp > 1000)
            {
                temp = 1000;            
            }
            else if (temp<-1000)
            {
                temp = -1000;
            }
            // rt_printf("temp: %d\n\n", temp);
            // ecat_master.RxPDO1_SEND(i, (short)temp);
            ecat_iservo[i-1].writeTorque(temp);
        }
        else if (ecat_iservo[i-1].mode_of_operation_ == ecat_iservo[i-1].MODE_CYCLIC_SYNC_VELOCITY)
        {
            // rt_printf("velocity: %d\n",Axis_Motor[i-1].getDesVelInRPM());
            ecat_iservo[i-1].writeVelocity(Axis_Motor[i-1].getDesVelInRPM(info_mob.des.tau(i-1)));
        }
		ecat_master.RxUpdate();
	}

    for(int i=1;i<=NRMK_DRIVE_NUM;i++){

        Axis_Core[i-1].setDesTorInNm(info_arm.des.tau(i-1));
            
        INT16 temp = Axis_Core[i-1].getDesTorInCnt();

        // ecat_master.RxPDO1_SEND(i, (short)temp);
        ecat_drive[i-1].writeTorque(temp);

        ecat_master.RxUpdate();
	}
    ecat_master.SyncEcatMaster(rt_timer_read());
}

void motor_run(void *arg)
{
    RTIME beginCycle, endCycle;
   
    memset(&info_mob, 0, sizeof(MOB_ROBOT_INFO));
    memset(&info_arm, 0, sizeof(ARM_ROBOT_INFO));

	int ft_init_cnt = 0;

	info_arm.des.q = Arm_JVec::Zero();
	info_arm.des.q_dot = Arm_JVec::Zero();
	info_arm.des.q_ddot = Arm_JVec::Zero();
	info_arm.des.F = Vector6d::Zero();
	info_arm.des.F_CB = Vector6d::Zero();
	info_arm.act.tau_aux = Arm_JVec::Zero();

	// Real
	NRIC_Kp << 20.0, 25.0, 10.0, 3.0, 3.0, 1.5;
	NRIC_Ki << 5.0, 5.5, 2.5, 0.8, 0.8, 0.6;
	NRIC_K_gamma << 550.0, 600.0, 450.0, 250.0, 250.0, 175.0;
	cs_indy7.setNRICgain(NRIC_Kp, NRIC_Ki, NRIC_K_gamma);
	
	// nominal
	Kp_n << 50.0, 50.0, 30.0, 15.0, 15.0, 15.0;
	Kd_n << 5.0, 5.0, 3.0, 1.5, 1.5, 1.5;
	Ki_n = Arm_JVec::Zero();

	cs_nom_indy7.setPIDgain(Kp_n, Kd_n, Ki_n);

    // ecat_master.activate_all(DEVICE2, config, MOBILE_DRIVE_NUM);

    for(int j=0; j<MOBILE_DRIVE_NUM; ++j)
	{
		ecat_master.addSlaveiServo(0, j+offset_junction, &ecat_iservo[j]);
		ecat_iservo[j].mode_of_operation_ = ecat_iservo[j].MODE_CYCLIC_SYNC_VELOCITY;
	}
    for(int j=0; j<NRMK_DRIVE_NUM; ++j)
	{
		ecat_master.addSlaveNRMKdrive(0, j+offset_junction+MOBILE_DRIVE_NUM, &ecat_drive[j]);
		ecat_drive[j].mode_of_operation_ = ecat_drive[j].MODE_CYCLIC_SYNC_TORQUE;
	}
    for(int j=0; j<NRMK_TOOL_NUM; ++j)
	{
		ecat_master.addSlaveNRMKtool(0, j+offset_junction+MOBILE_DRIVE_NUM+NRMK_DRIVE_NUM, &ecat_tool[j]);
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
			info_arm.des.tau = cs_indy7.computeG(info_arm.act.q);
		}
        // Write Joint Data
        writeData();
        
        endCycle = rt_timer_read();
		periodCycle = (unsigned long) endCycle - beginCycle;

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
				// Set Filter 100Hz
				UINT32 FTConfigParam=FT_SET_FILTER_10;
				ecat_tool[0].writeFTconfig(FTConfigParam);			
        		ecat_master.RxUpdate();
				ft_init_cnt++;
			}
			else if(ft_init_cnt==3)
			{
				// Set bias
				UINT32 FTConfigParam=FT_SET_BIAS;
				ecat_tool[0].writeFTconfig(FTConfigParam);			
        		ecat_master.RxUpdate();
				ft_init_cnt++;
			}
			else
				system_ready=true;;	//all drives have been done
		} 
            
        gt+= period;
        if (periodCycle > cycle_ns) overruns++;
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

	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);
	
	while (1)
	{
		rt_task_wait_period(NULL); //wait for next cycle
		
		if (system_ready)
		{
			for(int i=0;i<NRMK_DRIVE_NUM;i++)
			{
				if(Axis_Core[i].isLimitReached())
				{
					for(int i=0;i<NRMK_DRIVE_NUM;i++)
						ecat_drive[i].setServoOff();
					rt_printf("Servo Off!!\n");
					break;
				}
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
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns*100);
	
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

			rt_printf("Time=%0.3lfs, cycle_dt=%lius,  overrun=%d\n", gt, periodCycle/1000, overruns);
			
            rt_printf("Mobile Data\n");
			rt_printf("Des_x: %lf, Des_y: %lf, Des_th: %lf\n", info_mm.des.q(0), info_mm.des.q(1), info_mm.des.q(2));
			rt_printf("Act_x: %lf, Act_y: %lf, Act_th: %lf\n", info_mm.act.q(0), info_mm.act.q(1), info_mm.act.q(2));
			rt_printf("Des_Vx: %lf, Des_Vy: %lf, Des_Wz: %lf\n", info_mm.des.q_dot(0), info_mm.des.q_dot(1), info_mm.des.q_dot(2));
			rt_printf("Act_Vx: %lf, Act_Vy: %lf, Act_Wz: %lf\n", info_mm.act.q_dot(0), info_mm.act.q_dot(1), info_mm.act.q_dot(2));
			rt_printf("Max act: %lf, %lf, %lf, %lf\n", act_max[0], act_max[1], act_max[2], act_max[3]);
			
			for(int j=0; j<MOBILE_DRIVE_NUM; ++j){
				rt_printf("ID: %d", j);
				rt_printf("\t DesPos: %lf, DesVel :%lf, DesAcc :%lf\n",info_mob.des.q[j],info_mob.des.q_dot[j],info_mob.des.q_ddot[j]);
				rt_printf("\t ActPos: %lf, ActVel: %lf \n",info_mob.act.q(j), info_mob.act.q_dot(j));
				rt_printf("\t TarTor: %lf, ActTor: %lf, ExtTor: %lf \n", info_mob.des.tau(j), info_mob.act.tau(j), info_mob.act.tau_ext(j));
			}
            rt_printf("Arm Data\n");
			for(int j=0; j<NRMK_DRIVE_NUM; ++j){
				rt_printf("ID: %d", j+MOBILE_DRIVE_NUM);
				rt_printf("\t DesPos: %lf, DesVel :%lf, DesAcc :%lf\n",info_arm.des.q[j],info_arm.des.q_dot[j],info_arm.des.q_ddot[j]);
				rt_printf("\t ActPos: %lf, ActVel: %lf \n",info_arm.act.q(j), info_arm.act.q_dot(j));
				rt_printf("\t NomPos: %lf, NomVel: %lf \n",info_arm.nom.q(j), info_arm.nom.q_dot(j));
				rt_printf("\t TarTor: %lf, ActTor: %lf, NomTor: %lf, ExtTor: %lf \n", info_arm.des.tau(j), info_arm.act.tau(j), info_arm.nom.tau(j), info_arm.act.tau_ext(j));
			}
			rt_printf("ReadFT: %lf, %lf, %lf, %lf, %lf, %lf\n", info_arm.act.F(0),info_arm.act.F(1),info_arm.act.F(2),info_arm.act.F(3),info_arm.act.F(4),info_arm.act.F(5));
			rt_printf("CoM_x:\t%lf, %lf, %lf\n", info_mm.act.CoM_x(0), info_mm.act.CoM_x(1), info_mm.act.CoM_x(2));
			rt_printf("J_com:\t%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n",info_mm.act.J_com(0,0),info_mm.act.J_com(0,1),info_mm.act.J_com(0,2),info_mm.act.J_com(0,3),info_mm.act.J_com(0,4),info_mm.act.J_com(0,5),info_mm.act.J_com(0,6),info_mm.act.J_com(0,7),info_mm.act.J_com(0,8));
			rt_printf("\t%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n",info_mm.act.J_com(1,0),info_mm.act.J_com(1,1),info_mm.act.J_com(1,2),info_mm.act.J_com(1,3),info_mm.act.J_com(1,4),info_mm.act.J_com(1,5),info_mm.act.J_com(1,6),info_mm.act.J_com(1,7),info_mm.act.J_com(1,8));
			rt_printf("\t%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n\n",info_mm.act.J_com(2,0),info_mm.act.J_com(2,1),info_mm.act.J_com(2,2),info_mm.act.J_com(2,3),info_mm.act.J_com(2,4),info_mm.act.J_com(2,5),info_mm.act.J_com(2,6),info_mm.act.J_com(2,7),info_mm.act.J_com(2,8));

			rt_printf("\n");

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
}


void signal_handler(int signum)
{
    rt_task_delete(&motor_task);
    rt_task_delete(&print_task);
    rt_task_delete(&xddp_writer);
    
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

    cs_indy7=CS_Indy7();
	cs_indy7.CSSetup("../lib/URDF2CASADI/indy7/indy7.json", period);
	cs_nom_indy7=CS_Indy7();
	cs_nom_indy7.CSSetup("../lib/URDF2CASADI/indy7/indy7.json", period);
	cs_sim_indy7=CS_Indy7();
	cs_sim_indy7.CSSetup("../lib/URDF2CASADI/indy7/indy7.json", period);

	cs_hyumm=CS_hyumm();
	cs_hyumm.CSSetup("../lib/URDF2CASADI/hyumm/hyumm.json", period);

    // // For CST (cyclic synchronous torque) control
	// if (ecat_master.init(OP_MODE_CYCLIC_SYNC_TORQUE, cycle_ns) == -1)
	// {
	// 	printf("System Initialization Failed\n");
	//     return 0;
	// }
	// for (int i = 0; i < MOBILE_DRIVE_NUM; ++i)
	// 	ModeOfOperation[i] = OP_MODE_CYCLIC_SYNC_TORQUE;
    
    // std::thread qtThread(runQtApplication, argc, argv);
    // pthread_t pthread = qtThread.native_handle();
    // int rc = pthread_setaffinity_np(pthread, sizeof(cpu_set_t), &cpuset_qt);

    rt_task_create(&safety_task, "safety_task", 0, 93, 0);
    rt_task_set_affinity(&safety_task, &cpuset_rt1);
	rt_task_start(&safety_task, &safety_run, NULL);

    rt_task_create(&motor_task, "motor_task", 0, 99, 0);
    rt_task_set_affinity(&motor_task, &cpuset_rt2);
    rt_task_start(&motor_task, &motor_run, NULL);

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

