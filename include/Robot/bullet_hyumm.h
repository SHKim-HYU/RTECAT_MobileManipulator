#ifndef BULLET_HYUMM_SETUP_H
#define BULLET_HYUMM_SETUP_H

#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "Bullet3Common/b3HashMap.h"
#include "RobotSimulator/b3RobotSimulatorClientAPI.h"
#include "liegroup_robotics.h"
#include "PropertyDefinition.h"
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>      // std::setprecision

using namespace std;
using namespace Eigen;
using namespace lr;
class Bullet_Hyumm
{
	int robotId;
	int actuated_joint_num;
	int eef_num;
	vector<int> actuated_joint_id;
	vector<string> actuated_joint_name;
		
public:

	Bullet_Hyumm(class b3RobotSimulatorClientAPI* sim, int robotId);
	void set_torque(MM_JVec  torques ,MM_JVec  max_torques );
	MM_JVec get_q();	
	MM_JVec get_qdot();	
	SE3 get_eef_pose();	
	void reset_q(MM_JVec q);
	Vector6d get_FT();
	void apply_ext_FT(MM_JVec FT);
	int get_actuated_joint_num(){
		return this->actuated_joint_num;
	};
	
	virtual ~Bullet_Hyumm();
private:
	class b3RobotSimulatorClientAPI* sim;
};
#endif  //BULLET_HYUMM_SETUP_H