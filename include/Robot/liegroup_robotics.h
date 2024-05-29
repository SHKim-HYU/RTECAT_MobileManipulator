/*
 * liegroup_robotics.h
 *
 *  Created on: May 29, 2024
 *      Author: Minchang Sung, Sunhong Kim
 */

#ifndef LIEGROUP_ROBOTICS_H
#define LIEGROUP_ROBOTICS_H
    
#pragma once
#include <iostream>
#include <vector>
#include "PropertyDefinition.h"

namespace lr {

    Matrix6d ad(const Vector6d& V) ;
    Matrix6d Ad(const SE3& T);
	  Matrix6d AdInv(const SE3& T);    
    SE3 TransInv(const SE3& T) ;
    SO3 TransToR(const SE3& T);
    Vector3d TransToP(const SE3& T);
    Vector6d se3ToVec(const se3& T);
    Vector3d so3ToVec(const so3& so3mat);
    so3 VecToso3(const Vector3d& omg);
    se3 VecTose3(const Vector6d& V);
    bool NearZero(const double val) ;
    SO3 MatrixExp3(const so3& so3mat);
    SE3 MatrixExp6(const se3& se3mat);
    so3 MatrixLog3(const SO3& R);
    se3 MatrixLog6(const SE3& T);
    SE3 FKinSpace(const SE3& M, const MM_ScrewList& Slist, const MM_JVec& thetaList);
    SE3 FKinBody(const SE3& M, const MM_ScrewList& Blist, const MM_JVec& thetaList);
    MM_Jacobian dJacobianBody(const SE3& M,const MM_ScrewList& Blist, const MM_JVec& q ,const MM_JVec& dq);
    MM_Jacobian JacobianSpace(const MM_ScrewList& Slist, const MM_JVec& thetaList);
    MM_Jacobian JacobianBody(const MM_ScrewList& Blist, const MM_JVec& thetaList) ;
    SE3 RpToTrans(const Matrix3d& R, const Vector3d& p);
    bool IKinBody(const MM_ScrewList& Blist, const SE3& M, const SE3& T,
		MM_JVec& thetalist, double eomg, double ev);
    bool IKinSpace(const MM_ScrewList& Slist, const SE3& M, const SE3& T,
		MM_JVec& thetalist, double eomg, double ev) ;
    MM_JVec InverseDynamics(const MM_JVec& thetalist, const MM_JVec& dthetalist, const MM_JVec& ddthetalist,
                  const Vector3d& g, const Vector6d& Ftip, const std::vector<SE3>& Mlist,
                  const std::vector<Matrix6d>& Glist, const MM_ScrewList& Slist);
    MM_JVec InverseDynamics(const MM_JVec& thetalist, const MM_JVec& dthetalist, const MM_JVec& ddthetalist,
                  const Vector3d& g, const Vector6d& Ftip, const std::vector<SE3>& Mlist,
                  const std::vector<Matrix6d>& Glist, const MM_ScrewList& Slist,double eef_mass);                  
    MM_JVec GravityForces(const MM_JVec& thetalist, const Vector3d& g,
                  const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const MM_ScrewList& Slist) ;                                    
    MM_JVec GravityForces(const MM_JVec& thetalist, const Vector3d& g,
                  const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const MM_ScrewList& Slist,double eef_mass) ;                    
    MM_MassMat MassMatrix(const MM_JVec& thetalist, const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const MM_ScrewList& Slist) ;
    MM_MassMat MassMatrix(const MM_JVec& thetalist, const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const MM_ScrewList& Slist,double eef_mass) ;                          
                                      
    MM_JVec VelQuadraticForces(const MM_JVec& thetalist, const MM_JVec& dthetalist,const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const MM_ScrewList& Slist) ;
    MM_JVec VelQuadraticForces(const MM_JVec& thetalist, const MM_JVec& dthetalist,const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const MM_ScrewList& Slist,double eef_mass) ;
    
    MM_JVec EndEffectorForces(const MM_JVec& thetalist, const Vector6d& Ftip,const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const MM_ScrewList& Slist);
    MM_JVec EndEffectorForces(const MM_JVec& thetalist, const Vector6d& Ftip,const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const MM_ScrewList& Slist,double eef_mass);
    
    MM_JVec ForwardDynamics(const MM_JVec& thetalist, const MM_JVec& dthetalist, const MM_JVec& taulist,
                  const Vector3d& g, const Vector6d& Ftip, const std::vector<SE3>& Mlist,
                  const std::vector<Matrix6d>& Glist, const MM_ScrewList& Slist);       
               	MM_JVec ForwardDynamics(const MM_JVec& thetalist, const MM_JVec& dthetalist, const MM_JVec& taulist,
									const Vector3d& g, const Vector6d& Ftip, const std::vector<SE3>& Mlist,
									const std::vector<Matrix6d>& Glist, const MM_ScrewList& Slist,double eef_mass);                    

    void EulerStep(MM_JVec& thetalist, MM_JVec& dthetalist, const MM_JVec& ddthetalist, double dt);                                    
    Vector3d QuinticTimeScalingKinematics(double s0,double sT,double ds0,double dsT,double dds0,double ddsT,double Tf, double t) ;
    void FKinBody(const SE3& M,const MM_ScrewList& Blist, const MM_JVec& q ,const MM_JVec& dq, SE3 &T, MM_Jacobian &Jb,MM_Jacobian& dJb);
    Matrix3d dexp3(const Vector3d& xi);
    Matrix3d dlog3(const Vector3d& xi);
    Matrix6d dexp6(const Vector6d& lambda);
    Matrix3d ddexp3(const Vector3d& xi, const Vector3d& dxi);
    Matrix3d dddexp3(const Vector3d& xi, const Vector3d& dxi, const Vector3d& y, const Vector3d& dy);
    Matrix6d ddexp6(const Vector6d& lambda, const Vector6d& lambda_dot);
    Matrix3d skew_sum(const Vector3d& a, const Vector3d& b);
    Matrix3d ddlog3(const Vector3d& xi, const Vector3d& dxi);
    Matrix3d dddlog3(const Vector3d& xi, const Vector3d& dxi, const Vector3d& y, const Vector3d& dy);
    Matrix6d dlog6(const Vector6d& lambda);
    Matrix6d ddlog6(const Vector6d& lambda, const Vector6d& lambda_dot) ;    
    //void LieScrewTrajectory(const SE3 X0,const SE3 XT,const Vector6d V0,const Vector6d VT,const Vector6d dV0,const Vector6d dVT,double Tf,int N,std::vector<SE3>&Xd_list,std::vector<Vector6d>&Vd_list,std::vector<Vector6d>&dVd_list);
    void LieScrewTrajectory(const SE3 X0,const SE3 XT,const Vector6d V0,const Vector6d VT,const Vector6d dV0,const Vector6d dVT,double Tf,double t,SE3& T_des,Vector6d& V_des,Vector6d& V_des_dot);
    void JointTrajectory(const MM_JVec q0, const MM_JVec qT, double Tf, double t , int method , MM_JVec& q_des, MM_JVec& q_dot_des, MM_JVec& q_ddot_des) ;
    // SE3 RelFKinSpace(const SE3& M, const RelMM_ScrewList& Slist, const RelMM_JVec& thetaList);
    // SE3 RelFKinBody(const SE3& M, const RelMM_ScrewList& Blist, const RelMM_JVec& thetaList);
    // RelMM_Jacobian RelMM_JacobianSpace(const RelMM_ScrewList& Slist, const RelMM_JVec& thetaList) ;
    // RelMM_Jacobian RelMM_JacobianBody(const RelMM_ScrewList& Blist, const RelMM_JVec& thetaList);
    // RelMM_Jacobian ReldMM_JacobianBody(const SE3& M,const RelMM_ScrewList& Blist, const RelMM_JVec& q ,const RelMM_JVec& dq);
    // bool RelIKinSpace(const RelMM_ScrewList& Slist, const SE3& M, const SE3& T,RelMM_JVec& thetalist, double eomg, double ev);
    // bool RelIKinBody(const RelMM_ScrewList& Blist, const SE3& M, const SE3& T,RelMM_JVec& thetalist, double eomg, double ev);
}


#endif
