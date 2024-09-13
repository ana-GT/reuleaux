#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <urdf/model.h>
#include <srdfdom/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <trac_ik/trac_ik.hpp>

namespace kinematics
{
class Kinematics
{
public:

  Kinematics();
  bool init(const std::string &_group);
  
  double SIGN(double x);
  double NORM(double a, double b, double c, double d);
  void getPoseFromFK(const std::vector< double > joint_values,
		     std::vector< double >& pose);

  bool isIKSuccess(const std::vector<double> &pose,
		   std::vector<double> &joints,
		   int& numOfSolns);

  const std::string getRobotName();

  bool isIkSuccesswithTransformedBase(const geometry_msgs::Pose& base_pose,
				      const geometry_msgs::Pose& grasp_pose,
				      std::vector<double>& joint_soln,
                                      int& numOfSolns);


  unsigned int getNumJoints();

  void computeFk(std::vector<double> joints,
		 double eetrans[3],
		 double eerot[9]);
  
protected:
  bool getChainInfo(const std::string &_group);
  void arrayToVector(const KDL::JntArray &_q, std::vector<double> &_js);
  void fillZeros(KDL::JntArray &_q);


  ros::NodeHandle nh_;

  std::string group_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::shared_ptr<TRAC_IK::TRAC_IK> ik_solver_;
  KDL::Chain chain_;
  std::string chain_root_link_;
  std::string chain_tip_link_;
  unsigned int chain_num_joints_;

  double ik_max_time_;
  double ik_epsilon_;
  TRAC_IK::SolveType ik_type_;

  
  std::shared_ptr<urdf::Model> urdf_;
  std::shared_ptr<srdf::Model> srdf_model_;
  KDL::Tree tree_;
  
}; // class Kinematics

} // namespace


