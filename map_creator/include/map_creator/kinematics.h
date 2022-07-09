#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>


namespace kinematics
{
class Kinematics
{
public:

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
		 double[3] eetrans,
		 double[9] eerot);
  
};
}

#endif  // KINEMATICS
