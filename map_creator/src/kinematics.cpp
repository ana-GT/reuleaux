/**
 * @file kinematics.cpp
 */
#include <map_creator/kinematics.h>

double eerot[9], eetrans[3];

namespace kinematics
{
double Kinematics::SIGN(double x)
{
  return (x >= 0.0f) ? +1.0f : -1.0f;
}

double Kinematics::NORM(double a, double b, double c, double d)
{
  return sqrt(a * a + b * b + c * c + d * d);
}

Kinematics::Kinematics()
{}

bool Kinematics::init(const std::string &_group)
{  
  ROS_WARN("Kinematics init");
  std::string urdf_string, srdf_string;
  if(!nh_.getParam("robot_description", urdf_string))
     return false;
  ROS_WARN_STREAM("URDF STRING: " << urdf_string << std::endl);
  if(!nh_.getParam("robot_description_semantic", urdf_string))
     return false;
  ROS_WARN("Seems we loaded urdf and srdf?");
  urdf_.reset(new urdf::Model());
  srdf_model_.reset(new srdf::Model());
  ROS_WARN("Initializing urdf model");
  
  if(!urdf_->initString(urdf_string))
  	return false;
  ROS_WARN("Initializing srdf model");

  if(!srdf_model_->initString(*urdf_, srdf_string))  	
  	return false;
  ROS_WARN("Starting tree");

  if(!kdl_parser::treeFromUrdfModel(*urdf_, tree_))
    return false;
  ROS_WARN("Get chain info");

  // Get chain for this group
  if(!getChainInfo(_group))
    return false;
    
  ik_max_time_ = 0.005;
  ik_epsilon_ = 1e-5;
  ik_type_ = TRAC_IK::Speed;  
  ik_solver_.reset( new TRAC_IK::TRAC_IK(chain_root_link_, 
                              chain_tip_link_, 
                              "robot_description", 
                              ik_max_time_, ik_epsilon_, ik_type_));

  // Create FIK solver
  fk_solver_.reset( new KDL::ChainFkSolverPos_recursive(chain_));
  return true;
}

/**
 * @function getChainInfo
 */
bool Kinematics::getChainInfo(const std::string &_group)
{
  std::vector<srdf::Model::Group> groups = srdf_model_->getGroups();
  bool found_chain = false;

  for(int i = 0; i < groups.size(); ++i)
  {
    if(groups[i].name_ == _group)
    {
        if(groups[i].chains_.size() == 1)
        {
         chain_root_link_ = groups[i].chains_[0].first;
         chain_tip_link_ = groups[i].chains_[0].second;

         tree_.getChain(chain_root_link_, chain_tip_link_, chain_);
         /*for(auto si : chain.segments)
         {
          if( isValidJointType(si.getJoint().getType()) )
            _chain_info.joint_names.push_back(si.getJoint().getName());
         }*/

         chain_num_joints_ = chain_.getNrOfJoints(); // Should be equal to joint_names.size()

         found_chain = true;
         break;
        }
    }
  }

  return found_chain;   
}

unsigned int Kinematics::getNumJoints()
{
  return chain_num_joints_;
}

void Kinematics::getPoseFromFK(const std::vector<double> joint_values,
			       std::vector<double>& pose)
{/*
  unsigned int num_of_joints = this->getNumJoints();
  //  unsigned int num_free_parameters = this->getNumFreeParameters();

  std::vector<double> joints = joint_values;


  this->computeFk(joints, eetrans, eerot);

  double q0 = (eerot[0] + eerot[4] + eerot[8] + 1.0f) / 4.0f;
  double q1 = (eerot[0] - eerot[4] - eerot[8] + 1.0f) / 4.0f;
  double q2 = (-eerot[0] + eerot[4] - eerot[8] + 1.0f) / 4.0f;
  double q3 = (-eerot[0] - eerot[4] + eerot[8] + 1.0f) / 4.0f;
  if (q0 < 0.0f)
    q0 = 0.0f;
  if (q1 < 0.0f)
    q1 = 0.0f;
  if (q2 < 0.0f)
    q2 = 0.0f;
  if (q3 < 0.0f)
    q3 = 0.0f;
  q0 = sqrt(q0);
  q1 = sqrt(q1);
  q2 = sqrt(q2);
  q3 = sqrt(q3);
  if (q0 >= q1 && q0 >= q2 && q0 >= q3)
  {
    q0 *= +1.0f;
    q1 *= SIGN(eerot[7] - eerot[5]);
    q2 *= SIGN(eerot[2] - eerot[6]);
    q3 *= SIGN(eerot[3] - eerot[1]);
  }
  else if (q1 >= q0 && q1 >= q2 && q1 >= q3)
  {
    q0 *= SIGN(eerot[7] - eerot[5]);
    q1 *= +1.0f;
    q2 *= SIGN(eerot[3] + eerot[1]);
    q3 *= SIGN(eerot[2] + eerot[6]);
  }
  else if (q2 >= q0 && q2 >= q1 && q2 >= q3)
  {
    q0 *= SIGN(eerot[2] - eerot[6]);
    q1 *= SIGN(eerot[3] + eerot[1]);
    q2 *= +1.0f;
    q3 *= SIGN(eerot[7] + eerot[5]);
  }
  else if (q3 >= q0 && q3 >= q1 && q3 >= q2)
  {
    q0 *= SIGN(eerot[3] - eerot[1]);
    q1 *= SIGN(eerot[6] + eerot[2]);
    q2 *= SIGN(eerot[7] + eerot[5]);
    q3 *= +1.0f;
  }
  else
  {
    printf("Error while converting to quaternion! \n");
  }
  double r = NORM(q0, q1, q2, q3);
  q0 /= r;
  q1 /= r;
  q2 /= r;
  q3 /= r;
  pose.push_back(eetrans[0]);
  pose.push_back(eetrans[1]);
  pose.push_back(eetrans[2]);
  pose.push_back(q1);
  pose.push_back(q2);
  pose.push_back(q3);
  pose.push_back(q0);*/
}

void Kinematics::computeFk(std::vector<double> joints,
			   double eetrans[3],
			   double eerot[9])
{

}
  
bool Kinematics::isIKSuccess(const std::vector< double >& _pose,
			     std::vector< double >& _joints,
			     int& _numOfSolns)
{

   KDL::JntArray q_in, q_out;
   KDL::Frame p_in;
   KDL::Twist bounds = KDL::Twist::Zero();
   
   p_in.p = KDL::Vector(_pose[0], _pose[1], _pose[2]);
   p_in.M = KDL::Rotation::Quaternion(_pose[6], _pose[5], _pose[4], _pose[3]);
   
   fillZeros(q_in);
   
   int sols = ik_solver_->CartToJnt(q_in, p_in, q_out, bounds);
   if(sols > 0 )
   {
     arrayToVector(q_out, _joints);
     _numOfSolns = sols;
     return true;
   }
   
   return false;
}


const std::string Kinematics::getRobotName()
{
  return urdf_->getName();
}

bool Kinematics::isIkSuccesswithTransformedBase(const geometry_msgs::Pose& base_pose,
                                                const geometry_msgs::Pose& grasp_pose, std::vector<double>& joint_soln,int& numOfSolns)
{/*
  // Creating a transformation out of base pose
  tf2::Vector3 base_vec(base_pose.position.x, base_pose.position.y, base_pose.position.z);
  tf2::Quaternion base_quat(base_pose.orientation.x, base_pose.orientation.y, base_pose.orientation.z,
                            base_pose.orientation.w);
  base_quat.normalize();
  tf2::Transform base_trns;
  base_trns.setOrigin(base_vec);
  base_trns.setRotation(base_quat);

  // Inverse of the transformation
  tf2::Transform base_trns_inv;
  base_trns_inv = base_trns.inverse();

  // Creating a transformation of grasp pose
  tf2::Vector3 grasp_vec(grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z);
  tf2::Quaternion grasp_quat(grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z,
                             grasp_pose.orientation.w);
  grasp_quat.normalize();
  tf2::Transform grasp_trns;
  grasp_trns.setOrigin(grasp_vec);
  grasp_trns.setRotation(grasp_quat);

  // Transforming grasp pose to origin from where we can check for Ik
  tf2::Transform new_grasp_trns;
  // new_grasp_trns = grasp_trns * base_trns_inv;
  new_grasp_trns = base_trns_inv * grasp_trns;
  // Creating a new grasp pose in the origin co-ordinate
  std::vector< double > new_grasp_pos;
  tf2::Vector3 new_grasp_vec;
  tf2::Quaternion new_grasp_quat;
  new_grasp_vec = new_grasp_trns.getOrigin();
  new_grasp_quat = new_grasp_trns.getRotation();
  new_grasp_quat.normalize();
  new_grasp_pos.push_back(new_grasp_vec[0]);
  new_grasp_pos.push_back(new_grasp_vec[1]);
  new_grasp_pos.push_back(new_grasp_vec[2]);
  new_grasp_pos.push_back(new_grasp_quat[0]);
  new_grasp_pos.push_back(new_grasp_quat[1]);
  new_grasp_pos.push_back(new_grasp_quat[2]);
  new_grasp_pos.push_back(new_grasp_quat[3]);

  // Check the new grasp_pose for Ik
  Kinematics k;
  //std::vector< double > joints;

  //joints.resize(6);
  if (k.isIKSuccess(new_grasp_pos,  joint_soln, numOfSolns))
    return true;
  else
    return false;*/
    return true;
}


void Kinematics::arrayToVector(const KDL::JntArray &_q, std::vector<double> &_js)
{
   _js.clear();
   for(int i = 0; i < _q.data.size(); ++i)
      _js.push_back(_q.data(i));
}

void Kinematics::fillZeros(KDL::JntArray &_q)
{
   _q.data = Eigen::VectorXd::Zero(chain_num_joints_);
   
}




};
