/*
 * Copyright (c) 2012, The Johns Hopkins University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of The Johns Hopkins University. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __BARD_COMPONENTS_CONTROLLERS_CARTESIAN_POSE_H
#define __BARD_COMPONENTS_CONTROLLERS_CARTESIAN_POSE_H

#include <iostream>

#include <boost/scoped_ptr.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_pinv_nso.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>

#include <tf/tf.h>

#include <geometry_msgs/TransformStamped.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace bard_components {
  namespace controllers {
    class CartesianPose : public RTT::TaskContext
    {
      // RTT Properties
      std::string robot_description_;
      std::string joint_prefix_;
      std::string root_link_;
      std::string tip_link_;
      std::string target_frame_;
      std::vector<double> kp_; // Proportional gains                                                                                                                             
      std::vector<double> kd_; // Derivative gains                                                                                                                               

      // RTT Ports
      RTT::InputPort<KDL::JntArrayVel> positions_in_port_;
      RTT::OutputPort<KDL::JntArrayVel> positions_out_port_;
      RTT::OutputPort<KDL::JntArray> torques_out_port_;
      RTT::OutputPort<trajectory_msgs::JointTrajectory> trajectories_out_port_;

      RTT::OperationCaller<geometry_msgs::TransformStamped(const std::string&, const std::string&)> tf_lookup_transform_;

    public:
      CartesianPose(std::string const& name);
      bool configureHook();
      bool startHook();
      void updateHook();
      void stopHook();
      void cleanupHook();

      void test_ik();
      void compute_ik(bool debug);
    private:

      // Kinematic properties
      unsigned int n_dof_;
      KDL::Chain kdl_chain_;
      KDL::Tree kdl_tree_;
      urdf::Model urdf_model_;


      // Working variables
      KDL::JntArrayVel positions_;
      KDL::JntArrayVel positions_des_;
      KDL::JntArray torques_;

      // Joint limits
      KDL::JntArray joint_limits_min_;
      KDL::JntArray joint_limits_max_;

      // KDL IK solver which accounts for joint limits
      boost::scoped_ptr<KDL::ChainIkSolverPos> kdl_ik_solver_top_;
      boost::scoped_ptr<KDL::ChainIkSolverVel> kdl_ik_solver_vel_;

      // KDL FK solver
      boost::scoped_ptr<KDL::ChainFkSolverPos> kdl_fk_solver_pos_;

      geometry_msgs::TransformStamped tip_frame_msg_;
      tf::Transform tip_frame_tf_;
      KDL::Frame tip_frame_;
      KDL::Frame tip_frame_des_;

      trajectory_msgs::JointTrajectory trajectory_;

    };
  }
}


#endif // ifndef __BARD_COMPONENTS_CONTROLLERS_CARTESIAN_POSE_H


