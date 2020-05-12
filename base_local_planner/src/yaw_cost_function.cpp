/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Universite de Sherbrooke
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Cedric Godin
 *********************************************************************/

#include <base_local_planner/yaw_cost_function.h>
#include <angles/angles.h>

namespace base_local_planner {

    YawCostFunction::YawCostFunction() {

        ros::NodeHandle nh("yaw_cost");   
        pub_goal_th_ = nh.advertise<std_msgs::Float64>("goal_th", 1);
        pub_start_th_ = nh.advertise<std_msgs::Float64>("start_th", 1);
        pub_end_th_ = nh.advertise<std_msgs::Float64>("end_th", 1);
    }


    void YawCostFunction::setGoalPose(Eigen::Vector3f goal_pose) {
        goal_pose_ = goal_pose;
    }

    void YawCostFunction::setCurrentPose(Eigen::Vector3f current_pose) {
        current_pose_ = current_pose;
    }

    void YawCostFunction::setCutoffDistance(double cutoff_distance) {
        cutoff_distance_ = cutoff_distance;
    }

    void YawCostFunction::useYawFromPlan(bool use_yaw_from_plan) {
        use_yaw_from_plan_ = use_yaw_from_plan;
    }

    double YawCostFunction::scoreTrajectoryWithLogging(Trajectory &traj, bool logging) {

        // Cost is zero if trajectory is empty
        if (traj.getPointsSize() == 0) {
          return 0;
        }

        // Goal theta
        double gth = fmod(goal_pose_[2], 2.0*M_PI);

        // Trajectory endpoint
        double endx, endy, endth;
        traj.getEndpoint(endx, endy, endth);

        // Trajectory velocity
        double vth = fabs(traj.thetav_);
        double vtrans = sqrt(traj.xv_*traj.xv_ + traj.yv_*traj.yv_);

        // Cost according to if the trajectory steers in the correct direction
        double thcost = fabs(angles::shortest_angular_distance(endth, gth));

        // Cost according to the translation velocity
        double vcost = 10*vtrans*thcost;

        if( logging ) {
            
            std_msgs::Float64 msg;

            msg.data = gth;
            pub_goal_th_.publish(msg);

            msg.data = fmod(current_pose_[2], 2.0*M_PI);
            pub_start_th_.publish(msg);

            msg.data = endth;
            pub_end_th_.publish(msg);

            /*msg.data = thcost;
            pub_delta_ahead_.publish(msg);

            msg.data = gdst;
            pub_goal_dst_.publish(msg);*/
        }

        double dircost = -2.0 * angles::shortest_angular_distance(endth, gth) / traj.thetav_;
        if (dircost < 0) dircost = 0;

        return thcost * (1.0 + 2.0/vth) + vcost;

    }

    double YawCostFunction::scoreTrajectory(Trajectory &traj) {
        return scoreTrajectoryWithLogging(traj, false);
    }

} /* namespace base_local_planner */
