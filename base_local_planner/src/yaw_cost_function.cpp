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

namespace base_local_planner {

    YawCostFunction::YawCostFunction() {

        ros::NodeHandle nh("yaw_cost");
        pub_goal_dst_ = nh.advertise<std_msgs::Float64>("goal_distance", 1);
        pub_delta_goal_ = nh.advertise<std_msgs::Float64>("delta_goal", 1);
        pub_delta_ahead_ = nh.advertise<std_msgs::Float64>("delta_ahead", 1);

    }


    void YawCostFunction::setGoalPose(Eigen::Vector3f goal_pose) {
        goal_pose_ = goal_pose;
    }

    void YawCostFunction::setCurrentPose(Eigen::Vector3f current_pose) {
        current_pose_ = current_pose;
    }

    double YawCostFunction::scoreTrajectoryWithLogging(Trajectory &traj, bool logging) {

        // AGV current position
        double x = current_pose_[0];
        double y = current_pose_[1];
        double th = fmod(current_pose_[2], 2.0*M_PI);
        double vtrans = traj.xv_*traj.xv_ + traj.yv_*traj.yv_;

        // Goal position
        double gx = goal_pose_[0];
        double gy = goal_pose_[1];
        double gth = fmod(goal_pose_[2], 2.0*M_PI);
        double gdst = (gx-x)*(gx-x)+(gy-y)*(gy-y);

        // Trajectory endpoint
        double endx, endy, endth;
        if( !traj.getPointsSize() > 0)
            return 0.0;
        traj.getEndpoint(endx, endy, endth);
        endth = atan2(endy-y, endx-x);
        endth = fmod(endth + 2.0*M_PI, 2.0*M_PI);

        // Switch target_yaw according to goal distance
        double target_yaw;
        if(gdst <= cutoff_distance_*cutoff_distance_)
            target_yaw = gth;
        else
            target_yaw = endth;

        // Angular distance to go to desired yaw
        double delta_yaw = target_yaw - th;
        delta_yaw = delta_yaw >= M_PI ? delta_yaw - 2.0*M_PI : delta_yaw;
        delta_yaw = delta_yaw <= -M_PI ? delta_yaw + 2.0*M_PI : delta_yaw;

        // Cost according to if the trajectory steers in the correct direction
        double thcost = (delta_yaw - traj.thetav_)*(delta_yaw - traj.thetav_);

        // Cost according to the translation velocity
        double vcost = fabs(delta_yaw) * vtrans;

        if( logging ) {
            
            std_msgs::Float64 msg;

            msg.data = endth;
            pub_delta_goal_.publish(msg);

            msg.data = th;
            pub_delta_ahead_.publish(msg);

            msg.data = gdst;
            pub_goal_dst_.publish(msg);
        }

        return thcost + vcost;

    }

    double YawCostFunction::scoreTrajectory(Trajectory &traj) {
        return scoreTrajectoryWithLogging(traj, false);
    }

} /* namespace base_local_planner */