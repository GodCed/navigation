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

    void YawCostFunction::setGoalPose(Eigen::Vector3f goal_pose) {
        goal_pose_ = goal_pose;
    }

    void YawCostFunction::setCurrentPose(Eigen::Vector3f current_pose) {
        current_pose_ = current_pose;
    }

    double YawCostFunction::scoreTrajectory(Trajectory &traj) {

        double goal_distance_sq =
            (goal_pose_[0] - current_pose_[0]) * (goal_pose_[0] - current_pose_[0]) +
            (goal_pose_[1] - current_pose_[1]) * (goal_pose_[1] - goal_pose_[1]);

        double delta_goal = goal_pose_[2] - current_pose_[2];
        delta_goal >= 180.0 ? delta_goal - 360.0 : delta_goal;
        delta_goal <= -180.0 ? delta_goal + 360.0 : delta_goal;
        delta_goal = fabs(delta_goal);

        double lx, ly, lyaw;
        traj.getEndpoint(lx, ly, lyaw);

        double delta_ahead = fabs(atan2(ly - current_pose_[1], lx - current_pose_[0]));

        if( goal_distance_sq > cutoff_distance_ * cutoff_distance_) {
            return delta_ahead;
        }

        else {
            return delta_goal;
        }
    }

} /* namespace base_local_planner */