/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 * Author: TKruse
 *********************************************************************/

#include <base_local_planner/obstacle_cost_function.h>
#include <cmath>
#include <Eigen/Core>
#include <ros/console.h>

namespace base_local_planner {

ObstacleCostFunction::ObstacleCostFunction(costmap_2d::Costmap2D* costmap) 
    : costmap_(costmap),
      sum_scores_(false),
      max_acc_x_(1000.0),
      max_acc_y_(1000.0),
      max_trans_vel_(0.0) {
  if (costmap != NULL) {
    world_model_ = new base_local_planner::CostmapModel(*costmap_);
  }
}

ObstacleCostFunction::~ObstacleCostFunction() {
  if (world_model_ != NULL) {
    delete world_model_;
  }
}


void ObstacleCostFunction::setParams(double max_acc_x, double max_acc_y, double max_trans_vel) {
  max_acc_x_     = max_acc_x;
  max_acc_y_     = max_acc_y;
  max_trans_vel_ = max_trans_vel;
}

void ObstacleCostFunction::setFootprint(std::vector<geometry_msgs::Point> footprint_spec) {
  footprint_spec_ = footprint_spec;
}

bool ObstacleCostFunction::prepare() {
  return true;
}

double ObstacleCostFunction::scoreTrajectory(Trajectory &traj) {
  double cost = 0;
  double px, py, pth;
  if (footprint_spec_.size() == 0) {
    // Bug, should never happen
    ROS_ERROR("Footprint spec is empty, maybe missing call to setFootprint?");
    return -9;
  }

  // In normal order
  for (unsigned int i = 0; i < traj.getPointsSize(); ++i) {

  // In reverse order
  //for (unsigned int j = 0; j < traj.getPointsSize(); ++j) {
  //    unsigned int i = traj.getPointsSize() - 1 - j;

    traj.getPoint(i, px, py, pth);
    double f_cost = footprintCost(px, py, pth,
        traj.xv_, traj.yv_, traj.thetav_,
        max_acc_x_, max_acc_y_, max_trans_vel_,
        footprint_spec_,
        costmap_, world_model_);

    if(f_cost < 0){
        //ROS_WARN("Cost = %f at (%i) [%f, %f, %f] with [%f, %f]", f_cost, i, px, py, pth, traj.xv_, traj.yv_);
        return f_cost;
    }

    if(sum_scores_)
        cost +=  f_cost;
    else
        cost = f_cost;
  }
  return cost;
}

/*
double ObstacleCostFunction::getScalingFactor(Trajectory &traj, double scaling_speed, double max_trans_vel, double max_scaling_factor) {
  double vmag = hypot(traj.xv_, traj.yv_);

  //if we're over a certain speed threshold, we'll scale the robot's
  //footprint to make it either slow down or stay further from walls
  double scale = 1.0;
  if (vmag > scaling_speed) {
    //scale up to the max scaling factor linearly... this could be changed later
    double ratio = (vmag - scaling_speed) / (max_trans_vel - scaling_speed);
    scale = max_scaling_factor * ratio + 1.0;
  }
  return scale;
}
*/

double ObstacleCostFunction::footprintCost (
    const double& x,
    const double& y,
    const double& th,
    const double& vel_x,
    const double& vel_y,
    const double& vel_theta,
    const double& max_acc_x,
    const double& max_acc_y,
    const double& max_trans_vel,
    std::vector<geometry_msgs::Point> footprint_spec,
    costmap_2d::Costmap2D* costmap,
    base_local_planner::WorldModel* world_model) {

  //check if the footprint is legal
  // TODO: Cache inscribed radius
    double cos_th = cos(th);
    double sin_th = sin(th);

    // dx and dy in robot frame equal stopping distances
    double dx = 0.5*vel_x*vel_x/max_acc_x;
    double dy = 0.5*vel_y*vel_y/max_acc_y;

    // This assumes that the frame is in the middle of the footprint, i.e., there are x, y < 0 and x, y > 0
    std::vector<geometry_msgs::Point> scaled_oriented_footprint;
    for(unsigned int i  = 0; i < footprint_spec.size(); ++i) {
        geometry_msgs::Point new_pt;
        //new_pt.x = x + (scale * footprint_spec[i].x * cos_th - scale * footprint_spec[i].y * sin_th);
        //new_pt.y = y + (scale * footprint_spec[i].x * sin_th + scale * footprint_spec[i].y * cos_th);

        double dxc = 0.0;
        double dyc = 0.0;
        if (footprint_spec[i].x > 0.0 && vel_x > 0.0) {
            dxc = dx;
        } else if (footprint_spec[i].x < 0.0 && vel_x < 0.0) {
            dxc = -dx;
        }
        if (footprint_spec[i].y > 0.0 && vel_y > 0.0) {
            dyc = dy;
        } else if (footprint_spec[i].y < 0.0 && vel_y < 0.0) {
            dyc = -dy;
        }
        new_pt.x = x + (footprint_spec[i].x + dxc) * cos_th - (footprint_spec[i].y + dyc) * sin_th;
        new_pt.y = y + (footprint_spec[i].x + dxc) * sin_th + (footprint_spec[i].y + dyc) * cos_th;
        scaled_oriented_footprint.push_back(new_pt);
        //ROS_INFO("Point: [%f, %f] --> [%f, %f]", x+footprint_spec[i].x*cos_th-footprint_spec[i].y*sin_th,
        //                                         y+footprint_spec[i].x*sin_th+footprint_spec[i].y*cos_th,
        //                                         new_pt.x, new_pt.y);
    }

  //double footprint_cost = world_model->footprintCost(x, y, th, scaled_oriented_footprint);
    geometry_msgs::Point robot_position;
    robot_position.x = x;
    robot_position.y = y;
    double footprint_cost = world_model->footprintCost(robot_position, scaled_oriented_footprint, 0.0, 0.0);//ToDo: don't hardcode
    //double footprint_cost = world_model->footprintCost(x, y, th, footprint_spec);

  if (footprint_cost < 0) {
      //ROS_INFO("Footprint cost = %f at [%f, %f, %f]", footprint_cost, x, y, th);
      //for (unsigned int i = 0; i < scaled_oriented_footprint.size(); i++) {
      //    std::cout << "\tx: " << scaled_oriented_footprint[i].x << ", y: " << scaled_oriented_footprint[i].y << std::endl;
      //}
    return -6.0;
  }
  unsigned int cell_x, cell_y;

  //we won't allow trajectories that go off the map... shouldn't happen that often anyways
  if ( ! costmap->worldToMap(x, y, cell_x, cell_y)) {
      ROS_INFO("Trajectories off map...");
    return -7.0;
  }

  //double occ_cost = std::max(std::max(occ_cost, footprint_cost), double(costmap->getCost(cell_x, cell_y)));
  double center_cost = double(costmap->getCost(cell_x, cell_y));
  double occ_cost = std::max(footprint_cost, center_cost);

  // Limit velocity based on costs
  // ToDo: efficiency!
  // Choose either based on occ_cost or center cost
  double vel_cost = occ_cost; // or center_cost
  //double max_vel = (1 - occ_cost / 255.0) * max_trans_vel;
  //double max_vel = (1 - center_cost / 255.0) * max_trans_vel;
  double max_vel = 0.0;
  double thresh_vel = 0.2*max_trans_vel;
  unsigned int thresh_cost = 128;
  if (vel_cost < thresh_cost) {
      max_vel = max_trans_vel - (max_trans_vel - thresh_vel)/thresh_cost*vel_cost;
  } else {
      max_vel = thresh_vel;
      //max_vel = thresh_vel - thresh_vel/(254-thresh_cost)*(vel_cost-thresh_cost);
  }

  if (hypot(vel_x, vel_y) > max_vel) {// && i == traj.getPointsSize() - 1)
      return -5.0;
  }


  return occ_cost;
}

} /* namespace base_local_planner */
