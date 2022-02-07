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

#include <ccssp_local_planner/scored_planner.h>

#include <ros/console.h>


namespace ccssp_local_planner {
  
  ScoredPlanner::ScoredPlanner(base_local_planner::SimpleTrajectoryGenerator* gen, std::vector<base_local_planner::TrajectoryCostFunction*> &critics, std::vector<base_local_planner::TrajectoryCostFunction*>& risk_critics,
                                                            int max_samples) {
    max_samples_ = max_samples;
    traj_gen = gen; //replaced gen_list_
    critics_ = critics;
    risk_critics_ = risk_critics;

    if(false){ // disable risk
        for(auto critic: risk_critics_)
            critics_.push_back(critic);

        risk_critics_ = *new std::vector<base_local_planner::TrajectoryCostFunction*>;
    }


  }


/*
  double SimpleScoredSamplingPlanner::scoreTrajectory(base_local_planner::Trajectory& traj, double best_traj_cost) {
    double traj_cost = 0;
    int gen_id = 0;
    for(std::vector<base_local_planner::TrajectoryCostFunction*>::iterator score_function = critics_.begin(); score_function != critics_.end(); ++score_function) {
      base_local_planner::TrajectoryCostFunction* score_function_p = *score_function;
      if (score_function_p->getScale() == 0) {
        continue;
      }
      double cost = score_function_p->scoreTrajectory(traj);
      if (cost < 0) {
        ROS_DEBUG("Velocity %.3lf, %.3lf, %.3lf discarded by cost function  %d with cost: %f", traj.xv_, traj.yv_, traj.thetav_, gen_id, cost);
        traj_cost = cost;
        break;
      }
      if (cost != 0) {
        cost *= score_function_p->getScale();
      }
      traj_cost += cost;
      if (best_traj_cost > 0) {
        // since we keep adding positives, once we are worse than the best, we will stay worse
        if (traj_cost > best_traj_cost+1000000) { //this is to ignore the constriant
          break;
        }
      }
      gen_id ++;
    }


    return traj_cost;
  }


    double SimpleScoredSamplingPlanner::riskTrajectory(base_local_planner::Trajectory& traj) {
        double traj_risk = 0;

    for(std::vector<base_local_planner::TrajectoryCostFunction*>::iterator score_function = risk_critics_.begin(); score_function != risk_critics_.end(); ++score_function) {
      base_local_planner::TrajectoryCostFunction* score_function_p = *score_function;
      if (score_function_p->getScale() == 0) {
        continue;
      }
      traj_risk =  score_function_p->scoreTrajectory(traj)/254.0;
    }

        return std::max(0.0,traj_risk);
    }
*/












  bool ScoredPlanner::findBestTrajectory(base_local_planner::Trajectory& traj, std::vector<base_local_planner::Trajectory>* all_explored,
              Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f goal, base_local_planner::LocalPlannerLimits* limits, Eigen::Vector3f vsamples_) {
    /* This function calls the ccssp planner: 1) genrate all trajectories with cost and risk vals, 2) if min risk traj is above risk_cc then return false 
    3) run ccssp planner (not all gurobi errors are hundled 100005)
    */

    base_local_planner::Trajectory loop_traj;
    base_local_planner::Trajectory best_traj;
    std::vector<double> traj_cost_list;
    std::vector<double> traj_risk_list;
      std::vector<base_local_planner::Trajectory> all_traj;

    double loop_traj_cost, loop_traj_risk, best_traj_cost = -1;
    bool gen_success;
    int count, count_valid;
    for (std::vector<base_local_planner::TrajectoryCostFunction*>::iterator loop_critic = critics_.begin(); loop_critic != critics_.end(); ++loop_critic) {
      base_local_planner::TrajectoryCostFunction* loop_critic_p = *loop_critic;
      if (loop_critic_p->prepare() == false) {
        ROS_WARN("A scoring function failed to prepare");
        return false;
      }
    }




      std::vector<std::vector<std::vector<double>>> predict_trajectories;  //TODO: subscribe to them (also make sure timestep is smilar to generator) 


      robot_model robotModel(traj_gen, pos, vel, goal, limits, vsamples_, &critics_, &risk_critics_, predict_trajectories, safety_dist); // TODO: initialize
      robotModel.h = planning_h;
      std::cout<<"Planing horizon: "<<robotModel.h<<std::endl;
      



    ccssp solver(&robotModel, risk_cc);
    int best_action_idx=0;

      if(solver.feasiblility()){
        best_traj = solver.get_opt_trajectory();
        best_traj_cost = 1;// temp fix
            }





      if (best_traj_cost >= 0) {
          traj.xv_ = best_traj.xv_;
          traj.yv_ = best_traj.yv_;
          traj.thetav_ = best_traj.thetav_;
          traj.cost_ = best_traj_cost;
          traj.resetPoints();
          double px, py, pth;
          for (unsigned int i = 0; i < best_traj.getPointsSize(); i++) {
              best_traj.getPoint(i, px, py, pth);
              traj.addPoint(px, py, pth);
          }
      }






    return best_traj_cost >= 0;
  }

  
}// namespace
