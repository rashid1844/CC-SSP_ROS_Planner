//
// Created by rashi on 7/26/2021.
//

#ifndef CCSSP_LOCAL_PLANNER_NODE_H
#define CCSSP_LOCAL_PLANNER_NODE_H

#include <cmath>
#include <Eigen/Core>
#include <string>
#include <vector>
#include <base_local_planner/trajectory.h>
#include <base_local_planner/trajectory_sample_generator.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/simple_trajectory_generator.h>

#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/twirling_cost_function.h>



namespace ccssp_local_planner {




//TODO: need to add the following: 1) Action should return a set of trajectories with risk, reward
//                                 2) We need a function that combines trajectories (for determinctic transition case)

// This file contains 1) trajectory cost and risk functions (must add predict traj to risk function)
//                    2) state and action struct, 3) robot model (risk, reward, trans, ....)
//                    4) Node struct used in the ccssp tree
//













struct State{Eigen::Vector3f state; double probability=1.0;};

struct Action{base_local_planner::Trajectory action; double risk, reward;}; // TODO: should include 1) base_local_planner::Trajectory action  , 2) risk and reward ..... which is computed when functuon Actions is called




struct robot_model{
    int h=1;
    State init_state;
    Eigen::Vector3f pos, vel, goal, vsamples_;
    base_local_planner::LocalPlannerLimits* limits;

      std::vector<base_local_planner::TrajectoryCostFunction*>* critics_;
      std::vector<base_local_planner::TrajectoryCostFunction*>* risk_critics_;
      base_local_planner::SimpleTrajectoryGenerator* traj_gen;

      std::vector<std::vector<std::vector<double>>> predict_trajectories;
      double safety_dist;

      //constructor
    robot_model(base_local_planner::SimpleTrajectoryGenerator* traj_gen_, Eigen::Vector3f pos_, Eigen::Vector3f vel_, Eigen::Vector3f goal_, base_local_planner::LocalPlannerLimits* limits_, Eigen::Vector3f vsamples__,
        std::vector<base_local_planner::TrajectoryCostFunction*>* critics__ ,std::vector<base_local_planner::TrajectoryCostFunction*>* risk_critics__, std::vector<std::vector<std::vector<double>>> predict_trajectories_, double safety_dist_){
        pos = pos_; vel = vel_; goal = goal_; limits = limits_; vsamples_ = vsamples__;
        critics_ = critics__; risk_critics_ = risk_critics__;
        init_state.state = pos;
        traj_gen = traj_gen_;
        predict_trajectories = predict_trajectories_;
        safety_dist = safety_dist_;
    }
    robot_model(){} //default constructor

    double risk(State state, Action action){return action.risk;}
    double reward(State state, Action action){return action.reward;}
    int get_h(){return h;}
    State get_init_state(){return init_state;}


    std::vector<State> Transition(State state, Action action){
    // get last point of traj action.action
        State new_state;
        double x, y, th;
        action.action.getEndpoint(x,y,th); //copied by refence

        new_state.state = *new Eigen::Vector3f(x, y, th);
        return *new std::vector<State> ({new_state});} 


      double scoreTrajectory(base_local_planner::Trajectory& traj) {
    double traj_cost = 0;
    int gen_id = 0;
    for(std::vector<base_local_planner::TrajectoryCostFunction*>::iterator score_function = critics_->begin(); score_function != critics_->end(); ++score_function) {
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
        if (traj_cost > 1000000) { //this is to ignore the constriant
          break;
        }
      
      gen_id ++;
    }


    return traj_cost;
  }


    double riskTrajectory(base_local_planner::Trajectory& traj) {
        //risk from costmap
        double traj_risk = 0;
        for(std::vector<base_local_planner::TrajectoryCostFunction*>::iterator score_function = risk_critics_->begin(); score_function != risk_critics_->end(); ++score_function) {
          base_local_planner::TrajectoryCostFunction* score_function_p = *score_function;
          if (score_function_p->getScale() == 0) {
            continue;
          }
          traj_risk =  score_function_p->scoreTrajectory(traj)/254.0;
        }


        // risk from prediction
        double predict_risk = 0;
        double x, y, th, dist;
        for(std::vector<std::vector<double>> predict_trajectory: predict_trajectories){
          if(predict_trajectory.size() == 0)
            continue;

          int min_size = std::min((int)predict_trajectory.size(),(int) traj.getPointsSize());
          for(int i=0; i< min_size; i++){
            traj.getPoint(i, x,y,th);
            dist = std::pow(x - predict_trajectory[i][0], 2) + std::pow(y - predict_trajectory[i][1], 2);
            if(dist > std::pow(safety_dist, 2)){
              predict_risk =1;
              break; }
          }
          if(predict_risk == 1)
            break;
        }


        return std::max(0.0, std::max(predict_risk,traj_risk));
    }

    

    std::vector<Action> Actions(State state){
        // returns a list of actions using trajectory generator
        std::vector<Action> actions;
        base_local_planner::Trajectory loop_traj;
        double traj_cost, traj_risk;
        bool gen_success;
        Action action;

          traj_gen->initialise(state.state, vel, goal, limits, vsamples_);
          while (traj_gen->hasMoreTrajectories()) {
              gen_success = traj_gen->nextTrajectory(loop_traj);
              if (gen_success == false) 
                  continue;
              traj_cost = scoreTrajectory(loop_traj);
              traj_risk = riskTrajectory(loop_traj);
              if(traj_cost<0)
                continue;
            action = *new Action;
            action.action = loop_traj;
            action.risk = traj_risk;
            action.reward = traj_cost;
            actions.push_back(action);
            }
        return actions;
    } 
    


};



struct Node{
    int BFS_idx, depth;
    double risk=0, reward=0;
    int id;
    std::vector<std::vector<double>> T_prob;
    std::vector<double> x_vals;
    std::vector<int> parents_act, parents_trans_idx, z_vals;
    std::vector<Action> action_list;
    // parents_act: list of act_idx of each parent, parents_trans_idx: list of state_idx of each parent

    State state;

    int best_action;
    std::vector<Node*> parents;
    std::vector<std::vector<Node*>> child_list;
    bool terminal= false;

    void find_best_action(){
        for(int i=0; i<this->z_vals.size(); i++){
            if(this->z_vals[i] == 1){
                this->best_action =  i;
                break;
            }
        }
    }


};


};


#endif //CCSSP_LOCAL_PLANNER_NODE_H
