//
// Created by rashid on 7/26/2021.
//



#ifndef CCSSP_LOCAL_PLANNER_CCSSP_H
#define CCSSP_LOCAL_PLANNER_CCSSP_H

#include "vector"
#include "ccssp_local_planner/Node.h"
#include "gurobi_c++.h"
#include "iostream"
#include <string>
#include <vector>
#include "queue"
#include "ctime"
#include <chrono>
#include <base_local_planner/trajectory.h>

namespace ccssp_local_planner {


class ccssp {
private:

public:
    double cc=0.4;
    bool feasible;
    double tree_expand_time, ILP_time, ILP_obj;
    std::vector<Node*> tree;
    robot_model* model;  // TODO: change model
    Node* node_search(int id);

    ccssp();
    ccssp(robot_model* models, double risk_cc);
    ~ccssp();

    void expand_tree();
    void ILP_solver();
    int get_root_best_action();
    base_local_planner::Trajectory get_opt_trajectory();

    bool feasiblility(){return feasible;}


    std::vector<double> add_vectors(std::vector<double> a, std::vector<double> b){
        a.insert(a.end(), b.begin(), b.end());
        return a;
    }



};

};


#endif //CCSSP_LOCAL_PLANNER_CCSSP_H
