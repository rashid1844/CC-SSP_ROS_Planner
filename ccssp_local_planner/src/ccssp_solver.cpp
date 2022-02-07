//
// Created by rashid on 7/26/2021.
//

#include "ccssp_local_planner/ccssp_solver.h"





namespace ccssp_local_planner {

/*
ccssp::ccssp(){
    std::cout<<"ccssp Constructor \n";

    ccssp::expand_tree();
    ccssp::ILP_solver();
}
*/

ccssp::ccssp(robot_model* models, double risk_cc){
    cc = risk_cc; 
    std::cout<<"ccssp robot Constructor \n";
    model = models;

    ccssp::expand_tree();
    ccssp::ILP_solver();

}



ccssp::~ccssp(){}


int ccssp::get_root_best_action(){

    return tree[0]->best_action;
}


base_local_planner::Trajectory ccssp::get_opt_trajectory(){
    // 1) get a list of best actions, 2) create trajectory and combine all actions into one trajecotry (use add_vectors function)
    std::vector<base_local_planner::Trajectory*> best_traj_list;
    Node* current_node = tree[0];

    while(!current_node->terminal){
        best_traj_list.push_back(&current_node->action_list[current_node->best_action].action);
        current_node = current_node->child_list[current_node->best_action][0]; //assuming no trans prob
    }

    double x, y, th;
    base_local_planner::Trajectory opt_traj(best_traj_list[0]->xv_, best_traj_list[0]->yv_, best_traj_list[0]->thetav_, best_traj_list[0]->time_delta_, 0);
    //< traj constructor is The x, y, and theta velocities of the trajectory, The time gap between points, zero for expected number of points in traj


    // get all points from best_traj_list to opt_traj
    for(int i=0; i < best_traj_list.size(); i++){
        for(int j=0; j < best_traj_list[i]->getPointsSize(); j++){
            best_traj_list[i]->getPoint(j, x,y,th);
            opt_traj.addPoint(x, y, th);
        }
    }


return opt_traj; //*best_traj_list[0];  
}




Node* ccssp::node_search(int id){

    for(Node *node: tree)
        if(node->id == id)
            return node;

    return NULL;
}






void ccssp::expand_tree(){    std::cout<<" expand tree start\n";
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    std::queue<Node*> opennodes; // saves new_node by reference
    int BFS_idx = 0, act_idx, state_idx, id=0;

    Node* node;
    Node* new_node;
    std::vector<State> new_states;
    std::vector<Node*> child_list;
    std::vector<double> T_prob_list;
    // tree contains real nodes, child and parent lists are by reference
    // probability values are by value


    // TODO: define b0 new_node
    new_node = new Node;
    new_node->BFS_idx = BFS_idx;
    BFS_idx ++;
    new_node->state = model->get_init_state();
    new_node->depth = 0;
    //new_node->id = model->hash(new_node->state, new_node->depth);
    // TODO: define b0 new_node

    tree.push_back(new_node);  // new_node is copied to list
    opennodes.push(tree.back()); // node ref from tree (back is last element)


    while (!opennodes.empty()){
        node = opennodes.front();
        opennodes.pop();
        act_idx = 0;

        for(Action action: model->Actions(node->state)){
            node->action_list.push_back(action);
            new_states = model->Transition(node->state, action);

            child_list = *new std::vector<Node*>;
            T_prob_list = *new std::vector<double>;
            state_idx = 0;

            for(const State& new_state: new_states){// for loop creates st new copy and not by reference
                T_prob_list.push_back(new_state.probability);
                //id = model->hash(new_state, node->depth + 1); //uncomment to utilize the graph
                //new_node = node_search(id);
                new_node = NULL;

                if(new_node == NULL){
                    new_node = new Node;  // creates st new new_node
                    new_node->state = new_state;
                    new_node->depth = node->depth + 1;
                    new_node->id = id;
                    new_node->BFS_idx = BFS_idx;
                    BFS_idx++;
                    new_node->risk = model->risk(node->state, action); // This is based on old state and action
                    new_node->reward = model->reward(node->state, action);

                    if(new_node->depth>= model->get_h()) // if terminal set terminal, else add to opennodes
                    {new_node->terminal = true;
                        tree.push_back(new_node); }
                    else
                    {tree.push_back(new_node);
                        opennodes.push(tree.back()); }

                }

                new_node->parents_act.push_back(act_idx);
                new_node->parents_trans_idx.push_back(state_idx);
                new_node->parents.push_back(node);




                child_list.push_back(tree.back());
                state_idx ++;
            }

            node->child_list.push_back(child_list);
            node->T_prob.push_back(T_prob_list);

            act_idx ++;
        }
    }

      /*
      std::cout<<"Tree cost list: ";
      for(auto node: tree)
          std::cout<<node->reward<<" ";
      std::cout<<std::endl;
      */

    std::cout<<"Node count: "<<tree.size()<<"\n";
    std::cout<<"tree expanded\n";
    tree_expand_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count()/1000. ;
    std::cout << "Tree Time = " << tree_expand_time << "s" << std::endl;

}



void ccssp::ILP_solver(){    std::cout<<"ccssp ILP solver \n";
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    int var_size = tree.size();

    try {

        // Create an environment
        GRBEnv env = GRBEnv(true);
        env.set("LogFile", "mip1.log");
        env.start();

        // Create an empty grbmodel
        GRBModel grbmodel = GRBModel(env);

        grbmodel.set(GRB_IntParam_OutputFlag, 0);

        // Compute child_count
        std::vector<int> child_count;
        for(Node* node: tree)
            if(!node->terminal)
                child_count.push_back(node->child_list.size());

        int nonterminal_count = child_count.size();

        // Variables
        std::vector<GRBVar*> x;
        std::vector<GRBVar*> z;

        for (int i = 0; i < child_count.size(); i++) {
               x.push_back(grbmodel.addVars(child_count[i], GRB_SEMICONT)); //define the range
               z.push_back(grbmodel.addVars(child_count[i], GRB_BINARY));
        }

        std::vector<GRBLinExpr> flow_in={1.};
        for(int i=1; i<var_size; i++) {
            flow_in.push_back(0.);
            for (int p_idx = 0; p_idx < tree[i]->parents.size(); p_idx++) {
                int p_BFS_idx = tree[i]->parents[p_idx]->BFS_idx;
                flow_in[i] += x[p_BFS_idx][tree[i]->parents_act[p_idx]] * tree[p_BFS_idx]->T_prob[tree[i]->parents_act[p_idx]][tree[i]->parents_trans_idx[p_idx]] * (1. - tree[p_BFS_idx]->risk);
            }
        }

        // Set objective:
        GRBLinExpr obj = 0;
        for (int i = 0; i < var_size; i++)
            obj += flow_in[i] * tree[i]->reward;
        grbmodel.setObjective(obj, GRB_MINIMIZE);


        // x semicont
        /*
        GRBLinExpr x_tot = 0;
        for (int i = 0; i < child_count.size(); i++) {
            x_tot=0;
            for(int j=0; j<child_count[i]; j++)
                x_tot += x[i][j];
            grbmodel.addConstr(x_tot <= 1);
        }
        */


        // Add risk constraint:
        GRBLinExpr risk_cst = 0;
        for (int i = 0; i < var_size; i++)
            risk_cst += flow_in[i] * tree[i]->risk;
        grbmodel.addConstr(risk_cst <= cc);


        // Force select root
        GRBLinExpr root_child=0.;
        for(int i=0; i<child_count[0]; i++)
            root_child += x[0][i];
        grbmodel.addConstr(root_child == 1.);


        // Flow in = flow out
        GRBLinExpr node_child;// flow out
        for(int i=1; i<nonterminal_count; i++){
            node_child = 0.;
            for(int j=0; j<child_count[i]; j++)
                node_child += x[i][j];
            grbmodel.addConstr(node_child == flow_in[i]);
        }


        // ILP const
        for(int i=0; i<nonterminal_count; i++) {
            node_child = 0.;
            for(int j=0; j<child_count[i]; j++){
                node_child += z[i][j];
                grbmodel.addConstr(x[i][j] <= z[i][j]); // z const if x>0 then z is selected
                grbmodel.addConstr(x[i][j] >= 0.);
                grbmodel.addConstr(x[i][j] <= 1.);

            }
            grbmodel.addConstr(node_child <= 1); // select one z child
        }



        // Optimize grbmodel
        grbmodel.optimize();

        if (GRB_OPTIMAL != 2) // or ==3
            feasible = false;
        else{

            feasible = true;
        std::vector<std::vector<int>> output_z;
        std::vector<int> temp_vec;

        for(int i=0; i<nonterminal_count; i++) {
            temp_vec = *new std::vector<int>;
            for (int j = 0; j < child_count[i]; j++)
                temp_vec.push_back(z[i][j].get(GRB_DoubleAttr_X));
            output_z.push_back(temp_vec);
        }

        for(int i=0; i<nonterminal_count; i++)
            tree[i]->z_vals = output_z[i];

        for(int i=0; i<nonterminal_count; i++)
            tree[i]->find_best_action();


        /*
          std::cout<<"Root z list: ";
      for(auto zz: tree[0]->z_vals)
          std::cout<<zz<<" ";
      std::cout<<std::endl;
        */

            //std::cout << x.get(GRB_StringAttr_VarName) << " "<< x.get(GRB_DoubleAttr_X) << std::endl;

        ILP_obj = grbmodel.get(GRB_DoubleAttr_ObjVal);
        std::cout << "Obj: " << ILP_obj << std::endl;

}

    } catch(GRBException e) {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    } catch(...) {
        std::cout << "Exception during optimization" << std::endl;
                    feasible = false;

    }


    ILP_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count()/1000. ;
    std::cout << "ILP Time = " << ILP_time << "s" << std::endl;

    std::cout << "Total Time = " << ILP_time + tree_expand_time << "s" << std::endl;
    std::cout << "risk_cc = " << cc << std::endl;


}



};



