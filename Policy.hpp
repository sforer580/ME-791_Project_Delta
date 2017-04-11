//
//  Policy.hpp
//  ME-791_SF_Project_Delta_project
//
//  Created by Scott S Forer on 4/6/17.
//  Copyright © 2017 Scott S Forer. All rights reserved.
//

#ifndef Policy_hpp
#define Policy_hpp

#include <stdio.h>
using namespace std;

class Policy
{
    friend class EA;
    friend class Simulator;
    friend class Agent;
    friend class Parameters;
    friend class neural_network;
    
public:
    vector<double> weights;
    double fitness;
    double current_x;
    double current_y;
    double current_theta;
    double current_omega;
    double current_dist_to_goal;
    double current_u;
    double new_x;
    double new_y;
    double new_theta;
    double new_omega;
    double new_dist_to_goal;
    double new_u;
    
    int in_map = 0;
    
    vector<double> angle_error;
    
    vector<double> path;
    vector<double> all_theta;
    vector<double> all_omega;
    vector<double> all_dist_to_goal;
    vector<double> all_u;
};

#endif /* Policy_hpp */
