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

    int in_map = 1;                 //0=false, 1=true
    int in_goal_box = 0;            //0=false, 1=true
    int reached_goal = 0;           //0=false, 1=true
    
    vector<double> angle_error;
    
    vector<double> x;                       //[length]
    vector<double> y;                       //[length]
    vector<double> theta;                   //[radians]
    vector<double> omega;                   //[radians/second]
    vector<double> dist_to_goal;            //[length]
    vector<double> u;
    int total_time_steps;
};

#endif /* Policy_hpp */
