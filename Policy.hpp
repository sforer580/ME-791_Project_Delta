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
    double current_x;                           //[length]
    double current_y;                           //[length]
    double current_theta;                       //[radians]
    double current_omega;                       //[radian/second]
    double current_dist_to_goal;                //[length]
    double current_u;
    double new_x;                               //[length]
    double new_y;                               //[length]
    double new_theta;                           //[radians]
    double new_omega;                           //[radians/second]
    double new_dist_to_goal;                    //[length]
    double new_u;
    
    int in_map = 0;
    
    vector<double> angle_error;
    
    vector<double> path;                        //[length]
    vector<double> all_theta;                   //[radians]
    vector<double> all_omega;                   //[radians/second]
    vector<double> all_dist_to_goal;            //[length]
    vector<double> all_u;
};

#endif /* Policy_hpp */
