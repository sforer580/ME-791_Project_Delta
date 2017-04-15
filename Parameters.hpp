//
//  Parameters.hpp
//  ME-791_SF_Project_Delta_project
//
//  Created by Scott S Forer on 4/6/17.
//  Copyright © 2017 Scott S Forer. All rights reserved.
//

#ifndef Parameters_hpp
#define Parameters_hpp

#include <stdio.h>

using namespace std;

class Parameters
{
    friend class EA;
    friend class Simulator;
    friend class Agent;
    friend class Policy;
    friend class neural_network;
    
public:
    //ANN Parameters
    int num_inputs = 3;
    int num_hidden_nodes = 3;
    int num_outputs = 1;
    
    //EA Parameters
    int pop_size = 1;
    int num_weights;;
    int gen_max = 2;
    double mutation_rate = 0.5;
    double range = 0.5;
    
    //Simulator Parameters
    double x_max = 100;             //[length]
    double y_max = 100;             //[length]
    double time_max = 50;           //[secodns]
    double time_step = 0.1;         //[secodns]
    double v = 3.0;                 //[length/second]
    double dt = 0.2;                //[secodns]
    double T = 5.0;                 //[seconds]
    double u_max = 15;              //[degree/second]
    double u_min = -15;             //[degree/second]
    
};

#endif /* Parameters_hpp */
