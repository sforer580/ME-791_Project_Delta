//
//  Parameters.hpp
//  ME-791_SF_Project_Delta_project
//
//  Created by Scott S Forer on 4/6/17.
//  Copyright © 2017 Scott S Forer. All rights reserved.
//

#ifndef Parameters_hpp
#define Parameters_hpp
# define Pi           3.14159265358979323846  /* pi */

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
    int num_inputs = 1;
    int num_hidden_nodes = 5;
    int num_outputs = 1;
    
    //EA Parameters
    int pop_size = 100;
    int num_weights;
    int gen_max = 300;
    double mutation_rate = 0.5;
    double range = 0.05;
    int HR_1 = 0;                       //0=off, 1=on
    int HR_3 = 0;                       //0=off, 1=on
    int HR_4 = 0;                       //0=off, 1=on
    
    //Simulator Parameters
    double x_max = 100;             //[length]
    double y_max = 100;             //[length]
    double goal_length = 10;        //[length]
    double time_max = 200;          //[secodns]
    double time_step = 0.1;         //[secodns]
    double v = 3.0;                 //[length/second]
    double dt = 0.2;                //[secodns]
    double T = 5.0;                 //[seconds]
    double u_max = 15*(Pi/180);     //[radians/second]
    double u_min = -15*(Pi/180);    //[radians/second]
    
};

#endif /* Parameters_hpp */
