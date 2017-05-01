//
//  Simulator.hpp
//  ME-791_SF_Project_Delta_project
//
//  Created by Scott S Forer on 4/6/17.
//  Copyright © 2017 Scott S Forer. All rights reserved.
//

#ifndef Simulator_hpp
#define Simulator_hpp
# define Pi           3.14159265358979323846  /* pi */

#include <stdio.h>

using namespace std;

class Simulator
{
    friend class EA;
    friend class Parameters;
    friend class Agent;
    friend class Policy;
    friend class neural_network;
    
public:
    Parameters* pP;
    
    vector<double> a;
    
    void Run_LR_3(Policy* pPo);
    void Run_MR_2(Policy* pPo);
    
    void Give_Weights_To_NN(Policy* pPo);
    void Initalize_Agent_Sensing(Policy* pPo);
    void Reset_Agent_Sensing(Policy* pPo);
    void Get_Angle_Error(Policy* pPo, vector<double> goal_mid, int ts);
    void Get_Distance_to_Goal(Policy* pPo, vector<double> goal_mid, int ts);
    void Calculate_New_X(Policy* pPo, int ts);
    void Calculate_New_Y(Policy* pPo, int ts);
    void Calculate_New_Theta(Policy* pPo, int ts);
    void Calculate_New_Controller(Policy* pPo, int ts, neural_network* pN);
    void Calculate_New_Omega(Policy* pPo, int ts, neural_network* pN);
    void Check_If_In_Goal_Box(Policy* pPo, int ts, vector<double> goal_box);
    void Check_Line_Intersection(Policy* pPo, int ts, vector<double> goal_line);
    void Check_If_Passed_Goal(Policy* pPo, int ts, vector<double> goal_line, vector<double> goal_box);
    void Output_State_Info(Policy* pPo, double t, int ts);
    void Check_If_In_Map(Policy* pPo, int ts);
    
    void Simulate(Policy* pPo, vector<double> goal_line, vector<double> goal_mid, vector<double> goal_box);
    
};


//-----------------------------------------------------------
//Runs LR_3
void Simulator::Run_LR_3(Policy* pPo)
{
    assert(pPo->in_map == 0);
}

//-----------------------------------------------------------
//Runs MR_2
void Simulator::Run_MR_2(Policy* pPo)
{
    assert(pPo->reached_goal == 1);
}

//-----------------------------------------------------------
//Gives the weights to the NN
void Simulator::Give_Weights_To_NN(Policy* pPo)
{
    vector<double> user_input;
    for (int w=0; w<pPo->weights.size(); w++)
    {
        user_input.push_back(pPo->weights.at(w));
    }
    //NN.set_weights(user_input, safe);
}


//-----------------------------------------------------------
//Initalizes the agents sensing data
void Simulator::Initalize_Agent_Sensing(Policy *pPo)
{
    pPo->in_map = 1;
    pPo->in_goal_box = 0;
    pPo->reached_goal = 0;
    pPo->total_time_steps = 0;
}


//-----------------------------------------------------------
//Resets the agents sensing data
void Simulator::Reset_Agent_Sensing(Policy *pPo)
{
    pPo->in_goal_box = 0;
}


//-----------------------------------------------------------
//Calculates the agents distance to the goal
void Simulator::Get_Distance_to_Goal(Policy* pPo , vector<double> goal_mid, int ts)
{
    double del_x = goal_mid.at(0) - pPo->x.at(ts);
    double del_y = goal_mid.at(1) - pPo->y.at(ts);
    pPo->dist_to_goal.push_back(sqrt(del_x*del_x+del_y*del_y));
}


//-----------------------------------------------------------
//Gets the angle error for each time step
void Simulator::Get_Angle_Error(Policy* pPo, vector<double> goal_mid, int ts)
{
    /*
    double x_1 = sin(pPo->theta.at(ts));
    double y_1 = cos(pPo->theta.at(ts));
    //double ph = x_1*x_1+y_1*y_1;
    assert (x_1*x_1+y_1*y_1 >= 1-0.0001 && x_1*x_1+y_1*y_1 <= 1+0.0001);
    
    double del_x = goal_mid.at(0) - pPo->x.at(ts);
    double del_y = goal_mid.at(1) - pPo->y.at(ts);
    double x_2 = del_x/pPo->dist_to_goal.at(ts);
    double y_2 = del_y/pPo->dist_to_goal.at(ts);
    assert (x_2*x_2+y_2*y_2>=1-0.001 && x_2*x_2+y_2*y_2<=1+0.001);
    double beta = acos(x_1*x_2+y_1*y_2);
    //cout << "BETA" << "\t" << beta*(180/Pi) << endl;
    //cout << "cp" << endl;
     */
    
    double alpha = 0;
    double beta = 0;
    double gamma = 0;
    double del_x = 0;
    double del_y = 0;
    double zeta = 0;
    del_x = abs(pPo->x.at(ts)-goal_mid.at(0));
    del_y= abs(pPo->y.at(ts)-goal_mid.at(1));
    gamma = atan(del_x/del_y);
    assert(gamma>=0 && gamma<=Pi/2);
    
    //above and to the left of goal
    if (pPo->x.at(ts)<goal_mid.at(0) && pPo->y.at(ts)>goal_mid.at(1) && pPo->theta.at(ts)==0)
    {
        beta = Pi-gamma;
    }
    if (pPo->x.at(ts)<goal_mid.at(0) && pPo->y.at(ts)>goal_mid.at(1) && pPo->theta.at(ts)>0 && pPo->theta.at(ts)<Pi/2)
    {
        beta = Pi-pPo->theta.at(ts)-gamma;
    }
    if (pPo->x.at(ts)<goal_mid.at(0) && pPo->y.at(ts)>goal_mid.at(1) && pPo->theta.at(ts)==Pi/2)
    {
        beta = Pi-pPo->theta.at(ts)-gamma;
    }
    if (pPo->x.at(ts)<goal_mid.at(0) && pPo->y.at(ts)>goal_mid.at(1) && pPo->theta.at(ts)>Pi/2 && pPo->theta.at(ts)<Pi)
    {
        alpha = (Pi/2)-gamma;
        zeta = alpha+(Pi/2);
        if (pPo->theta.at(ts)>zeta)
        {
            beta = zeta-pPo->theta.at(ts);
        }
        if (pPo->theta.at(ts)==zeta)
        {
            beta = 0;
        }
        if (pPo->theta.at(ts)<zeta)
        {
            beta = zeta-pPo->theta.at(ts);
        }
    }
    if (pPo->x.at(ts)<goal_mid.at(0) && pPo->y.at(ts)>goal_mid.at(1) && pPo->theta.at(ts)==Pi)
    {
        beta = -gamma;
    }
    if (pPo->x.at(ts)<goal_mid.at(0) && pPo->y.at(ts)>goal_mid.at(1) && pPo->theta.at(ts)>Pi && pPo->theta.at(ts)<3*Pi/2)
    {
        beta = -(pPo->theta.at(ts)-Pi+gamma);
    }
    if (pPo->x.at(ts)<goal_mid.at(0) && pPo->y.at(ts)>goal_mid.at(1) && pPo->theta.at(ts)==3*Pi/2)
    {
        beta = -(pPo->theta.at(ts)-Pi+gamma);
    }
    if (pPo->x.at(ts)<goal_mid.at(0) && pPo->y.at(ts)>goal_mid.at(1) && pPo->theta.at(ts)>3*Pi/2 && pPo->theta.at(ts)<2*Pi)
    {
        double beta_1 = 0;
        double beta_2 = 0;
        beta_1 = pPo->theta.at(ts)-Pi+gamma;
        beta_2 = (2*Pi)-beta_1;
        if (beta_1 < beta_2)
        {
            beta = -beta_1;
        }
        else
        {
            beta = beta_2;
        }
    }
    
    //directly above the goal
    if (pPo->x.at(ts)==goal_mid.at(0) && pPo->y.at(ts)>goal_mid.at(1) && pPo->theta.at(ts)==0)
    {
        beta = Pi;
    }
    if (pPo->x.at(ts)==goal_mid.at(0) && pPo->y.at(ts)>goal_mid.at(1) && pPo->theta.at(ts)>0 && pPo->theta.at(ts)<Pi/2)
    {
        beta = Pi-pPo->theta.at(ts);
    }
    if (pPo->x.at(ts)==goal_mid.at(0) && pPo->y.at(ts)>goal_mid.at(1) && pPo->theta.at(ts)==Pi/2)
    {
        beta = pPo->theta.at(ts);
    }
    if (pPo->x.at(ts)==goal_mid.at(0) && pPo->y.at(ts)>goal_mid.at(1) && pPo->theta.at(ts)>Pi/2 && pPo->theta.at(ts)<Pi)
    {
        beta = Pi-pPo->theta.at(ts);
    }
    if (pPo->x.at(ts)==goal_mid.at(0) && pPo->y.at(ts)>goal_mid.at(1) && pPo->theta.at(ts)==Pi)
    {
        beta = 0;
    }
    if (pPo->x.at(ts)==goal_mid.at(0) && pPo->y.at(ts)>goal_mid.at(1) && pPo->theta.at(ts)>Pi && pPo->theta.at(ts)<3*Pi/2)
    {
        beta = -(pPo->theta.at(ts)-Pi);
    }
    if (pPo->x.at(ts)==goal_mid.at(0) && pPo->y.at(ts)>goal_mid.at(1) && pPo->theta.at(ts)==3*Pi/2)
    {
        beta = -(pPo->theta.at(ts)-Pi);
    }
    if (pPo->x.at(ts)==goal_mid.at(0) && pPo->y.at(ts)>goal_mid.at(1) && pPo->theta.at(ts)>3*Pi/2 && pPo->theta.at(ts)<2*Pi)
    {
        beta = -(pPo->theta.at(ts)-Pi);
    }
    
    
    //above and to the right of the goal
    if (pPo->x.at(ts)>goal_mid.at(0) && pPo->y.at(ts)>goal_mid.at(1) && pPo->theta.at(ts)==0)
    {
        alpha = (Pi/2)-gamma;
        beta = -((Pi/2)+alpha);
    }
    if (pPo->x.at(ts)>goal_mid.at(0) && pPo->y.at(ts)>goal_mid.at(1) && pPo->theta.at(ts)>0 && pPo->theta.at(ts)<Pi/2)
    {
        double beta_1 = 0;
        double beta_2 = 0;
        beta_1 = Pi-pPo->theta.at(ts)+gamma;
        beta_2 = (2*Pi)-beta_1;
        if (beta_1 < beta_2)
        {
            beta = beta_1;
        }
        else
        {
            beta = -beta_2;
        }
    }
    if (pPo->x.at(ts)>goal_mid.at(0) && pPo->y.at(ts)>goal_mid.at(1) && pPo->theta.at(ts)==Pi/2)
    {
        beta = ((Pi/2)+gamma);
    }
    if (pPo->x.at(ts)>goal_mid.at(0) && pPo->y.at(ts)>goal_mid.at(1) && pPo->theta.at(ts)>Pi/2 && pPo->theta.at(ts)<Pi)
    {
        beta = (Pi-pPo->theta.at(ts)+gamma);
    }
    if (pPo->x.at(ts)>goal_mid.at(0) && pPo->y.at(ts)>goal_mid.at(1) && pPo->theta.at(ts)==Pi)
    {
        beta = gamma;
    }
    if (pPo->x.at(ts)>goal_mid.at(0) && pPo->y.at(ts)>goal_mid.at(1) && pPo->theta.at(ts)>Pi && pPo->theta.at(ts)<3*Pi/2)
    {
        zeta = Pi+gamma;
        if (pPo->theta.at(ts)<zeta)
        {
            beta = zeta-pPo->theta.at(ts);
        }
        if (pPo->theta.at(ts)==zeta)
        {
            beta = 0;
        }
        if (pPo->theta.at(ts)>zeta)
        {
            beta = zeta-pPo->theta.at(ts);
        }
    }
    if (pPo->x.at(ts)>goal_mid.at(0) && pPo->y.at(ts)>goal_mid.at(1) && pPo->theta.at(ts)==3*Pi/2)
    {
        beta = -(pPo->theta.at(ts)-Pi-gamma);
    }
    if (pPo->x.at(ts)>goal_mid.at(0) && pPo->y.at(ts)>goal_mid.at(1) && pPo->theta.at(ts)>3*Pi/2 && pPo->theta.at(ts)<2*Pi)
    {
        beta = -(pPo->theta.at(ts)-Pi-gamma);
    }
    
    //directly to the right of the goal
    if (pPo->x.at(ts)>goal_mid.at(0) && pPo->y.at(ts)==goal_mid.at(1) && pPo->theta.at(ts)==0)
    {
        beta = -(Pi/2);
    }
    if (pPo->x.at(ts)>goal_mid.at(0) && pPo->y.at(ts)==goal_mid.at(1) && pPo->theta.at(ts)>0 && pPo->theta.at(ts)<Pi/2)
    {
        beta = -(Pi/2+pPo->theta.at(ts));
    }
    if (pPo->x.at(ts)>goal_mid.at(0) && pPo->y.at(ts)==goal_mid.at(1) && pPo->theta.at(ts)==Pi/2)
    {
        beta = Pi;
    }
    if (pPo->x.at(ts)>goal_mid.at(0) && pPo->y.at(ts)==goal_mid.at(1) && pPo->theta.at(ts)>Pi/2 && pPo->theta.at(ts)<Pi)
    {
        beta = Pi-(pPo->theta.at(ts)-Pi/2);
    }
    if (pPo->x.at(ts)>goal_mid.at(0) && pPo->y.at(ts)==goal_mid.at(1) && pPo->theta.at(ts)==Pi)
    {
        beta = (Pi/2);
    }
    if (pPo->x.at(ts)>goal_mid.at(0) && pPo->y.at(ts)==goal_mid.at(1) && pPo->theta.at(ts)>Pi && pPo->theta.at(ts)<3*Pi/2)
    {
        beta = ((3*Pi/2)-pPo->theta.at(ts));
    }
    if (pPo->x.at(ts)>goal_mid.at(0) && pPo->y.at(ts)==goal_mid.at(1) && pPo->theta.at(ts)==3*Pi/2)
    {
        beta = 0;
    }
    if (pPo->x.at(ts)>goal_mid.at(0) && pPo->y.at(ts)==goal_mid.at(1) && pPo->theta.at(ts)>3*Pi/2 && pPo->theta.at(ts)<2*Pi)
    {
        beta = -(pPo->theta.at(ts)-(3*Pi/2));
    }
    
    //below and to the right of the goal
    if (pPo->x.at(ts)>goal_mid.at(0) && pPo->y.at(ts)<goal_mid.at(1) && pPo->theta.at(ts)==0)
    {
        beta = -gamma;
    }
    if (pPo->x.at(ts)>goal_mid.at(0) && pPo->y.at(ts)<goal_mid.at(1) && pPo->theta.at(ts)>0 && pPo->theta.at(ts)<Pi/2)
    {
        beta = -(pPo->theta.at(ts)+gamma);
    }
    if (pPo->x.at(ts)>goal_mid.at(0) && pPo->y.at(ts)<goal_mid.at(1) && pPo->theta.at(ts)==Pi/2)
    {
        beta = -(pPo->theta.at(ts)+gamma);
    }
    if (pPo->x.at(ts)>goal_mid.at(0) && pPo->y.at(ts)<goal_mid.at(1) && pPo->theta.at(ts)>Pi/2 && pPo->theta.at(ts)<Pi)
    {
        double beta_1 = 0;
        double beta_2 = 0;
        beta_1 = pPo->theta.at(ts)+gamma;
        beta_2 = (2*Pi)-beta_1;
        if (beta_1 < beta_2)
        {
            beta = -beta_1;
        }
        else
        {
            beta = beta_2;
        }
    }
    if (pPo->x.at(ts)>goal_mid.at(0) && pPo->y.at(ts)<goal_mid.at(1) && pPo->theta.at(ts)==Pi)
    {
        beta = (Pi-gamma);
    }
    if (pPo->x.at(ts)>goal_mid.at(0) && pPo->y.at(ts)<goal_mid.at(1) && pPo->theta.at(ts)>Pi && pPo->theta.at(ts)<3*Pi/2)
    {
        alpha = (Pi/2)-gamma;
        beta = ((3*Pi/2)-pPo->theta.at(ts)+alpha);
    }
    if (pPo->x.at(ts)>goal_mid.at(0) && pPo->y.at(ts)<goal_mid.at(1) && pPo->theta.at(ts)==3*Pi/2)
    {
        alpha = (Pi/2)-gamma;
        beta = alpha;
    }
    if (pPo->x.at(ts)>goal_mid.at(0) && pPo->y.at(ts)<goal_mid.at(1) && pPo->theta.at(ts)>3*Pi/2 && pPo->theta.at(ts)<2*Pi)
    {
        alpha = (Pi/2)-gamma;
        zeta = (3*Pi/2)+alpha;
        if (pPo->theta.at(ts)<zeta)
        {
            beta = zeta-pPo->theta.at(ts);
        }
        if (pPo->theta.at(ts)==zeta)
        {
            beta = 0;
        }
        if (pPo->theta.at(ts)>zeta)
        {
            beta = zeta-pPo->theta.at(ts);
        }
    }
    
    //directly below the goal
    if (pPo->x.at(ts)==goal_mid.at(0) && pPo->y.at(ts)<goal_mid.at(1) && pPo->theta.at(ts)==0)
    {
        beta = 0;
    }
    if (pPo->x.at(ts)==goal_mid.at(0) && pPo->y.at(ts)<goal_mid.at(1) && pPo->theta.at(ts)>0 && pPo->theta.at(ts)<Pi/2)
    {
        beta = -(pPo->theta.at(ts));
    }
    if (pPo->x.at(ts)==goal_mid.at(0) && pPo->y.at(ts)<goal_mid.at(1) && pPo->theta.at(ts)==Pi/2)
    {
        beta = -(pPo->theta.at(ts));
    }
    if (pPo->x.at(ts)==goal_mid.at(0) && pPo->y.at(ts)<goal_mid.at(1) && pPo->theta.at(ts)>Pi/2 && pPo->theta.at(ts)<Pi)
    {
        beta = -(pPo->theta.at(ts));
    }
    if (pPo->x.at(ts)==goal_mid.at(0) && pPo->y.at(ts)<goal_mid.at(1) && pPo->theta.at(ts)==Pi)
    {
        beta = pPo->theta.at(ts);
    }
    if (pPo->x.at(ts)==goal_mid.at(0) && pPo->y.at(ts)<goal_mid.at(1) && pPo->theta.at(ts)>Pi && pPo->theta.at(ts)<3*Pi/2)
    {
        beta = ((2*Pi)-pPo->theta.at(ts));
    }
    if (pPo->x.at(ts)==goal_mid.at(0) && pPo->y.at(ts)<goal_mid.at(1) && pPo->theta.at(ts)==3*Pi/2)
    {
        beta = ((2*Pi)-pPo->theta.at(ts));
    }
    if (pPo->x.at(ts)==goal_mid.at(0) && pPo->y.at(ts)<goal_mid.at(1) && pPo->theta.at(ts)>3*Pi/2 && pPo->theta.at(ts)<2*Pi)
    {
        beta = ((2*Pi)-pPo->theta.at(ts));
    }
    
    //below and to the left of the goal
    if (pPo->x.at(ts)<goal_mid.at(0) && pPo->y.at(ts)<goal_mid.at(1) && pPo->theta.at(ts)==0)
    {
        beta = gamma;
    }
    if (pPo->x.at(ts)<goal_mid.at(0) && pPo->y.at(ts)<goal_mid.at(1) && pPo->theta.at(ts)>0 && pPo->theta.at(ts)<Pi/2)
    {
        zeta = gamma;
        if (pPo->theta.at(ts)<zeta)
        {
            beta = zeta-pPo->theta.at(ts);
        }
        if (pPo->theta.at(ts)==zeta)
        {
            beta = 0;
        }
        if (pPo->theta.at(ts)>zeta)
        {
            beta = zeta-pPo->theta.at(ts);
        }
    }
    if (pPo->x.at(ts)<goal_mid.at(0) && pPo->y.at(ts)<goal_mid.at(1) && pPo->theta.at(ts)==Pi/2)
    {
        beta = -(pPo->theta.at(ts)-gamma);
    }
    if (pPo->x.at(ts)<goal_mid.at(0) && pPo->y.at(ts)<goal_mid.at(1) && pPo->theta.at(ts)>Pi/2 && pPo->theta.at(ts)<Pi)
    {
        beta = -(pPo->theta.at(ts)-gamma);
    }
    if (pPo->x.at(ts)<goal_mid.at(0) && pPo->y.at(ts)<goal_mid.at(1) && pPo->theta.at(ts)==Pi)
    {
        beta = -(pPo->theta.at(ts)-gamma);
    }
    if (pPo->x.at(ts)<goal_mid.at(0) && pPo->y.at(ts)<goal_mid.at(1) && pPo->theta.at(ts)>Pi && pPo->theta.at(ts)<3*Pi/2)
    {
        double beta_1 = 0;
        double beta_2 = 0;
        beta_1 = pPo->theta.at(ts)-gamma;
        beta_2 = (2*Pi)-beta_1;
        if (beta_1 < beta_2)
        {
            beta = -beta_1;
        }
        else
        {
            beta = beta_2;
        }
    }
    if (pPo->x.at(ts)<goal_mid.at(0) && pPo->y.at(ts)<goal_mid.at(1) && pPo->theta.at(ts)==3*Pi/2)
    {
        beta = (Pi/2)+gamma;
    }
    if (pPo->x.at(ts)<goal_mid.at(0) && pPo->y.at(ts)<goal_mid.at(1) && pPo->theta.at(ts)>3*Pi/2 && pPo->theta.at(ts)<2*Pi)
    {
        beta = (2*Pi)-pPo->theta.at(ts)+gamma;
    }
    
    //directly to the left of the goal
    if (pPo->x.at(ts)<goal_mid.at(0) && pPo->y.at(ts)==goal_mid.at(1) && pPo->theta.at(ts)==0)
    {
        beta = Pi/2;
    }
    if (pPo->x.at(ts)<goal_mid.at(0) && pPo->y.at(ts)==goal_mid.at(1) && pPo->theta.at(ts)>0 && pPo->theta.at(ts)<Pi/2)
    {
        beta = (Pi/2)-pPo->theta.at(ts);
    }
    if (pPo->x.at(ts)<goal_mid.at(0) && pPo->y.at(ts)==goal_mid.at(1) && pPo->theta.at(ts)==Pi/2)
    {
        beta = 0;
    }
    if (pPo->x.at(ts)<goal_mid.at(0) && pPo->y.at(ts)==goal_mid.at(1) && pPo->theta.at(ts)>Pi/2 && pPo->theta.at(ts)<Pi)
    {
        beta = -(pPo->theta.at(ts)-(Pi/2));
    }
    if (pPo->x.at(ts)<goal_mid.at(0) && pPo->y.at(ts)==goal_mid.at(1) && pPo->theta.at(ts)==Pi)
    {
        beta = -(Pi/2);
    }
    if (pPo->x.at(ts)<goal_mid.at(0) && pPo->y.at(ts)==goal_mid.at(1) && pPo->theta.at(ts)>Pi && pPo->theta.at(ts)<3*Pi/2)
    {
        beta = -(pPo->theta.at(ts)-(2*Pi));
    }
    if (pPo->x.at(ts)<goal_mid.at(0) && pPo->y.at(ts)==goal_mid.at(1) && pPo->theta.at(ts)==3*Pi/2)
    {
        beta = Pi;
    }
    if (pPo->x.at(ts)<goal_mid.at(0) && pPo->y.at(ts)==goal_mid.at(1) && pPo->theta.at(ts)>3*Pi/2 && pPo->theta.at(ts)<2*Pi)
    {
        beta = (2*Pi)-pPo->theta.at(ts)+(Pi/2);
    }
    assert(beta>=-Pi && beta<=Pi);
    double check_gamma = gamma*(180/Pi);
    double check_beta = beta*(180/Pi);
    double check_3 = beta*(180/Pi)+gamma*(180/Pi);
    double check_theta = pPo->theta.at(ts)*(180/Pi);
    double check_zeta = zeta*(180/Pi);
    pPo->angle_error.push_back(beta);
}


//-----------------------------------------------------------
//Calculates the new x position
void Simulator::Calculate_New_X(Policy* pPo, int ts)
{
    //satisfies LR_8
    pPo->x.push_back(pPo->x.at(ts-1) + pP->v*sin(pPo->theta.at(ts-1))*pP->dt);
}


//-----------------------------------------------------------
//Calculates the new y position
void Simulator::Calculate_New_Y(Policy* pPo, int ts)
{
    pPo->y.push_back(pPo->y.at(ts-1) + pP->v*cos(pPo->theta.at(ts-1))*pP->dt);
}


//-----------------------------------------------------------
//Calculates the new theta
void Simulator::Calculate_New_Theta(Policy* pPo, int ts)
{
    double theta = pPo->theta.at(ts-1) + pPo->omega.at(ts-1)*pP->dt;
    int n = 0;
    while (theta>2*Pi)
    {
        theta = theta-(2*Pi);
        n += 1;
    }
    if (theta<0)
    {
        theta = theta*-1;
        while (theta>2*Pi)
        {
            theta = theta-(2*Pi);
            n += 1;
        }
    }
    assert(theta>=0 && theta<=2*Pi);
    pPo->theta.push_back(theta);
}


//-----------------------------------------------------------
//Calculates the new controller
void Simulator::Calculate_New_Controller(Policy* pPo, int ts, neural_network* pN)
{
    
    //a.push_back(pPo->x.at(ts-1));
    //a.push_back(pPo->y.at(ts-1));
    //a.push_back(pPo->theta.at(ts-1));
    //a.push_back(pPo->omega.at(ts-1));
    //satisfies MR_3
    a.push_back(pPo->angle_error.at(ts-1));
    pN->set_vector_input(a);
    vector<double> user_input;
    for (int w=0; w<pPo->weights.size(); w++)
    {
        user_input.push_back(pPo->weights.at(w));
    }
    pN->set_weights(user_input, true);
    pN->execute();
    double output = pN->get_output(0);
    assert(output>=pP->u_min && output<=pP->u_max);
    //satisfies LR_7
    pPo->u.push_back(output);
    a.clear();
    
    //double u = 0;
    //pPo->u.push_back(u);
}


//-----------------------------------------------------------
//Calculates the new omega
void Simulator::Calculate_New_Omega(Policy* pPo, int ts, neural_network* pN)
{
    //whats the fastes that omega can be?
    Calculate_New_Controller(pPo, ts, pN);
    double a = pPo->omega.at(ts-1) + ((pPo->u.at(ts-1) - pPo->omega.at(ts-1))*(pP->dt/pP->T));
    pPo->omega.push_back(a);
    
    //cout << "Omega" << "\t" << pPo->omega.at(ts-1)*(180/Pi) << "\t" << "U" << "\t" << pPo->u.at(ts-1)*(180/Pi) << endl;
    //cout << "New Omega" << "\t" << a*(180/Pi) << endl;
    //cout << "Acceptable Range" << "\t" << pP->u_min*(180/Pi) << "\t" << pP->u_max*(180/Pi) << endl;
    assert(a>=pP->u_min && a<=pP->u_max);
}


//-----------------------------------------------------------
//Checks if the agent is within traveling distance of the goal
void Simulator::Check_If_In_Goal_Box(Policy* pPo, int ts, vector<double> goal_box)
{
    if (ts > 0)
    {
        if (pPo->x.at(ts-1)>=goal_box.at(0) && pPo->x.at(ts-1)<=goal_box.at(2))
        {
            if (pPo->y.at(ts-1)<=goal_box.at(1) && pPo->y.at(ts-1)>=goal_box.at(5))
            {
                pPo->in_goal_box = 1;
            }
        }
    }
}


//-----------------------------------------------------------
//Checks if the agent has intersected the goal
void Simulator::Check_Line_Intersection(Policy *pPo, int ts, vector<double> goal_line)
{
    float ax = pPo->x.at(ts-1);
    float ay = pPo->y.at(ts-1);
    float bx = pPo->x.at(ts);
    float by = pPo->y.at(ts);
    float cx = goal_line.at(0);
    float cy = goal_line.at(1);
    float dx = goal_line.at(2);
    float dy = goal_line.at(3);
    float den = ((dy-cy)*(bx-ax)-(dx-cx)*(by-ay));
    float num1 = ((dx - cx)*(ay-cy) - (dy- cy)*(ax-cx));
    float num2 = ((bx-ax)*(ay-cy)-(by-ay)*(ax-cx));
    float u1 = num1/den;
    float u2 = num2/den;
    //cout << u1 << ":" << u2 << std::endl;
    if (den == 0 && num1  == 0 && num2 == 0)
    {
        /* The two lines are coincidents */
        pPo->reached_goal = 1;
    }
    if (den == 0)
    {
        /* The two lines are parallel */
    }
    if (u1 <0 || u1 > 1 || u2 < 0 || u2 > 1)
    {
        /* Lines do not collide */
    }
    else
    {
        /* Lines DO collide */
        pPo->reached_goal = 1;
    }
}


//-----------------------------------------------------------
//Checks if the agent has passed the goal line
void Simulator::Check_If_Passed_Goal(Policy* pPo, int ts, vector<double> goal_line, vector<double> goal_box)
{
    Check_If_In_Goal_Box(pPo, ts, goal_box);
    if (pPo->in_goal_box == 1)
    {
        Check_Line_Intersection (pPo, ts, goal_line);
    }
}


//-----------------------------------------------------------
//Checks that the agent is still in the map space
void Simulator::Check_If_In_Map(Policy* pPo, int ts)
{
    if (pPo->x.at(ts)<0 || pPo->x.at(ts)>pP->x_max || pPo->y.at(ts)<0 || pPo->y.at(ts)>pP->y_max)
    {
        pPo->in_map = 0;
    }
}


//-----------------------------------------------------------
//Outputs all the state information for that time step
void Simulator::Output_State_Info(Policy* pPo, double t, int ts)
{
    cout << "--------------------------------------" << endl;
    cout << "Current Time" << "\t" << t << "\t" << "Time Step" << "\t" << ts <<endl;
    cout << "X" << "\t" << "Y" << "\t" << "Theta [deg]" << "\t" << "Omega [deg/s]" << "\t" << "U [deg/s]" << "\t" << "Angle Error [deg]" << endl;
    cout << pPo->x.at(ts) << "\t" << pPo->y.at(ts) << "\t" << pPo->theta.at(ts)*(180/Pi) << "\t" << pPo->omega.at(ts)*(180/Pi) << "\t" << pPo->u.at(ts)*(180/Pi) << "\t" << pPo->angle_error.at(ts)*(180/Pi) << endl;
    cout << endl;
}


//-----------------------------------------------------------
//Runs the entire simulation process
void Simulator::Simulate(Policy* pPo, vector<double> goal_line, vector<double> goal_mid, vector<double> goal_box)
{
    //satisfies LR_4
    neural_network NN;
    neural_network* pN;
    pN = &NN;
    NN.setup(pP->num_inputs, pP->num_hidden_nodes, pP->num_outputs);
    //satisfies LR_6
    NN.set_in_min_max(-Pi, Pi);
    NN.set_out_min_max(pP->u_min, pP->u_max);
    //Give_Weights_To_NN(pPo);
    double t=0;
    int ts=0;
    pPo->in_map = 0;
    while (t<pP->time_max)
    {
        if (t==0)
        {
            Initalize_Agent_Sensing(pPo);
            Get_Distance_to_Goal(pPo, goal_mid, ts);
            Get_Angle_Error(pPo, goal_mid, ts);
            //Output_State_Info(pPo, t, ts);
            Check_If_Passed_Goal(pPo, ts, goal_line,goal_box);
            Check_If_In_Map(pPo, ts);
        }
        if (t>0)
        {
            Reset_Agent_Sensing(pPo);
            Calculate_New_X(pPo, ts);
            Calculate_New_Y(pPo, ts);
            Calculate_New_Theta(pPo, ts);
            Calculate_New_Omega(pPo, ts, pN);
            Get_Distance_to_Goal(pPo, goal_mid, ts);
            Get_Angle_Error(pPo, goal_mid, ts);
            Check_If_Passed_Goal(pPo, ts, goal_line,goal_box);
            if (pPo->reached_goal == 1)
            {
                //Output_State_Info(pPo, t, ts);
                pPo->angle_error.erase(pPo->angle_error.begin() + pPo->angle_error.size()-1);
                pPo->in_map = 1;
                pPo->total_time_steps = ts;
                Run_MR_2(pPo);
                //cout << "REACHED GOAL" << endl;
                break;
            }
            Check_If_In_Map(pPo, ts);
            if (pPo->in_map == 0)
            {
                //Output_State_Info(pPo, t, ts);
                pPo->total_time_steps = ts;
                pPo->angle_error.erase(pPo->angle_error.begin() + pPo->angle_error.size()-1);
                Run_LR_3(pPo);
                //cout << "OUT OF MAP SPACE" << endl;
                break;
            }
            //Output_State_Info(pPo, t, ts);
        }
        ts += 1;
        t += pP->dt;
    }
    if (pPo->reached_goal==0 && pPo->in_map==1)
    {
        pPo->total_time_steps = ts;
        //cout << "OUT OF TIME" << endl;
    }
}

#endif /* Simulator_hpp */
