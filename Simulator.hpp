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
    
    vector<double> goal_1;
    vector<double> goal_2;
    double goal_length;                 //[length]
    double goal_angle;                  //[radians]
    
    void Give_Weights_To_NN(Policy* pPo);
    void Initalize_Goal();
    void Get_Goal_Angle();
    void Agent_Set_Up(Policy* pPo);
    void Get_Angle_Error(Policy* pPo, vector<double> goal_mid);
    void Get_Distance_to_Goal(Policy* pPo);
    void Calculate_New_X(Policy* pPo);
    void Calculate_New_Y(Policy* pPo);
    void Calculate_New_Theta(Policy* pPo);
    void Calculate_New_Controller(Policy* pPo);
    void Calculate_New_Omega(Policy* pPo);
    void Output_State_Info(Policy* pPo, double t);
    void Check_If_In_Map(Policy* pPo);
    
    void Simulate(Policy* pPo, vector<double> goal_mid);
    
};


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
//Sets the agent in its initial spot
void Simulator::Agent_Set_Up(Policy* pPo)
{
    pPo->new_x = pPo->current_x;
    pPo->new_y = pPo->current_y;
    pPo->new_theta = pPo->current_theta;
    pPo->new_omega = pPo->current_omega;
    pPo->new_u = pPo->current_u;
    
    pPo->path.push_back(pPo->new_x);
    pPo->path.push_back(pPo->new_y);
    pPo->all_theta.push_back(pPo->new_theta);
    pPo->all_omega.push_back(pPo->new_omega);
    pPo->all_u.push_back(pPo->new_u);
}


//-----------------------------------------------------------
//Gets the angle error for each time step
void Simulator::Get_Angle_Error(Policy* pPo, vector<double> goal_mid)
{
    double alpha = 0;
    double beta = 0;
    double gamma = 0;
    double del_x = 0;
    double del_y = 0;
    double zeta = 0;
    
    //above and to the left of goal
    if (pPo->new_x<goal_mid.at(0) && pPo->new_y>goal_mid.at(1) && pPo->new_theta==0)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        beta = Pi-gamma;
    }
    if (pPo->new_x<goal_mid.at(0) && pPo->new_y>goal_mid.at(1) && pPo->new_theta>0 && pPo->new_theta<Pi/2)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        beta = Pi-pPo->new_theta-gamma;
    }
    if (pPo->new_x<goal_mid.at(0) && pPo->new_y>goal_mid.at(1) && pPo->new_theta==Pi/2)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        beta = Pi-pPo->new_theta-gamma;
    }
    if (pPo->new_x<goal_mid.at(0) && pPo->new_y>goal_mid.at(1) && pPo->new_theta>Pi/2 && pPo->new_theta<Pi)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        alpha = (Pi/2)-gamma;
        zeta = alpha+(Pi/2);
        if (pPo->new_theta>zeta)
        {
            beta = pPo->new_theta-zeta;
        }
        if (pPo->new_theta==zeta)
        {
            beta = 0;
        }
        if (pPo->new_theta<zeta)
        {
            beta = zeta-pPo->new_theta;
        }
    }
    if (pPo->new_x<goal_mid.at(0) && pPo->new_y>goal_mid.at(1) && pPo->new_theta==Pi)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        beta = gamma;
    }
    if (pPo->new_x<goal_mid.at(0) && pPo->new_y>goal_mid.at(1) && pPo->new_theta>Pi && pPo->new_theta<3*Pi/2)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        beta = pPo->new_theta-Pi+gamma;
    }
    if (pPo->new_x<goal_mid.at(0) && pPo->new_y>goal_mid.at(1) && pPo->new_theta==3*Pi/2)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        beta = pPo->new_theta-Pi+gamma;
    }
    if (pPo->new_x<goal_mid.at(0) && pPo->new_y>goal_mid.at(1) && pPo->new_theta>3*Pi/2 && pPo->new_theta<2*Pi)
    {
        double beta_1 = 0;
        double beta_2 = 0;
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        beta_1 = pPo->new_theta-Pi+gamma;
        beta_2 = (2*Pi)-beta_1;
        if (beta_1 < beta_2)
        {
            beta = beta_1;
        }
        else
        {
            beta = beta_2;
        }
    }
    
    //directly above the goal
    if (pPo->new_x==goal_mid.at(0) && pPo->new_y>goal_mid.at(1) && pPo->new_theta==0)
    {
        beta = Pi;
    }
    if (pPo->new_x==goal_mid.at(0) && pPo->new_y>goal_mid.at(1) && pPo->new_theta>0 && pPo->new_theta<Pi/2)
    {
        beta = Pi-pPo->new_theta;
    }
    if (pPo->new_x==goal_mid.at(0) && pPo->new_y>goal_mid.at(1) && pPo->new_theta==Pi/2)
    {
        beta = pPo->new_theta;
    }
    if (pPo->new_x==goal_mid.at(0) && pPo->new_y>goal_mid.at(1) && pPo->new_theta>Pi/2 && pPo->new_theta<Pi)
    {
        beta = Pi-pPo->new_theta;
    }
    if (pPo->new_x==goal_mid.at(0) && pPo->new_y>goal_mid.at(1) && pPo->new_theta==Pi)
    {
        beta = 0;
    }
    if (pPo->new_x==goal_mid.at(0) && pPo->new_y>goal_mid.at(1) && pPo->new_theta>Pi && pPo->new_theta<3*Pi/2)
    {
        beta = pPo->new_theta-Pi;
    }
    if (pPo->new_x==goal_mid.at(0) && pPo->new_y>goal_mid.at(1) && pPo->new_theta==3*Pi/2)
    {
        beta = pPo->new_theta-Pi;
    }
    if (pPo->new_x==goal_mid.at(0) && pPo->new_y>goal_mid.at(1) && pPo->new_theta>3*Pi/2 && pPo->new_theta<2*Pi)
    {
        beta = pPo->new_theta-Pi;
    }
    
    
    //above and to the right of the goal
    if (pPo->new_x>goal_mid.at(0) && pPo->new_y>goal_mid.at(1) && pPo->new_theta==0)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        alpha = (Pi/2)-gamma;
        beta = (Pi/2)+alpha;
    }
    if (pPo->new_x>goal_mid.at(0) && pPo->new_y>goal_mid.at(1) && pPo->new_theta>0 && pPo->new_theta<Pi/2)
    {
        double beta_1 = 0;
        double beta_2 = 0;
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        beta_1 = Pi-pPo->new_theta+gamma;
        beta_2 = (2*Pi)-beta_1;
        if (beta_1 < beta_2)
        {
            beta = beta_1;
        }
        else
        {
            beta = beta_2;
        }
    }
    if (pPo->new_x>goal_mid.at(0) && pPo->new_y>goal_mid.at(1) && pPo->new_theta==Pi/2)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        beta = (Pi/2)+gamma;
    }
    if (pPo->new_x>goal_mid.at(0) && pPo->new_y>goal_mid.at(1) && pPo->new_theta>Pi/2 && pPo->new_theta<Pi)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        beta = Pi-pPo->new_theta+gamma;
    }
    if (pPo->new_x>goal_mid.at(0) && pPo->new_y>goal_mid.at(1) && pPo->new_theta==Pi)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        beta = gamma;
    }
    if (pPo->new_x>goal_mid.at(0) && pPo->new_y>goal_mid.at(1) && pPo->new_theta>Pi && pPo->new_theta<3*Pi/2)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        zeta = Pi+gamma;
        if (pPo->new_theta<zeta)
        {
            beta = zeta-pPo->new_theta;
        }
        if (pPo->new_theta==zeta)
        {
            beta = 0;
        }
        if (pPo->new_theta>zeta)
        {
            beta = pPo->new_theta-zeta;
        }
    }
    if (pPo->new_x>goal_mid.at(0) && pPo->new_y>goal_mid.at(1) && pPo->new_theta==3*Pi/2)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        beta = pPo->new_theta-Pi-gamma;
    }
    if (pPo->new_x>goal_mid.at(0) && pPo->new_y>goal_mid.at(1) && pPo->new_theta>3*Pi/2 && pPo->new_theta<2*Pi)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        beta = pPo->new_theta-Pi-gamma;
    }
    
    //directly to the right of the goal
    if (pPo->new_x>goal_mid.at(0) && pPo->new_y==goal_mid.at(1) && pPo->new_theta==0)
    {
        beta = Pi/2;
    }
    if (pPo->new_x>goal_mid.at(0) && pPo->new_y==goal_mid.at(1) && pPo->new_theta>0 && pPo->new_theta<Pi/2)
    {
        beta = Pi/2+pPo->new_theta;
    }
    if (pPo->new_x>goal_mid.at(0) && pPo->new_y==goal_mid.at(1) && pPo->new_theta==Pi/2)
    {
        beta = Pi;
    }
    if (pPo->new_x>goal_mid.at(0) && pPo->new_y==goal_mid.at(1) && pPo->new_theta>Pi/2 && pPo->new_theta<Pi)
    {
        beta = Pi/2+pPo->new_theta;
    }
    if (pPo->new_x>goal_mid.at(0) && pPo->new_y==goal_mid.at(1) && pPo->new_theta==Pi)
    {
        beta = Pi/2;
    }
    if (pPo->new_x>goal_mid.at(0) && pPo->new_y==goal_mid.at(1) && pPo->new_theta>Pi && pPo->new_theta<3*Pi/2)
    {
        beta = (3*Pi/2)-pPo->new_theta;
    }
    if (pPo->new_x>goal_mid.at(0) && pPo->new_y==goal_mid.at(1) && pPo->new_theta==3*Pi/2)
    {
        beta = 0;
    }
    if (pPo->new_x>goal_mid.at(0) && pPo->new_y==goal_mid.at(1) && pPo->new_theta>3*Pi/2 && pPo->new_theta<2*Pi)
    {
        beta = (2*Pi)-pPo->new_theta;
    }
    
    //below and to the right of the goal
    if (pPo->new_x>goal_mid.at(0) && pPo->new_y<goal_mid.at(1) && pPo->new_theta==0)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        beta = gamma;
    }
    if (pPo->new_x>goal_mid.at(0) && pPo->new_y<goal_mid.at(1) && pPo->new_theta>0 && pPo->new_theta<Pi/2)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        beta = pPo->new_theta+gamma;
    }
    if (pPo->new_x>goal_mid.at(0) && pPo->new_y<goal_mid.at(1) && pPo->new_theta==Pi/2)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        beta = pPo->new_theta+gamma;
    }
    if (pPo->new_x>goal_mid.at(0) && pPo->new_y<goal_mid.at(1) && pPo->new_theta>Pi/2 && pPo->new_theta<Pi)
    {
        double beta_1 = 0;
        double beta_2 = 0;
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        beta_1 = pPo->new_theta+gamma;
        beta_2 = (2*Pi)-beta_1;
        if (beta_1 < beta_2)
        {
            beta = beta_1;
        }
        else
        {
            beta = beta_2;
        }
    }
    if (pPo->new_x>goal_mid.at(0) && pPo->new_y<goal_mid.at(1) && pPo->new_theta==Pi)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        beta = Pi-gamma;
    }
    if (pPo->new_x>goal_mid.at(0) && pPo->new_y<goal_mid.at(1) && pPo->new_theta>Pi && pPo->new_theta<3*Pi/2)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        alpha = (Pi/2)-gamma;
        beta = (3*Pi/2)-pPo->new_theta+alpha;
    }
    if (pPo->new_x>goal_mid.at(0) && pPo->new_y<goal_mid.at(1) && pPo->new_theta==3*Pi/2)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        alpha = (Pi/2)-gamma;
        beta = alpha;
    }
    if (pPo->new_x>goal_mid.at(0) && pPo->new_y<goal_mid.at(1) && pPo->new_theta>3*Pi/2 && pPo->new_theta<2*Pi)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        alpha = (Pi/2)-gamma;
        zeta = (3*Pi/2)+alpha;
        if (pPo->new_theta<zeta)
        {
            beta = zeta-pPo->new_theta;
        }
        if (pPo->new_theta==zeta)
        {
            beta = 0;
        }
        if (pPo->new_theta>zeta)
        {
            beta = pPo->new_theta-zeta;
        }
    }
    
    //directly below the goal
    if (pPo->new_x==goal_mid.at(0) && pPo->new_y<goal_mid.at(1) && pPo->new_theta==0)
    {
        beta = 0;
    }
    if (pPo->new_x==goal_mid.at(0) && pPo->new_y<goal_mid.at(1) && pPo->new_theta>0 && pPo->new_theta<Pi/2)
    {
        beta = pPo->new_theta;
    }
    if (pPo->new_x==goal_mid.at(0) && pPo->new_y<goal_mid.at(1) && pPo->new_theta==Pi/2)
    {
        beta = pPo->new_theta;
    }
    if (pPo->new_x==goal_mid.at(0) && pPo->new_y<goal_mid.at(1) && pPo->new_theta>Pi/2 && pPo->new_theta<Pi)
    {
        beta = pPo->new_theta;
    }
    if (pPo->new_x==goal_mid.at(0) && pPo->new_y<goal_mid.at(1) && pPo->new_theta==Pi)
    {
        beta = pPo->new_theta;
    }
    if (pPo->new_x==goal_mid.at(0) && pPo->new_y<goal_mid.at(1) && pPo->new_theta>Pi && pPo->new_theta<3*Pi/2)
    {
        beta = (2*Pi)-pPo->new_theta;
    }
    if (pPo->new_x==goal_mid.at(0) && pPo->new_y<goal_mid.at(1) && pPo->new_theta==3*Pi/2)
    {
        beta = (2*Pi)-pPo->new_theta;
    }
    if (pPo->new_x==goal_mid.at(0) && pPo->new_y<goal_mid.at(1) && pPo->new_theta>3*Pi/2 && pPo->new_theta<2*Pi)
    {
        beta = (2*Pi)-pPo->new_theta;
    }
    
    //below and to the left of the goal
    if (pPo->new_x<goal_mid.at(0) && pPo->new_y<goal_mid.at(1) && pPo->new_theta==0)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        beta = gamma;
    }
    if (pPo->new_x<goal_mid.at(0) && pPo->new_y<goal_mid.at(1) && pPo->new_theta>0 && pPo->new_theta<Pi/2)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        zeta = gamma;
        if (pPo->new_theta<zeta)
        {
            beta = zeta-pPo->new_theta;
        }
        if (pPo->new_theta==zeta)
        {
            beta = 0;
        }
        if (pPo->new_theta>zeta)
        {
            beta = pPo->new_theta-zeta;
        }
    }
    if (pPo->new_x<goal_mid.at(0) && pPo->new_y<goal_mid.at(1) && pPo->new_theta==Pi/2)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        beta = pPo->new_theta-gamma;
    }
    if (pPo->new_x<goal_mid.at(0) && pPo->new_y<goal_mid.at(1) && pPo->new_theta>Pi/2 && pPo->new_theta<Pi)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        beta = pPo->new_theta-gamma;
    }
    if (pPo->new_x<goal_mid.at(0) && pPo->new_y<goal_mid.at(1) && pPo->new_theta==Pi)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        beta = pPo->new_theta-gamma;
    }
    if (pPo->new_x<goal_mid.at(0) && pPo->new_y<goal_mid.at(1) && pPo->new_theta>Pi && pPo->new_theta<3*Pi/2)
    {
        double beta_1 = 0;
        double beta_2 = 0;
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        beta_1 = pPo->new_theta-gamma;
        beta_2 = (2*Pi)-beta_1;
        if (beta_1 < beta_2)
        {
            beta = beta_1;
        }
        else
        {
            beta = beta_2;
        }
    }
    if (pPo->new_x<goal_mid.at(0) && pPo->new_y<goal_mid.at(1) && pPo->new_theta==3*Pi/2)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        beta = (Pi/2)+gamma;
    }
    if (pPo->new_x<goal_mid.at(0) && pPo->new_y<goal_mid.at(1) && pPo->new_theta>3*Pi/2 && pPo->new_theta<2*Pi)
    {
        del_x = abs(pPo->new_x-goal_mid.at(0));
        del_y= abs(pPo->new_y-goal_mid.at(1));
        gamma = atan(del_x/del_y);
        alpha = (Pi/2)-gamma;
        beta = (2*Pi)-pPo->new_theta+gamma;
    }
    
    //directly to the left of the goal
    if (pPo->new_x<goal_mid.at(0) && pPo->new_y==goal_mid.at(1) && pPo->new_theta==0)
    {
        beta = Pi/2;
    }
    if (pPo->new_x<goal_mid.at(0) && pPo->new_y==goal_mid.at(1) && pPo->new_theta>0 && pPo->new_theta<Pi/2)
    {
        beta = (Pi/2)-pPo->new_theta;
    }
    if (pPo->new_x<goal_mid.at(0) && pPo->new_y==goal_mid.at(1) && pPo->new_theta==Pi/2)
    {
        beta = 0;
    }
    if (pPo->new_x<goal_mid.at(0) && pPo->new_y==goal_mid.at(1) && pPo->new_theta>Pi/2 && pPo->new_theta<Pi)
    {
        beta = (Pi/2)-pPo->new_theta;
    }
    if (pPo->new_x<goal_mid.at(0) && pPo->new_y==goal_mid.at(1) && pPo->new_theta==Pi)
    {
        beta = Pi/2;
    }
    if (pPo->new_x<goal_mid.at(0) && pPo->new_y==goal_mid.at(1) && pPo->new_theta>Pi && pPo->new_theta<3*Pi/2)
    {
        beta = pPo->new_theta-(2*Pi);
    }
    if (pPo->new_x<goal_mid.at(0) && pPo->new_y==goal_mid.at(1) && pPo->new_theta==3*Pi/2)
    {
        beta = Pi;
    }
    if (pPo->new_x<goal_mid.at(0) && pPo->new_y==goal_mid.at(1) && pPo->new_theta>3*Pi/2 && pPo->new_theta<2*Pi)
    {
        beta = (5*Pi/2)-pPo->new_theta;
    }
}


//-----------------------------------------------------------
//Calculates the new x position
void Simulator::Calculate_New_X(Policy* pPo)
{
    pPo->current_x = pPo->new_x;
    pPo->new_x = pPo->current_x + pP->v*sin(pPo->current_theta)*pP->dt;
    pPo->path.push_back(pPo->new_x);
}


//-----------------------------------------------------------
//Calculates the new y position
void Simulator::Calculate_New_Y(Policy* pPo)
{
    pPo->current_y = pPo->new_y;
    pPo->new_y = pPo->current_y + pP->v*cos(pPo->current_theta)*pP->dt;
    pPo->path.push_back(pPo->new_y);
}


//-----------------------------------------------------------
//Calculates the new theta
void Simulator::Calculate_New_Theta(Policy* pPo)
{
    pPo->current_theta = pPo->new_theta;
    pPo->new_theta = pPo->current_theta + pPo->current_omega*pP->dt;
    pPo->all_theta.push_back(pPo->new_theta);
}


//-----------------------------------------------------------
//Calculates the new controller
void Simulator::Calculate_New_Controller(Policy* pPo)
{
    pPo->current_u = pPo->new_u;
    vector<double> a;
    a.push_back(pPo->current_x);
    a.push_back(pPo->current_y);
    a.push_back(pPo->current_theta);
    a.push_back(pPo->current_omega);
    
    //NN.set_vector_input(a);
    //pPo->new_u = NN.execute();
    a.clear();
    
    pPo->new_u = 0;
    pPo->all_u.push_back(pPo->new_u);
}


//-----------------------------------------------------------
//Calculates the new omega
void Simulator::Calculate_New_Omega(Policy* pPo)
{
    Calculate_New_Controller(pPo);
    pPo->current_omega = pPo->new_omega;
    pPo->new_omega = pPo->current_omega + ((pPo->new_u + pPo->current_omega)*pP->dt)/pP->T;
    pPo->all_omega.push_back(pPo->new_omega);
}


//-----------------------------------------------------------
//Checks that the agent is still in the map space
void Simulator::Check_If_In_Map(Policy* pPo)
{
    if (pPo->new_x<0 || pPo->new_x>pP->x_max || pPo->new_y<0 || pPo->new_y>pP->y_max)
    {
        cout << "OUT OF MAP SPACE" << endl;
        pPo->in_map = 1;
    }
}


//-----------------------------------------------------------
//Calculates the agents distance to the goal
void Simulator::Get_Distance_to_Goal(Policy* pPo)
{
    
}


//-----------------------------------------------------------
//Outputs all the state information for that time step
void Simulator::Output_State_Info(Policy* pPo, double t)
{
    cout << "--------------------------------------" << endl;
    cout << "Current Time" << "\t" << t << endl;
    cout << "New X" << "\t" << "New Y" << "\t" << "New Theta" << "\t" << "New Omega" << "\t" << "New U" << endl;
    cout << pPo->new_x << "\t" << pPo->new_y << "\t" << pPo->new_theta << "\t" << pPo->new_omega << "\t" << pPo->new_u << endl;
}


//-----------------------------------------------------------
//Runs the entire simulation process
void Simulator::Simulate(Policy* pPo, vector<double> goal_mid)
{
    neural_network NN;
    NN.setup(pP->num_inputs, pP->num_hidden_nodes, pP->num_outputs);
    NN.set_in_min_max(0, pP->x_max);
    NN.set_out_min_max(pP->u_min, pP->u_max);
    Give_Weights_To_NN(pPo);
    double t=0;
    int ts=0;
    pPo->in_map = 0;
    while (t<pP->time_max)
    {
        if (t==0)
        {
            Agent_Set_Up(pPo);
            Get_Angle_Error(pPo, goal_mid);
            Get_Distance_to_Goal(pPo);
            Output_State_Info(pPo, t);
            Check_If_In_Map(pPo);
        }
        if (t>0)
        {
            Get_Angle_Error(pPo, goal_mid);
            Calculate_New_X(pPo);
            Calculate_New_Y(pPo);
            Calculate_New_Theta(pPo);
            Calculate_New_Omega(pPo);
            Get_Distance_to_Goal(pPo);
            Output_State_Info(pPo, t);
            Check_If_In_Map(pPo);
        }
        if (pPo->in_map == 1)
        {
            break;
        }
        ts += 1;
        t += pP->dt;
    }
}

#endif /* Simulator_hpp */
