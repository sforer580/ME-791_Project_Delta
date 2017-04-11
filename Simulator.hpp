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
    
    void Initalize_Goal();
    void Initalize_Agent(Policy* pPo);
    void Get_Distance_to_Goal(Policy* pPo);
    void Calculate_New_X(Policy* pPo);
    void Calculate_New_Y(Policy* pPo);
    void Calculate_New_Theta(Policy* pPo);
    void Calculate_New_Controller(Policy* pPo);
    void Calculate_New_Omega(Policy* pPo);
    void Output_State_Info(Policy* pPo, double t);
    void Check_If_In_Map(Policy* pPo);
    
    void Simulate(Policy* pPo);
    
};


//-----------------------------------------------------------
//Initalizes the goal plane
void Simulator::Initalize_Goal()
{
    goal_1.push_back((rand() / double(RAND_MAX))*(pP->x_max));
    goal_1.push_back((rand() / double(RAND_MAX))*(pP->y_max));
    
    goal_2.push_back(goal_1.at(0)+5);
    if (goal_2.at(0)>pP->x_max)
    {
        goal_2.at(0) = goal_1.at(0)-5;
    }
    goal_2.push_back(goal_1.at(1)+5);
    if (goal_2.at(1)>pP->x_max)
    {
        goal_2.at(1) = goal_1.at(1)-5;
    }
    assert(goal_1.at(0)>=0 && goal_1.at(0)<=pP->x_max);
    assert(goal_1.at(1)>=0 && goal_1.at(1)<=pP->y_max);
    assert(goal_2.at(0)>=0 && goal_2.at(0)<=pP->x_max);
    assert(goal_2.at(1)>=0 && goal_2.at(1)<=pP->y_max);
}


//-----------------------------------------------------------
//Initalizes the agents starting state information
void Simulator::Initalize_Agent(Policy* pPo)
{
    pPo->current_x = 1;
    pPo->current_y = 1;
    pPo->current_theta = Pi/2;
    pPo->current_omega = 0;
    pPo->current_u = 0;
    
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
void Simulator::Simulate(Policy* pPo)
{
    double t=0;
    int ts=0;
    pPo->in_map = 0;
    while (t<pP->time_max)
    {
        if (t==0)
        {
            Initalize_Goal();
            Initalize_Agent(pPo);
            Get_Distance_to_Goal(pPo);
            Output_State_Info(pPo, t);
            Check_If_In_Map(pPo);
        }
        if (t>0)
        {
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
        t += pP->time_step;
    }
}

#endif /* Simulator_hpp */
