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
    
    void Give_Weights_To_NN(Policy* pPo);
    void Initalize_Agent_Sensing(Policy* pPo);
    void Reset_Agent_Sensing(Policy* pPo);
    void Get_Angle_Error(Policy* pPo, vector<double> goal_mid, int ts);
    void Get_Distance_to_Goal(Policy* pPo, vector<double> goal_mid, int ts);
    void Calculate_New_X(Policy* pPo, int ts);
    void Calculate_New_Y(Policy* pPo, int ts);
    void Calculate_New_Theta(Policy* pPo, int ts);
    void Calculate_New_Controller(Policy* pPo, int ts);
    void Calculate_New_Omega(Policy* pPo, int ts);
    void Check_If_In_Goal_Box(Policy* pPo, int ts, vector<double> goal_box);
    void Check_Line_Intersection(Policy* pPo, int ts, vector<double> goal_line);
    void Check_If_Passed_Goal(Policy* pPo, int ts, vector<double> goal_line, vector<double> goal_box);
    void Output_State_Info(Policy* pPo, double t, int ts);
    void Check_If_In_Map(Policy* pPo, int ts);
    
    void Simulate(Policy* pPo, vector<double> goal_line, vector<double> goal_mid, vector<double> goal_box);
    
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
//Initalizes the agents sensing data
void Simulator::Initalize_Agent_Sensing(Policy *pPo)
{
    pPo->in_map = 1;
    pPo->in_goal_box = 0;
    pPo->reached_goal = 0;
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
    double x_1 = sin(pPo->theta.at(ts));
    double y_1 = cos(pPo->theta.at(ts));
    assert (x_1*x_1+y_1*y_1 == 1);
    
    double del_x = goal_mid.at(0) - pPo->x.at(ts);
    double del_y = goal_mid.at(1) - pPo->y.at(ts);
    double x_2 = del_x/pPo->dist_to_goal.at(ts);
    double y_2 = del_y/pPo->dist_to_goal.at(ts);
    assert (x_2*x_2+y_2*y_2>1-0.001 && x_2*x_2+y_2*y_2<1+0.001);
    double beta = acos(x_1*x_2+y_1*y_2);
    //cout << "BETA" << "\t" << beta*(180/Pi) << endl;
    //cout << "cp" << endl;
    pPo->angle_error.push_back(beta);

}


//-----------------------------------------------------------
//Calculates the new x position
void Simulator::Calculate_New_X(Policy* pPo, int ts)
{
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
    pPo->theta.push_back(pPo->theta.at(ts-1) + pPo->omega.at(ts-1)*pP->dt);
}


//-----------------------------------------------------------
//Calculates the new controller
/*
void Simulator::Calculate_New_Controller(Policy* pPo, int ts)
{
    double u = 0;
    pPo->u.push_back(u);
}
 */


//-----------------------------------------------------------
//Calculates the new omega
void Simulator::Calculate_New_Omega(Policy* pPo, int ts)
{
    //Calculate_New_Controller(pPo, ts);
    pPo->omega.push_back(pPo->omega.at(ts-1) + ((pPo->u.at(ts-1) + pPo->omega.at(ts-1))*pP->dt)/pP->T);
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
    cout << "New X" << "\t" << "New Y" << "\t" << "New Theta" << "\t" << "New Omega" << "\t" << "New U" << "\t" << "Angle Error" << endl;
    cout << pPo->x.at(ts) << "\t" << pPo->y.at(ts) << "\t" << pPo->theta.at(ts) << "\t" << pPo->omega.at(ts) << "\t" << pPo->u.at(ts) << "\t" << pPo->angle_error.at(ts)*(180/Pi) << endl;
    cout << endl;
}


//-----------------------------------------------------------
//Runs the entire simulation process
void Simulator::Simulate(Policy* pPo, vector<double> goal_line, vector<double> goal_mid, vector<double> goal_box)
{
    neural_network NN;
    NN.setup(pP->num_inputs, pP->num_hidden_nodes, pP->num_outputs);
    NN.set_in_min_max(0, pP->x_max);
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
            Output_State_Info(pPo, t, ts);
            Check_If_Passed_Goal(pPo, ts, goal_line,goal_box);
            Check_If_In_Map(pPo, ts);
        }
        if (t>0)
        {
            Reset_Agent_Sensing(pPo);
            Calculate_New_X(pPo, ts);
            Calculate_New_Y(pPo, ts);
            Calculate_New_Theta(pPo, ts);
            
            a.push_back(pPo->x.at(ts-1));
            a.push_back(pPo->y.at(ts-1));
            a.push_back(pPo->theta.at(ts-1));
            a.push_back(pPo->omega.at(ts-1));
            a.push_back(pPo->angle_error.at(ts-1));
            NN.set_vector_input(a);
            vector<double> user_input;
            for (int w=0; w<pPo->weights.size(); w++)
            {
                user_input.push_back(pPo->weights.at(w));
            }
            NN.set_weights(user_input, true);
            NN.execute();
            pPo->u.push_back(NN.get_output(0));
            a.clear();
            
            
            Calculate_New_Omega(pPo, ts);
            Get_Distance_to_Goal(pPo, goal_mid, ts);
            Get_Angle_Error(pPo, goal_mid, ts);
            Output_State_Info(pPo, t, ts);
            Check_If_Passed_Goal(pPo, ts, goal_line,goal_box);
            Check_If_In_Map(pPo, ts);
        }
        if (pPo->reached_goal == 1)
        {
            cout << "REACHED GOAL" << endl;
            break;
        }
        if (pPo->in_map == 0)
        {
            cout << "OUT OF MAP SPACE" << endl;
            break;
        }
        ts += 1;
        t += pP->dt;
    }
    if (pPo->reached_goal==0 && pPo->in_map==1)
    {
        cout << "OUT OF TIME" << endl;
    }
}

#endif /* Simulator_hpp */
