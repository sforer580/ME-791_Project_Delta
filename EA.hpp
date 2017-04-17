//
//  EA.hpp
//  ME-791_SF_Project_Delta_project
//
//  Created by Scott S Forer on 4/6/17.
//  Copyright © 2017 Scott S Forer. All rights reserved.
//

#ifndef EA_hpp
#define EA_hpp
# define Pi           3.14159265358979323846  /* pi */

#include <stdio.h>

#include "Simulator.hpp"

using namespace std;

class EA
{
    friend class Parameters;
    friend class Simulator;
    friend class Agent;
    friend class Policy;
    friend class neural_network;
    
    public:
    Parameters* pP;
    vector<Agent> indiv;
    
    //EA Functions
    vector<double> goal_line;
    vector<double> goal_mid;
    double goal_angle;
    vector<double> goal_box;
    
    
    void Build_Pop();
    void Assign_Weights();
    void Initalize_Agent();
    void Initalize_Goal();
    void Create_Goal_Box();
    void Evalutate();
    void Run_Simulation();
    void Get_Fitness();
    int Binary_Select();
    void Down_Select();
    void Mutation(Policy &M);
    void Replicate();
    struct Less_Than_Policy_Fitness;
    
    void Run_Project_Delta();
};


//-----------------------------------------------------------
//Builds the population of policies for an agent
void EA::Build_Pop()
{
    Agent A;
    indiv.push_back(A);
    for (int p=0; p<pP->pop_size; p++)
    {
        Policy PO;
        indiv.at(0).pol.push_back(PO);
    }
    assert (indiv.at(0).pol.size() == pP->pop_size);
    cout << "Number of Policies" << "\t" << indiv.at(0).pol.size() << endl;
    cout << endl;
    Assign_Weights();
}


//-----------------------------------------------------------
//Assigns the weights for each policy
void EA::Assign_Weights()
{
    neural_network NN;
    NN.setup(pP->num_inputs, pP->num_hidden_nodes, pP->num_outputs);
    pP->num_weights = NN.intended_size;
    for (int p=0; p<pP->pop_size; p++)
    {
        for (int w=0; w<pP->num_weights; w++)
        {
            double r = 2*((double)rand()/RAND_MAX)+(-1);
            indiv.at(0).pol.at(p).weights.push_back(r);
        }
        assert (indiv.at(0).pol.at(p).weights.size() == pP->num_weights);
        
        cout << "Policy" << "\t" << p << "\t" << "Weights" << "\t" << indiv.at(0).pol.at(p).weights.size() << endl;
        for (int w=0; w<indiv.at(0).pol.at(p).weights.size(); w++)
        {
            cout << indiv.at(0).pol.at(p).weights.at(w) << "\t";
        }
        cout << endl;
        cout << endl;
    }
}


//-----------------------------------------------------------
//Gets the fitness for each policy
void EA::Get_Fitness()
{
    for (int p=0; p<indiv.at(0).pol.size(); p++)
    {
        indiv.at(0).pol.at(p).fitness = 0;
    }

    for (int p=0; p<indiv.at(0).pol.size(); p++)
    {
        if (indiv.at(0).pol.at(p).in_map == 0)
        {
            indiv.at(0).pol.at(p).fitness = 1000;
        }
        for (int ts=0; ts<indiv.at(0).pol.at(p).angle_error.size(); ts++)
        {
            indiv.at(0).pol.at(p).fitness += indiv.at(0).pol.at(p).angle_error.at(ts);
        }
    }
    
    cout << "Population Fitness" << endl;
    for (int p=0; p<indiv.at(0).pol.size(); p++)
    {
        cout << "Policy" << "\t" << indiv.at(0).pol.at(p).fitness << "\t";
    }
    cout << endl;
    
}


//-----------------------------------------------------------
//Initalizes the agents starting state information
void EA::Initalize_Agent()
{
    double x = 0;
    double y = 0;
    double theta = 0;
    double omega = 0;
    double u = 0;
    for (int p=0; p<pP->pop_size; p++)
    {
        indiv.at(0).pol.at(p).x.push_back(x);
        indiv.at(0).pol.at(p).y.push_back(y);
        indiv.at(0).pol.at(p).theta.push_back(theta);
        indiv.at(0).pol.at(p).omega.push_back(omega);
        indiv.at(0).pol.at(p).u.push_back(u);
    }
}


//-----------------------------------------------------------
//Initalizes the goal plane
void EA::Initalize_Goal()
{
    goal_line.empty();
    goal_line.push_back((rand() / double(RAND_MAX))*(pP->x_max));
    goal_line.push_back((rand() / double(RAND_MAX))*(pP->y_max));
    
    goal_line.push_back(goal_line.at(0));
    goal_line.push_back(goal_line.at(1)-10);
    if (goal_line.at(3)>0)
    {
        goal_line.at(0) = goal_line.at(1)+10;
    }
    goal_line.at(0) = 50;
    goal_line.at(1) = 55;
    goal_line.at(2) = 50;
    goal_line.at(3) = 45;
    assert(goal_line.at(0)>=0 && goal_line.at(0)<=pP->x_max);
    assert(goal_line.at(1)>=0 && goal_line.at(1)<=pP->y_max);
    assert(goal_line.at(0)>=0 && goal_line.at(0)<=pP->x_max);
    assert(goal_line.at(1)>=0 && goal_line.at(1)<=pP->y_max);
    
    goal_mid.empty();
    goal_mid.push_back(goal_line.at(0));
    double min = 0;
    if (goal_line.at(1) < goal_line.at(3))
    {
        min = goal_line.at(1);
    }
    else
    {
        min = goal_line.at(3);
    }
    goal_mid.push_back((abs(goal_line.at(1)-goal_line.at(3))/2)+min);
    cout << "Goal Location" << endl;
    cout << "Goal_1" << "\t" << goal_line.at(0) << "\t" << goal_line.at(1) << endl;
    cout << "Goal_2" << "\t" << goal_line.at(2) << "\t" << goal_line.at(3) << endl;
    cout << "Goal-mid" << "\t" << goal_mid.at(0) << "\t" << goal_mid.at(1) << endl;
    cout << endl;
    Create_Goal_Box();
}


//-----------------------------------------------------------
//Creates a box around the goal
void EA::Create_Goal_Box()
{
    goal_box.empty();
    goal_box.push_back(goal_mid.at(0)-pP->v*pP->dt);
    goal_box.push_back(goal_mid.at(1)+5);
    goal_box.push_back(goal_mid.at(0)+pP->v*pP->dt);
    goal_box.push_back(goal_mid.at(1)+5);
    goal_box.push_back(goal_mid.at(0)-pP->v*pP->dt);
    goal_box.push_back(goal_mid.at(1)-5);
    goal_box.push_back(goal_mid.at(0)+pP->v*pP->dt);
    goal_box.push_back(goal_mid.at(1)-5);
    
    cout << "Goal Box" << "\t";
    for (int i=0; i<8; i++)
    {
        cout << goal_box.at(i) << "\t";
    }
    cout << endl;

}


//-----------------------------------------------------------
//Runs the entire simulation for each policy
void EA::Run_Simulation()
{
    for (int p=0; p<indiv.at(0).pol.size(); p++)
    {
        cout << "/////////////////////////////////////////////////////////////" << endl;
        cout << "Policy" << "\t" << p << endl;
        Simulator S;
        S.pP = this->pP;
        Policy* pPo;
        pPo = & indiv.at(0).pol.at(p);
        S.Simulate(pPo, goal_line, goal_mid, goal_box);
    }
    //cout << "cp" << endl;
}


//-----------------------------------------------------------
//Runs the entire evaluation process
void EA::Evalutate()
{
    Run_Simulation();
    Get_Fitness();
}


//-----------------------------------------------------------
//Randomly selects two individuals and decides which one will die based on their fitness
int EA::Binary_Select()
{
    int loser;
    int index_1 = rand() % indiv.at(0).pol.size();
    int index_2 = rand() % indiv.at(0).pol.size();
    while (index_1 == index_2)
    {
        index_2 = rand() % indiv.at(0).pol.size();
    }
    if(indiv.at(0).pol.at(index_1).fitness < indiv.at(0).pol.at(index_2).fitness)
    {
        loser = index_2;
        assert(indiv.at(0).pol.at(index_1).fitness < indiv.at(0).pol.at(index_2).fitness);
    }
    else
    {
        loser = index_1;
        assert(indiv.at(0).pol.at(index_1).fitness >= indiv.at(0).pol.at(index_2).fitness);
    }
    //cout << loser << endl;
    return loser;
}


//-----------------------------------------------------------
//Runs the entire down select process
void EA::Down_Select()
{
    for(int k=0; k<pP->pop_size/2; k++)
    {
        int kill = 0;
        kill = Binary_Select();
        indiv.at(0).pol.erase(indiv.at(0).pol.begin() + kill);
    }
    assert(indiv.at(0).pol.size() == pP->pop_size/2);
}


//-----------------------------------------------------------
//Runs the entire mutation process
void EA::Mutation(Policy &M)
{
    for (int w=0; w<pP->num_weights; w++)
    {
        double random = ((double)rand()/RAND_MAX);
        if (random <= pP->mutation_rate)
        {
            double R1 = ((double)rand()/RAND_MAX) * pP->range;
            double R2 = ((double)rand()/RAND_MAX) * pP->range;
            M.weights.at(w) = M.weights.at(w) + (R1-R2);
        }
    }
}


//-----------------------------------------------------------
//Runs the entire replication process
void EA::Replicate()
{
    int to_replicate = pP->pop_size/2;
    for (int r=0; r<to_replicate; r++)
    {
        Policy M;
        int spot = rand() % indiv.at(0).pol.size();
        M = indiv.at(0).pol.at(spot);
        Mutation(M);
        indiv.at(0).pol.push_back(M);
    }
    assert(indiv.at(0).pol.size() == pP->pop_size);
}


//-----------------------------------------------------------
//Sorts the population based on their fitness from lowest highest
struct EA::Less_Than_Policy_Fitness
{
    inline bool operator() (const Policy& struct1, const Policy& struct2)
    {
        return (struct1.fitness < struct2.fitness);
    }
};


//-----------------------------------------------------------
//Runs the entire project
void EA::Run_Project_Delta()
{
    Build_Pop();
    for (int gen=0; gen<pP->gen_max; gen++)
    {
        if (gen < pP->gen_max-1)
        {
            Initalize_Agent();
            Initalize_Goal();
            Evalutate();
            sort(indiv.at(0).pol.begin(), indiv.at(0).pol.end(), Less_Than_Policy_Fitness());
            Down_Select();
            Replicate();
        }
        if (gen == pP->gen_max-1)
        {
            Evalutate();
            sort(indiv.at(0).pol.begin(), indiv.at(0).pol.end(), Less_Than_Policy_Fitness());
        }
    }
}


#endif /* EA_hpp */
