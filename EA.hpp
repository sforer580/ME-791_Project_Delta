//
//  EA.hpp
//  ME-791_SF_Project_Delta_project
//
//  Created by Scott S Forer on 4/6/17.
//  Copyright © 2017 Scott S Forer. All rights reserved.
//

#ifndef EA_hpp
#define EA_hpp

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
    void Build_Pop();
    void Assign_Weights();
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
    cout << indiv.at(0).pol.size() << endl;
    Assign_Weights();
}


//-----------------------------------------------------------
//Assigns the weights for each policy
void EA::Assign_Weights()
{
    for (int p=0; p<pP->pop_size; p++)
    {
        cout << "Policy" << "\t" << p << endl;
        for (int w=0; w<pP->num_weights; w++)
        {
            double r = 2*((double)rand()/RAND_MAX)+(-1);
            indiv.at(0).pol.at(p).weights.push_back(r);
            cout << r << "\t";
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
        
    }
}


//-----------------------------------------------------------
//Runs the entire simulation for each policy
void EA::Run_Simulation()
{
    for (int p=0; p<indiv.at(0).pol.size(); p++)
    {
        Simulator S;
        S.pP = this->pP;
        //S.Simulate();
    }
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
    }
    else
    {
        loser = index_1;
    }
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
}


//-----------------------------------------------------------
//Runs the entire mutation process
void EA::Mutation(Policy &M)
{
    
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
        if (gen <= pP->gen_max-1)
        {
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
