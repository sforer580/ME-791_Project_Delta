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

using namespace std;

class EA
{
    friend class Parameters;
    friend class Simulator;
    friend class Agent;
    friend class Policy;
    
    public:
    Parameters* pP;
    vector<Agent> indiv;
    
    void build_pop();
    void Assign_Weights();
};

//Builds the population of policies for an agent
void EA::build_pop()
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


#endif /* EA_hpp */
