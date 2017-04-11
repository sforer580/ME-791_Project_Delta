//
//  Simulator.hpp
//  ME-791_SF_Project_Delta_project
//
//  Created by Scott S Forer on 4/6/17.
//  Copyright © 2017 Scott S Forer. All rights reserved.
//

#ifndef Simulator_hpp
#define Simulator_hpp

#include <stdio.h>

using namespace std;

class Simulator
{
    friend class EA;
    friend class Parameters;
    friend class Agent;
    friend class Policy;
    
public:
    Parameters* pP;
    
    void Simulate();
    
};


//-----------------------------------------------------------
//Runs the entire simulation process
void Simulate()
{
    double t=0;
    while (t<pP->time_max)
    {
        
        t += pP->time_step;
    }
}

#endif /* Simulator_hpp */
