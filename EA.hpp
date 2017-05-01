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
    vector<double> goal_box;
    
    
    void Build_Pop();
    void Assign_Weights();
    void Initalize_Agent(double x, double y, double theta, double omega, double u);
    void Initalize_Goal();
    void Create_Goal_Box();
    void Evalutate();
    void Output_Pop_Data();
    void Run_Simulation();
    void Get_Fitness();
    int Binary_Select();
    void Down_Select();
    void Mutation(Policy &M);
    void Replicate();
    struct Less_Than_Policy_Fitness;
    void Clear_Policy_Data();
    void Store_Best_Policy_Data();
    void Run_LR_1();
    void Run_LR_2();
    void Run_HR_1(int sr);
    void Run_HR_3();
    void Run_HR_4();
    
    void Run_Project_Delta();
    
    
    //Statistics
    vector<double> best_x;
    vector<double> best_y;
    vector<double> best_theta;
    vector<double> best_omega;
    vector<double> best_fitness;
    void Write_txt_files();
    void Delete_Text_Files();
};


//-----------------------------------------------------------
//Runs LR_1
void EA::Run_LR_1()
{
    assert(indiv.size()==1);
}

//-----------------------------------------------------------
//Runs LR_2
void EA::Run_LR_2()
{
    assert(goal_line.size()==4);
}

//-----------------------------------------------------------
//Builds the population of policies for an agent
void EA::Build_Pop()
{
    indiv.clear();
    Agent A;
    indiv.push_back(A);
    Run_LR_1();
    for (int p=0; p<pP->pop_size; p++)
    {
        Policy PO;
        indiv.at(0).pol.push_back(PO);
    }
    assert (indiv.at(0).pol.size() == pP->pop_size);
    //cout << "Number of Policies" << "\t" << indiv.at(0).pol.size() << endl;
    //cout << endl;
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
        /*
        cout << "Policy" << "\t" << p << "\t" << "Weights" << "\t" << indiv.at(0).pol.at(p).weights.size() << endl;
        for (int w=0; w<indiv.at(0).pol.at(p).weights.size(); w++)
        {
            cout << indiv.at(0).pol.at(p).weights.at(w) << "\t";
        }
        cout << endl;
        cout << endl;
         */
    }
}


//-----------------------------------------------------------
//Initalizes the agents starting state information
void EA::Initalize_Agent(double x, double y, double theta, double omega, double u)
{
    if (pP->HR_1==1)
    {
        x = 0;
        y = 50;
        theta = (Pi/2);
        omega = 0;
        u = 0;
        for (int p=0; p<pP->pop_size; p++)
        {
            indiv.at(0).pol.at(p).x.push_back(x);
            indiv.at(0).pol.at(p).y.push_back(y);
            indiv.at(0).pol.at(p).theta.push_back(theta);
            indiv.at(0).pol.at(p).omega.push_back(omega);
            indiv.at(0).pol.at(p).u.push_back(u);
        }
    }
    if (pP->HR_3==1)
    {
        for (int p=0; p<pP->pop_size; p++)
        {
            indiv.at(0).pol.at(p).x.push_back(x);
            indiv.at(0).pol.at(p).y.push_back(y);
            indiv.at(0).pol.at(p).theta.push_back(theta);
            indiv.at(0).pol.at(p).omega.push_back(omega);
            indiv.at(0).pol.at(p).u.push_back(u);
        }
    }
}


//-----------------------------------------------------------
//Initalizes the goal plane
void EA::Initalize_Goal()
{
    goal_line.clear();
    goal_line.push_back((rand() / double(RAND_MAX))*(pP->x_max));
    goal_line.push_back((rand() / double(RAND_MAX))*(pP->y_max));
    
    goal_line.push_back(goal_line.at(0));
    goal_line.push_back(goal_line.at(1)-10);
    if (goal_line.at(3)>0)
    {
        goal_line.at(0) = goal_line.at(1)+10;
    }
    goal_line.at(0) = 50;
    goal_line.at(1) = 65;
    goal_line.at(2) = 50;
    goal_line.at(3) = 35;
    assert(goal_line.at(0)>=0 && goal_line.at(0)<=pP->x_max);
    assert(goal_line.at(1)>=0 && goal_line.at(1)<=pP->y_max);
    assert(goal_line.at(0)>=0 && goal_line.at(0)<=pP->x_max);
    assert(goal_line.at(1)>=0 && goal_line.at(1)<=pP->y_max);
    
    Run_LR_2();
    
    goal_mid.clear();
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
    /*
    cout << "Goal Location" << endl;
    cout << "Goal_1" << "\t" << goal_line.at(0) << "\t" << goal_line.at(1) << endl;
    cout << "Goal_2" << "\t" << goal_line.at(2) << "\t" << goal_line.at(3) << endl;
    cout << "Goal-mid" << "\t" << goal_mid.at(0) << "\t" << goal_mid.at(1) << endl;
    cout << endl;
     */
    Create_Goal_Box();
}


//-----------------------------------------------------------
//Creates a box around the goal
void EA::Create_Goal_Box()
{
    goal_box.clear();
    goal_box.push_back(goal_mid.at(0)-pP->v*pP->dt);
    goal_box.push_back(goal_mid.at(1)+5);
    goal_box.push_back(goal_mid.at(0)+pP->v*pP->dt);
    goal_box.push_back(goal_mid.at(1)+5);
    goal_box.push_back(goal_mid.at(0)-pP->v*pP->dt);
    goal_box.push_back(goal_mid.at(1)-5);
    goal_box.push_back(goal_mid.at(0)+pP->v*pP->dt);
    goal_box.push_back(goal_mid.at(1)-5);
    
    /*
    cout << "Goal Box" << "\t";
    for (int i=0; i<8; i++)
    {
        cout << goal_box.at(i) << "\t";
    }
    cout << endl;
     */
}


//-----------------------------------------------------------
//Gets the fitness for each policy
void EA::Get_Fitness()
{
    //satisfies MR_4
    for (int p=0; p<indiv.at(0).pol.size(); p++)
    {
        indiv.at(0).pol.at(p).fitness = 0;
    }
    
    for (int p=0; p<indiv.at(0).pol.size(); p++)
    {
        if (indiv.at(0).pol.at(p).in_map == 0)
        {
            indiv.at(0).pol.at(p).fitness = 1000;
            indiv.at(0).pol.at(p).fitness += 10*((pP->time_max/pP->dt) - (indiv.at(0).pol.at(p).total_time_steps*pP->dt));
        }
        for (int ts=0; ts<indiv.at(0).pol.at(p).angle_error.size(); ts++)
        {
            if (indiv.at(0).pol.at(p).angle_error.at(ts) < 0)
            {
                indiv.at(0).pol.at(p).angle_error.at(ts) = indiv.at(0).pol.at(p).angle_error.at(ts)*-1;
            }
            indiv.at(0).pol.at(p).fitness += indiv.at(0).pol.at(p).angle_error.at(ts);
        }
    }
}


//-----------------------------------------------------------
//Outputs the population data
void EA::Output_Pop_Data()
{
    cout << "Population Fitness" << endl;
    for (int p=0; p<indiv.at(0).pol.size(); p++)
    {
        cout << "Policy" << "\t" << p << "\t" << indiv.at(0).pol.at(p).fitness << "\t";
        if (indiv.at(0).pol.at(p).reached_goal==1)
        {
            cout << "REACHED GOAL" << endl;
        }
        if (indiv.at(0).pol.at(p).in_map==0)
        {
            cout << "OUT OF MAP" << endl;
        }
        if (indiv.at(0).pol.at(p).reached_goal==0 &&  indiv.at(0).pol.at(p).in_map==1)
        {
            cout << "OUT OF TIME" << endl;
        }
    }
    cout << endl;
}


//-----------------------------------------------------------
//Runs the entire simulation for each policy
void EA::Run_Simulation()
{
    for (int p=0; p<indiv.at(0).pol.size(); p++)
    {
        //cout << "/////////////////////////////////////////////////////////////" << endl;
        //cout << "Policy" << "\t" << p << endl;
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
    //satisfies LR_5
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
//
void EA::Write_txt_files()
{
    ofstream File1;
    File1.open("best_x.txt");
    for (int i=0; i<best_x.size(); i++)
    {
        File1 << best_x.at(i) << "\t";
    }
    
    ofstream File2;
    File2.open("best_y.txt");
    for (int i=0; i<best_y.size(); i++)
    {
        File2 << best_y.at(i) << "\t";
    }
    
    ofstream File3;
    File3.open("best_theta.txt");
    for (int i=0; i<best_theta.size(); i++)
    {
        File3 << best_theta.at(i) << "\t";
    }
    
    ofstream File4;
    File4.open("best_omega.txt");
    for (int i=0; i<best_omega.size(); i++)
    {
        File4 << best_omega.at(i) << "\t";
    }
    
    ofstream File5;
    File5.open("best_time.txt");
    File5 << indiv.at(0).pol.at(0).total_time_steps << endl;
    
    ofstream File6;
    File6.open("best_fitness.txt", ios_base::app);
    for (int i=0; i<best_fitness.size(); i++)
    {
        File6 << best_fitness.at(i) << "\t";
    }
    File6 << endl;
    
    File1.close();
    File2.close();
    File3.close();
    File4.close();
    File5.close();
    File6.close();
}


//-------------------------------------------------------------------------
//Deletes text files
void EA::Delete_Text_Files()
{
    if( remove( "best_fitness.txt" ) != 0 )
        perror( "ERROR DELETING FILE best_fitness" );
    else
        puts( "best_fitness FILE SUCCEDDFULLY DELETED" );
    cout << endl;
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
//Clears the data for each policy
void EA::Clear_Policy_Data()
{
    for (int p=0; p< pP->pop_size; p++)
    {
        indiv.at(0).pol.at(p).x.clear();
        indiv.at(0).pol.at(p).y.clear();
        indiv.at(0).pol.at(p).theta.clear();
        indiv.at(0).pol.at(p).omega.clear();
        indiv.at(0).pol.at(p).u.clear();
        indiv.at(0).pol.at(p).angle_error.clear();
    }
}


//-----------------------------------------------------------
//Stores the best policy data
void EA::Store_Best_Policy_Data()
{
    best_x.clear();
    best_y.clear();
    best_theta.clear();
    best_omega.clear();
    for (int i=0; i<indiv.at(0).pol.at(0).x.size(); i++)
    {
        best_x.push_back(indiv.at(0).pol.at(0).x.at(i));
    }
    for (int i=0; i<indiv.at(0).pol.at(0).y.size(); i++)
    {
        best_y.push_back(indiv.at(0).pol.at(0).y.at(i));
    }
    for (int i=0; i<indiv.at(0).pol.at(0).theta.size(); i++)
    {
        best_theta.push_back(indiv.at(0).pol.at(0).theta.at(i));
    }
    for (int i=0; i<indiv.at(0).pol.at(0).omega.size(); i++)
    {
        best_omega.push_back(indiv.at(0).pol.at(0).omega.at(i));
    }
    best_fitness.push_back(indiv.at(0).pol.at(0).fitness);
}


//-----------------------------------------------------------
//Runs HR_1
void EA::Run_HR_1(int sr)
{
    cout << "HR_1" << endl;
    pP->HR_1 = 1;
    Build_Pop();
    Initalize_Goal();
    double x;
    double y;
    double theta;
    double omega;
    double u;
    for (int gen=0; gen<pP->gen_max; gen++)
    {
        Clear_Policy_Data();
        if (gen%10 ==0)
        {
            cout << "------------------" << endl;
            //cout << "Generation" << "\t" << gen << endl;
            cout << sr << "::" << gen << endl;
        }
        if (gen < pP->gen_max-1)
        {
            Initalize_Agent(x, y, theta, omega, u);
            Evalutate();
            sort(indiv.at(0).pol.begin(), indiv.at(0).pol.end(), Less_Than_Policy_Fitness());
            Store_Best_Policy_Data();
            //Output_Pop_Data();
            Down_Select();
            Replicate();
        }
        if (gen == pP->gen_max-1)
        {
            Initalize_Agent(x, y, theta, omega, u);
            Evalutate();
            sort(indiv.at(0).pol.begin(), indiv.at(0).pol.end(), Less_Than_Policy_Fitness());
            //Output_Pop_Data();
            Store_Best_Policy_Data();
        }
    }
    Write_txt_files();
    best_fitness.clear();
    pP->HR_1 = 0;
    assert(indiv.at(0).pol.at(0).reached_goal == 1);
    
}


//-----------------------------------------------------------
//Runs HR_1
void EA::Run_HR_3()
{
    cout << "HR_3" << endl;
    pP->HR_3 = 1;
    Build_Pop();
    Initalize_Goal();
    double x;
    double y;
    double theta;
    double omega;
    double u;
    x = (rand() / double(RAND_MAX))*(pP->x_max);
    y = (rand() / double(RAND_MAX))*(pP->y_max);
    
    while (x>30 && x<70 && y<=30)
    {
        x = (rand() / double(RAND_MAX))*(pP->x_max);
    }
     
    //x = 70;
    //y = 40;
    if (x>30 && x<70)
    {
        while (y>30 && y<70)
        {
            y = (rand() / double(RAND_MAX))*(pP->y_max);
        }
    }
    if (pP->HR_3==1)
    {
        if (x<=30 && y<=30)
        {
            theta = (rand() / double(RAND_MAX))*(Pi/2);
        }
        if (x<=30 && y>30 && y<50)
        {
            theta = (Pi/4)+(rand() / double(RAND_MAX))*(Pi/4);
            theta = (Pi/2);
        }
        if (x<=30 && y==50)
        {
            theta = (Pi/4)+(rand() / double(RAND_MAX))*(Pi/4);
            theta = (Pi/2);
        }
        if (x<=30 && y>50 && y<70)
        {
            theta = (Pi/2)+(rand() / double(RAND_MAX))*(Pi/4);
            theta = (Pi/2);
        }
        if (x<=30 && y>=70)
        {
            theta = (Pi/2)+((rand() / double(RAND_MAX))*(Pi/2));
        }
        if (x>30 && x<50 && y>=70)
        {
            theta = (3*Pi/4)+((rand() / double(RAND_MAX))*(Pi/4));
            theta = (Pi);
        }
        if (x==50 && y>=70)
        {
            theta = (3*Pi/4)+((rand() / double(RAND_MAX))*(Pi/4));
            theta = (Pi);
        }
        if (x>50 && x<70 && y>=70)
        {
            theta = (Pi)+((rand() / double(RAND_MAX))*(Pi/4));
            theta = (Pi);
        }
        if (x>=70 && y>=70)
        {
            theta = (Pi)+((rand() / double(RAND_MAX))*(Pi/2));
        }
        if (x>70 && y<70 && y>50)
        {
            theta = (5*Pi/4)+((rand() / double(RAND_MAX))*(Pi/4));
            theta = (3*Pi/2);
        }
        if (x>70 && y==50)
        {
            theta = (5*Pi/4)+((rand() / double(RAND_MAX))*(Pi/2));
            theta = (3*Pi/2);
        }
        if (x>70 && y<50 && y>30)
        {
            theta = (3*Pi/2)+((rand() / double(RAND_MAX))*(Pi/4));
            theta = (3*Pi/2);
        }
        if (x>=70 && y<=30)
        {
            theta = (3*Pi/2)+((rand() / double(RAND_MAX))*(Pi/2));
            if (theta == 2*Pi)
            {
                theta = 0;
            }
            double check = theta*(180/Pi);
            check = theta*(180/Pi);
        }
        if (x<70 && x>50 && y<=30)
        {
            theta = (7*Pi/4)+((rand() / double(RAND_MAX))*(Pi/4));
            if (theta == 2*Pi)
            {
                theta = 0;
            }
            theta = 0;
        }
        if (x==50 && y<=30)
        {
            theta = (7*Pi/4)+((rand() / double(RAND_MAX))*(Pi/2));
            theta = 0;
            if (theta >= 2*Pi)
            {
                theta = theta-(2*Pi);
            }
            if (theta == 2*Pi)
            {
                theta = 0;
            }
        }
        if (x<50 && x>30 && y<=30)
        {
            theta = ((rand() / double(RAND_MAX))*(Pi/4));
            theta = 0;
        }
        double check = theta*(180/Pi);
        check = theta*(180/Pi);
        
        //theta = Pi/4;
        
    }
    omega = 0;
    u = 0;
    for (int gen=0; gen<pP->gen_max; gen++)
    {
        Clear_Policy_Data();
        if (gen%10 ==0)
        {
            cout << "------------------" << endl;
            cout << "Generation" << "\t" << gen << endl;
        }
        if (gen < pP->gen_max-1)
        {
            Initalize_Agent(x, y, theta, omega, u);
            Evalutate();
            sort(indiv.at(0).pol.begin(), indiv.at(0).pol.end(), Less_Than_Policy_Fitness());
            //Output_Pop_Data();
            Store_Best_Policy_Data();
            Down_Select();
            Replicate();
        }
        if (gen == pP->gen_max-1)
        {
            Initalize_Agent(x, y, theta, omega, u);
            Evalutate();
            sort(indiv.at(0).pol.begin(), indiv.at(0).pol.end(), Less_Than_Policy_Fitness());
            //Output_Pop_Data();
            Store_Best_Policy_Data();
        }
    }
    pP->HR_3 = 0;
    Write_txt_files();
    assert(indiv.at(0).pol.at(0).reached_goal == 1);
}


//-----------------------------------------------------------
//Runs HR_1
void EA::Run_HR_4()
{
    cout << "HR_4" << endl;
    for (int sr=0; sr<30; sr++)
    {
        int count = 0;
        pP->HR_4 = 1;
        Build_Pop();
        Initalize_Goal();
        for (int gen=0; gen<pP->gen_max; gen++)
        {
            Clear_Policy_Data();
            double x;
            double y;
            double theta;
            double omega=0;
            double u=0;
            x = (rand() / double(RAND_MAX))*(30);
            y = (rand() / double(RAND_MAX))*(pP->y_max);
            
            while (x>30 && x<70 && y<=30)
            {
                x = (rand() / double(RAND_MAX))*(pP->x_max);
            }
            if (x<=30 && y<=30)
            {
                theta = (rand() / double(RAND_MAX))*(Pi/2);
            }
            if (x<=30 && y>30 && y<50)
            {
                theta = (Pi/4)+(rand() / double(RAND_MAX))*(Pi/4);
                theta = (Pi/2);
            }
            if (x<=30 && y==50)
            {
                theta = (Pi/4)+(rand() / double(RAND_MAX))*(Pi/4);
                theta = (Pi/2);
            }
            if (x<=30 && y>50 && y<70)
            {
                theta = (Pi/2)+(rand() / double(RAND_MAX))*(Pi/4);
                theta = (Pi/2);
            }
            if (x<=30 && y>=70)
            {
                theta = (Pi/2)+((rand() / double(RAND_MAX))*(Pi/2));
            }
            if (x>30 && x<50 && y>=70)
            {
                theta = (3*Pi/4)+((rand() / double(RAND_MAX))*(Pi/4));
                theta = (Pi);
            }
            if (x==50 && y>=70)
            {
                theta = (3*Pi/4)+((rand() / double(RAND_MAX))*(Pi/4));
                theta = (Pi);
            }
            if (x>50 && x<70 && y>=70)
            {
                theta = (Pi)+((rand() / double(RAND_MAX))*(Pi/4));
                theta = (Pi);
            }
            if (x>=70 && y>=70)
            {
                theta = (Pi)+((rand() / double(RAND_MAX))*(Pi/2));
            }
            if (x>70 && y<70 && y>50)
            {
                theta = (5*Pi/4)+((rand() / double(RAND_MAX))*(Pi/4));
                theta = (3*Pi/2);
            }
            if (x>70 && y==50)
            {
                theta = (5*Pi/4)+((rand() / double(RAND_MAX))*(Pi/2));
                theta = (3*Pi/2);
            }
            if (x>70 && y<50 && y>30)
            {
                theta = (3*Pi/2)+((rand() / double(RAND_MAX))*(Pi/4));
                theta = (3*Pi/2);
            }
            if (x>=70 && y<=30)
            {
                theta = (3*Pi/2)+((rand() / double(RAND_MAX))*(Pi/2));
                if (theta == 2*Pi)
                {
                    theta = 0;
                }
                double check = theta*(180/Pi);
                check = theta*(180/Pi);
            }
            if (x<70 && x>50 && y<=30)
            {
                theta = (7*Pi/4)+((rand() / double(RAND_MAX))*(Pi/4));
                if (theta == 2*Pi)
                {
                    theta = 0;
                }
                theta = 0;
            }
            if (x==50 && y<=30)
            {
                theta = (7*Pi/4)+((rand() / double(RAND_MAX))*(Pi/2));
                theta = 0;
                if (theta >= 2*Pi)
                {
                    theta = theta-(2*Pi);
                }
                if (theta == 2*Pi)
                {
                    theta = 0;
                }
            }
            if (x<50 && x>30 && y<=30)
            {
                theta = ((rand() / double(RAND_MAX))*(Pi/4));
                theta = 0;
            }
            double check = theta*(180/Pi);
            check = theta*(180/Pi);
        for (int p=0; p<pP->pop_size; p++)
        {
            indiv.at(0).pol.at(p).x.push_back(x);
            indiv.at(0).pol.at(p).y.push_back(y);
            indiv.at(0).pol.at(p).theta.push_back(theta);
            indiv.at(0).pol.at(p).omega.push_back(omega);
            indiv.at(0).pol.at(p).u.push_back(u);
        }
            if (gen%10 ==0)
            {
                cout << "------------------" << endl;
                cout << "Generation" << "\t" << gen << endl;
            }
            if (gen < pP->gen_max-1)
            {
                Initalize_Agent(x, y, theta, omega, u);
                Evalutate();
                sort(indiv.at(0).pol.begin(), indiv.at(0).pol.end(), Less_Than_Policy_Fitness());
                //Output_Pop_Data();
                Down_Select();
                Replicate();
            }
            if (gen == pP->gen_max-1)
            {
                Initalize_Agent(x, y, theta, omega, u);
                Evalutate();
                sort(indiv.at(0).pol.begin(), indiv.at(0).pol.end(), Less_Than_Policy_Fitness());
                Output_Pop_Data();
                Store_Best_Policy_Data();
            }
            if(indiv.at(0).pol.at(0).reached_goal == 1)
            {
                count +=1;
            }
        }
        pP->HR_4 = 0;
        Write_txt_files();
        //assert(indiv.at(0).pol.at(0).reached_goal == 1);
        //assert(count>=.75*pP->gen_max);
    }
    //assert(count>=.75*30);
}


//-----------------------------------------------------------
//Runs the entire project
void EA::Run_Project_Delta()
{
    Delete_Text_Files();
    for (int sr=0; sr<30; sr++)
    {
        Run_HR_1(sr);
    }
    Run_HR_3();
    Run_HR_4();
}


#endif /* EA_hpp */
