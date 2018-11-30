#ifndef QLEARNING_H
#define QLEARNING_H

#include <iostream>
#include <iomanip>
#include <ctime>
#include <stdio.h>
#include <string.h>


class Qlearning
{
public:
    Qlearning();
    void run();
    void chooseAction(int initialState, int marbles, double time);
    int getRandomAction();
    void initialize();
    int maximum(int state, bool returnIndexOnly, bool temp);
    int reward(int action, int marbles, double time);
    void getReward(int action);
    void printR();
    void printroute();
    int currentState;
    int R[6][6] =  {{-1, 0, 0, 0, 0, 0},
                    {-1, -1, 0, 0, 0, 0},
                    {-1, 0, -1, 0, 0, 0},
                    {-1, 0, 0, -1, 0, 0},
                    {-1, 0, 0, 0, -1, 0},
                    {-1, 0, 0, 0, 0, -1}};

private:
    int initialStates[6] = {1, 3, 5, 2, 4, 0};
    //int Q[6][6];
    int possibleAction;

    const double gamma = 0.8;
    int newState;
    int rSize=6;
    int RTemp[6][6];
};

#endif // QLEARNING_H
