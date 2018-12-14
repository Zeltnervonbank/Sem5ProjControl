#ifndef QLEARNING_H
#define QLEARNING_H

#include <iostream>
#include <iomanip>
#include <ctime>
#include <stdio.h>
#include <string.h>
#include <fstream>
#include "globals.h"


class Qlearning
{
public:
    int currentState;

    Qlearning();
    ~Qlearning();
    void Run();
    int ChooseAction(int initialState);
    int GetRandomAction();
    int GetBestAction();
    void Initialize();
    int Maximum(int state, bool returnIndexOnly);
    int Reward(int marbles, double time);
    void GetReward(int action);
    void PrintR();
    void PrintRoute();
    void WriteToFile();
    void ReadFromFile();
    void UpdateReward(int currS, int prevS, int point, double time);


private:
    int R[12][12];
    int initialStates[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    int possibleAction;
    int rSize = 12;

    const double gamma = 0.8;
    const double alpha = 0.5;
    int newState = 0;
    int RTemp[12][12];

};

#endif // QLEARNING_H
