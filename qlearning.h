#ifndef QLEARNING_H
#define QLEARNING_H

#include <iostream>
#include <iomanip>
#include <ctime>
#include <stdio.h>
#include <string.h>
#include <fstream>


class Qlearning
{
public:
    Qlearning();
    ~Qlearning();
    void run();
    void chooseAction(int initialState, int marbles, double time);
    int getRandomAction();
    void initialize();
    int maximum(int state, bool returnIndexOnly, bool temp);
    int reward(int action, int marbles, double time);
    void getReward(int action);
    void printR();
    void printroute();
    void writeToFile();
    void readFromFile();

    int currentState;

private:
    int R[6][6];
    int initialStates[6] = {0,1,2,3,4,5};
    int possibleAction;

    const double gamma = 0.8;
    int newState=0;
    int rSize=6;
    int RTemp[6][6];
    int Ropt[6][6];
};

#endif // QLEARNING_H
