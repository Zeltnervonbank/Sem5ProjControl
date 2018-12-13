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
    int currentState;

    Qlearning();
    ~Qlearning();
    void run();
    void chooseAction(int initialState, int marbles, double time);
    int getRandomAction();
    void initialize();
    int maximum(int state, bool returnIndexOnly);
    int reward(int action, int marbles, double time);
    void getReward(int action);
    void printR();
    void printroute();
    void writeToFile();
    void readFromFile();
    void updateReward(int currS, int prevS, int point, double time);


private:
    int R[23][23];
    int initialStates[23] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 10, 20, 21, 22};
    int possibleAction;
    int rSize = 23;

    const double gamma = 0.8;
    int newState = 0;
    int RTemp[23][23];
    int Ropt[23][23];
};

#endif // QLEARNING_H
