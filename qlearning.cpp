#include "qlearning.h"

Qlearning::Qlearning()
{

}

Qlearning::~Qlearning()
{

}

void Qlearning::Initialize()
{
    srand((unsigned)time(NULL));

    Qlearning::ReadFromFile();//read R from external file
    Qlearning::PrintR();

    for(int i = 0; i <= (rSize - 1); i++)
    {
        for(int j = 0; j <= (rSize - 1); j++)
        {
            RTemp[i][j] = R[i][j];
        }
    }

    /*//Get matrix from file from former experiments - Reset state for now.
    for(int i = 0; i <= (rSize - 1); i++){
        for(int j = 0; j <= (rSize - 1); j++){
            //Q[i][j] = 0;
        }
    }*/
}

void Qlearning::Run()
{
    //Reset the temp array to a new run.
    for(int i = 0; i <= (rSize - 1); i++)
    {
        for(int j = 0; j <= (rSize - 1); j++)
        {
            RTemp[i][j] = R[i][j];
        }
    }
}

int Qlearning::GetRandomAction()
{
    int action;
    bool choiceIsValid = false;
    //printRTemp();

    do
    {
        //Get a random value between 1 and rSize.
        action = (rand() % rSize - 1) + 1;
        //std::cout << action << std::endl;
        //std::cout << "her" << std::endl;
        if(RTemp[currentState][action] > - 1)
        {
            choiceIsValid = true;
        }
    }
    while(!choiceIsValid);

    return action;
}

int Qlearning::GetBestAction(){

    int action = Maximum(currentState, true);
    return action;
}

int Qlearning::ChooseAction(int currstate)
{
    currentState = currstate;

    int var = (rand() % 100);
    std::cout << var << std::endl;

    if(var<=50)
    {
        possibleAction = GetRandomAction();
    }
    else
    {
       possibleAction = GetBestAction();
    }

    // Fill temp matrix with "-1" to avoid selecting it again.
    for(int i = 0; i <= (rSize - 1); i++)
    {
        RTemp[currentState][i] = -1;
    }

    /*if(R[currentState][possibleAction] >= 0)
    {
        //std::cout << reward(possibleAction,marbles,itterations) << std::endl;
        R[currentState][possibleAction] = Reward(marbles, time); //Update reward of choosen action.

        Qlearning::WriteToFile(); //update external file with rewards
        //std::cout << reward(possibleAction) << std::endl;
        currentState = possibleAction;
        std::cout << "currentState:" << currentState << std::endl;
    }
    */
    return possibleAction;
}

int Qlearning::Maximum(int state, bool returnIndexOnly)
{
// if returnIndexOnly = true, a Q matrix index is returned.
// if returnIndexOnly = false, a Q matrix element is returned.

    int winner;
    bool foundNewWinner;
    bool done = false;

    winner = 0;
    do
    {
        foundNewWinner = false;
        for(int i = 1; i <= (rSize - 1); i++)
        {
            if((i < winner) || (i > winner))
            {     //Avoid self-comparison.
                if(RTemp[state][i] > RTemp[state][winner])
                {
                    winner = i;
                    foundNewWinner = true;
                }
            }
        }
        if(!foundNewWinner){
            done = true;
        }
    }
    while(!done);

    //std::cout << "R" << R[state][winner] << std::endl;

    if(returnIndexOnly)
    {
        return winner;
    }
    else
    {
        return R[state][winner];
    }
}

int Qlearning::Reward(int marbles, double time)
{
    //Get reward
    //std::cout << "her1" << std::endl;
    //std::cout << gamma * R[currentState][possibleAction] << std::endl;
    int rewardV = marbles / time;
    std::cout << "reward" << rewardV << std::endl;
    return static_cast<int>(R[currentState][possibleAction]+gamma*(rewardV + Qlearning::Maximum(possibleAction,false) - R[currentState][possibleAction]));
}

void Qlearning::PrintR(){
    //Print out R matrix.
    for(int i = 0; i <= (rSize - 1); i++)
    {
        for(int j = 0; j <= (rSize - 1); j++)
        {
            std::cout << std::setw(5) << R[i][j];
            if(j < rSize - 1)
            {
                std::cout << ",";
            }
        } // j
        std::cout << "\n";
    } // i
    std::cout << "\n";
}

void Qlearning::PrintRoute()
{ //Print the most optimal route, based on past experience.
    for(int i = 0; i <= (rSize - 1); i++)
    {
        for(int j = 0; j <= (rSize - 1); j++)
        {
            RTemp[i][j] = R[i][j];
        }
    }
    currentState = initialStates[0];
    std::cout << "Optimal path:  " << std::endl;
    for(int i = 0; i < rSize; i++)
    {
        //printRTemp();
        //std::cout << "hej" << std::endl;
        newState = Maximum(currentState, true);
        for(int i = 0; i <= (rSize - 1); i++){
                RTemp[i][newState] = -1;
        }
        currentState = newState;
        std::cout << currentState << ", ";
    }
    std::cout << std::endl;

}

void Qlearning::WriteToFile()
{
    std::ofstream f("matrix.txt");
    for (int i = 0; i < rSize; i++)
    {
        for (int j = 0; j < rSize; j++)
        {
            f << R[i][j] << " ";
        }
    }
    f.close();
}

void Qlearning::ReadFromFile()
{
    std::ifstream f("matrix.txt");

    for (int i = 0; i < rSize; i++)
    {
        for (int j = 0; j < rSize; j++)
        {
            f >> R[i][j];
        }
    }
}

void Qlearning::UpdateReward(int currS, int prevS, int point, double time)
{
    //Get reward
    //std::cout << "her1" << std::endl;
    //std::cout << gamma * R[currentState][possibleAction] << std::endl;
    int rewardV=point/time;
    std::cout << "reward" << rewardV << std::endl;
    R[prevS][currS]=R[prevS][currS]+alpha*(rewardV + gamma*(Qlearning::Maximum(currS,false)) - R[prevS][currS]);

    for(int i = 0; i <= (rSize - 1); i++){
            RTemp[i][currS] = -1;
    }
    Qlearning::WriteToFile();
}



























