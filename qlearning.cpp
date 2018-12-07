#include "qlearning.h"

Qlearning::Qlearning()
{

}

Qlearning::~Qlearning()
{

}

void Qlearning::initialize(){

    srand((unsigned)time(NULL));

    Qlearning::readFromFile();//read R from external file
    Qlearning::printR();
    for(int i = 0; i <= (rSize - 1); i++){
        for(int j = 0; j <= (rSize - 1); j++){
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

void Qlearning::run(){
//Reset the temp array to a new run.
    for(int i = 0; i <= (rSize - 1); i++){
        for(int j = 0; j <= (rSize - 1); j++){
            RTemp[i][j] = R[i][j];
        }
    }

}

int Qlearning::getRandomAction(){
    int action;
    bool choiceIsValid = false;
    //printRtemp();

    do {
        //Get a random value between 1 and rSize.
        action = (rand()%rSize-1)+1;
        //std::cout << action << std::endl;
        //std::cout << "her" << std::endl;
        if(RTemp[currentState][action] > -1){
            choiceIsValid = true;
        }
    } while(choiceIsValid == false);
    for(int i = 0; i <= (rSize - 1); i++){ //fill temp matrix with "-1" to avoid selectning it again.
            RTemp[currentState][i] = -1;
        }
    return action;
}

void Qlearning::chooseAction(int initialState, int marbles, double time){

    currentState=initialState;

    possibleAction = getRandomAction();

    if(R[currentState][possibleAction] >= 0){
        //std::cout << reward(possibleAction,marbles,itterations) << std::endl;
        R[currentState][possibleAction] = reward(possibleAction,marbles,time); //Update reward of choosen action.
        Qlearning::writeToFile(); //update external file with rewards
        //std::cout << reward(possibleAction) << std::endl;
        currentState = possibleAction;
        std::cout << "currentState:" << currentState << std::endl;
    }
}

int Qlearning::maximum(int state, bool returnIndexOnly, bool temp){
// if returnIndexOnly = true, a Q matrix index is returned.
// if returnIndexOnly = false, a Q matrix element is returned.

    int winner;
    bool foundNewWinner;
    bool done = false;

    winner = 0;
if(!temp){
    do {
        foundNewWinner = false;
        for(int i = 0; i <= (rSize - 1); i++){
            if((i < winner) || (i > winner)){     //Avoid self-comparison.
                if(R[state][i] > R[state][winner]){
                    winner = i;
                    foundNewWinner = true;
                }
            }
        } // i

        if(foundNewWinner == false){
            done = true;
        }

    } while(done == false);
}
else{
    do {
        foundNewWinner = false;
        for(int i = 0; i <= (rSize - 1); i++){
            if((i < winner) || (i > winner)){     //Avoid self-comparison.
                if(Ropt[state][i] > Ropt[state][winner]){
                    winner = i;
                    foundNewWinner = true;
                }
            }
        } // i

        if(foundNewWinner == false){
            done = true;
        }

    } while(done = false);
}
    //std::cout << "R" << R[state][winner] << std::endl;

    if(returnIndexOnly == true){
        return winner;
    }else{
        return R[state][winner];
    }
}

int Qlearning::reward(int action, int marbles, double time){
    //Get reward
    //std::cout << "her1" << std::endl;
    //std::cout << gamma * R[currentState][possibleAction] << std::endl;
    int rewardV=marbles/time;
    std::cout << "reward" << rewardV << std::endl;
    return static_cast<int>((rewardV + (gamma * R[currentState][possibleAction]))/2);
}

void Qlearning::printR(){
    //Print out R matrix.
    for(int i = 0; i <= (rSize - 1); i++){
        for(int j = 0; j <= (rSize - 1); j++){
            std::cout << std::setw(5) << R[i][j];
            if(j < rSize - 1){
                std::cout << ",";
            }
        } // j
        std::cout << "\n";
    } // i
    std::cout << "\n";
}

void Qlearning::printroute(){ //Print the most optimal route, based on past experience.
    for(int i = 0; i <= (rSize - 1); i++){
        for(int j = 0; j <= (rSize - 1); j++){
            Ropt[i][j] = R[i][j];
        }
    }

    currentState=initialStates[0];
    std::cout << "Optimal path:  " << std::endl;
    for(int i=0; i<rSize;i++){
        //printRtemp();
        //std::cout << "hej" << std::endl;
        newState = maximum(currentState, true,true);
        for(int i = 0; i <= (rSize - 1); i++){
                Ropt[i][newState] = -1;
            }
        currentState = newState;
        std::cout << currentState << ", ";
    }
    std::cout << std::endl;

}

void Qlearning::writeToFile(){

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

void Qlearning::readFromFile(){
    std::ifstream f("matrix.txt");

    for (int i = 0; i < rSize; i++){
    for (int j = 0; j < rSize; j++){
      f >> R[i][j];
    }
    }
}



























