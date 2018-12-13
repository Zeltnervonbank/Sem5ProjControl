#include "wall_controller.h"

wall_Controller::wall_Controller()
{

}


void wall_Controller::buildController()
{
    //Create fuzzy
    wall_Engine = new fl::Engine;
    wall_Engine->setName("AvoidWall");
    wall_Engine->setDescription("");

    //Membership functions of input error
    fl::InputVariable* inputVariable1 = new fl::InputVariable;
    inputVariable1->setEnabled(true);
    inputVariable1->setName("WallDirection");
    inputVariable1->setRange(-2.600, 2.600);
    inputVariable1->setLockValueInRange(true);
    inputVariable1->addTerm(new fl::Ramp("farrigth", -0.50, -1.00));
    inputVariable1->addTerm(new fl::Triangle("rigth", 1.00, -0.50, -0.0));
    inputVariable1->addTerm(new fl::Triangle("center", -0.500, 0.00, 0.500));
    inputVariable1->addTerm(new fl::Triangle("left", 0.00, 0.50, 1.00));
    inputVariable1->addTerm(new fl::Ramp("farleft", 0.50, 1.00));
    //inputVariable1->addTerm(new fl::Ramp("error", -1.600, -5.00));
    wall_Engine->addInputVariable(inputVariable1);

    //Membership functions of outputdirr
    fl::OutputVariable* outputVariable1 = new fl::OutputVariable;
    outputVariable1->setEnabled(true);
    outputVariable1->setName("direction");
    outputVariable1->setRange(-1.57, 1.57);
    outputVariable1->setLockValueInRange(true);
    outputVariable1->setDefuzzifier(new fl::Centroid(100));
    outputVariable1->setAggregation(new fl::Maximum);
    outputVariable1->setDefaultValue(fl::nan);
    //outputVariable1->addTerm(new fl::Triangle("serror", -0.100, 0.00, 0.100));
    outputVariable1->addTerm(new fl::Ramp("ssharprigth", 0.300, 0.600));
    outputVariable1->addTerm(new fl::Trapezoid("srigth",-0.05, 0.05, 0.3, 0.6));
    //outputVariable1->addTerm(new fl::Triangle("sstraight", -0.05, 0, 0.05));
    outputVariable1->addTerm(new fl::Trapezoid("sleft", -0.6, -0.3, -0.05, 0.05));
    outputVariable1->addTerm(new fl::Ramp("ssharpleft", -0.30, -0.60));
    wall_Engine->addOutputVariable(outputVariable1);

    /*
    //Membership functions of outputspeed
    fl::OutputVariable* outputVariable2 = new fl::OutputVariable;
    outputVariable2->setEnabled(true);
    outputVariable2->setName("speed");
    outputVariable2->setRange(-1, 1);
    outputVariable2->setLockValueInRange(true);
    outputVariable2->setAggregation(new fl::Maximum);
    outputVariable2->setDefaultValue(fl::nan);
    outputVariable2->setDefuzzifier(new fl::Centroid(100));
    outputVariable2->addTerm(new fl::Ramp("forward", 0.00, 0.020));
    outputVariable2->addTerm(new fl::Ramp("stop", 0.020, 0.00));
    wall_Engine->addOutputVariable(outputVariable2);
    */

    //Rules
    fl::RuleBlock* mamdani = new fl::RuleBlock;
    mamdani->setName("mamdani");
    mamdani->setEnabled(true);
    mamdani->setConjunction(new fl::Minimum);
    mamdani->setDisjunction(new fl::Maximum);
    mamdani->setImplication(new fl::Minimum);
    mamdani->setActivation(new fl::General);
    mamdani->addRule(fl::Rule::parse("if WallDirection is center then direction is ssharpleft", wall_Engine));
    mamdani->addRule(fl::Rule::parse("if WallDirection is farleft then direction is srigth", wall_Engine));
    mamdani->addRule(fl::Rule::parse("if WallDirection is left then direction is ssharprigth",wall_Engine));
    mamdani->addRule(fl::Rule::parse("if WallDirection is farrigth then direction is sleft", wall_Engine));
    mamdani->addRule(fl::Rule::parse("if WallDirection is rigth then direction is ssharpleft", wall_Engine));
    //mamdani->addRule(fl::Rule::parse("if WallDirection is error then direction is serror", wall_Engine));

    //mamdani->addRule(fl::Rule::parse("if WallDistance is close and WallDirection is center then direction is ssharpleft", wall_Engine));
    wall_Engine->addRuleBlock(mamdani);

    std::string status;
    if (not wall_Engine->isReady(&status))
        throw fl::Exception("[engine error] engine is not ready:\n" + status, FL_AT);


    //Set in/outputs
    wall_Direction     = wall_Engine->getInputVariable("WallDirection");
    steer_Direction    = wall_Engine->getOutputVariable("direction");

}


ControlOutput wall_Controller::getControlOutput(float cent, float dist)
{

    wall_Direction->setValue(cent);

    //std::cout << "cent:" << cent << std::endl;

    wall_Engine->process();

    ControlOutput out;

    out.direction = ((int)(steer_Direction->getValue() * 100 + .5) / 100.0);
    out.speed     = -0.1;

    //std::cout << "dirr:" << out.direction << std::endl;
    //std::cout << "speed:" << out.speed << std::endl;


    return out;
}
