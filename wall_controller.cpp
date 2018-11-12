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
    inputVariable1->setLockValueInRange(false);
    inputVariable1->addTerm(new fl::Ramp("farrigth", -0.1500, -2.570));
    //inputVariable1->addTerm(new fl::Triangle("rigth", -0.500, -0.750, -1.000));
    inputVariable1->addTerm(new fl::Triangle("center", 0.1500, 0.000, -0.150));
    //inputVariable1->addTerm(new fl::Triangle("left", 1.000, 0.750, 0.500));
    inputVariable1->addTerm(new fl::Ramp("farleft", 0.1500, 2.570));
    //inputVariable1->addTerm(new fl::Ramp("error", -1.600, -5.000));
    wall_Engine->addInputVariable(inputVariable1);

    //Membership functions of input error
    fl::InputVariable* inputVariable2 = new fl::InputVariable;
    inputVariable2->setEnabled(true);
    inputVariable2->setName("WallDistance");
    inputVariable2->setRange(-1.600, 1.600);
    inputVariable2->setLockValueInRange(false);
    inputVariable2->addTerm(new fl::Ramp("close", 0.000, 3.000));
    inputVariable2->addTerm(new fl::Ramp("far", 4.000, 100.000));
    wall_Engine->addInputVariable(inputVariable2);


    //Membership functions of outputdirr
    fl::OutputVariable* outputVariable1 = new fl::OutputVariable;
    outputVariable1->setEnabled(true);
    outputVariable1->setName("direction");
    outputVariable1->setRange(-1.57, 1.57);
    outputVariable1->setLockValueInRange(false);
    outputVariable1->setDefuzzifier(new fl::Centroid(100));
    outputVariable1->setAggregation(new fl::Maximum);
    outputVariable1->setDefaultValue(0);
    //outputVariable1->addTerm(new fl::Triangle("serror", -0.100, 0.000, 0.100));
    outputVariable1->addTerm(new fl::Ramp("ssharprigth", 0.400, 1.570));
    //outputVariable1->addTerm(new fl::Triangle("srigth",1.000, 0.600, 0.200 ));
    outputVariable1->addTerm(new fl::Triangle("sstraight", 0.100, -0.000, -0.100));
    //outputVariable1->addTerm(new fl::Triangle("sleft", -0.200, -0.600, -1.000));
    outputVariable1->addTerm(new fl::Ramp("ssharpleft", -0.400, -1.570));
    wall_Engine->addOutputVariable(outputVariable1);

    //Membership functions of outputspeed
    fl::OutputVariable* outputVariable2 = new fl::OutputVariable;
    outputVariable2->setEnabled(true);
    outputVariable2->setName("speed");
    outputVariable2->setRange(-1, 1);
    outputVariable2->setLockValueInRange(false);
    outputVariable2->setAggregation(new fl::Maximum);
    outputVariable2->setDefaultValue(fl::nan);
    outputVariable2->setDefuzzifier(new fl::Centroid(100));
    outputVariable2->addTerm(new fl::Ramp("forward", 0.050, 1.000));
    outputVariable2->addTerm(new fl::Ramp("stop", 0.020, -0.020));
    wall_Engine->addOutputVariable(outputVariable2);

    //Rules
    fl::RuleBlock* mamdani = new fl::RuleBlock;
    mamdani->setName("mamdani");
    mamdani->setEnabled(true);
    mamdani->setConjunction(new fl::Minimum);
    mamdani->setDisjunction(new fl::Maximum);
    mamdani->setImplication(new fl::Minimum);
    mamdani->setActivation(new fl::General);
    mamdani->addRule(fl::Rule::parse("if WallDirection is center then speed is stop", wall_Engine));
    mamdani->addRule(fl::Rule::parse("if WallDirection is farleft then direction is ssharprigth", wall_Engine));
    //mamdani->addRule(fl::Rule::parse("if WallDirection is rigth then direction is sleft", wall_Engine));
    mamdani->addRule(fl::Rule::parse("if WallDirection is farrigth then direction is ssharpleft", wall_Engine));
    //mamdani->addRule(fl::Rule::parse("if WallDirection is rigth then direction is srigth",wall_Engine));
    //mamdani->addRule(fl::Rule::parse("if WallDirection is error then direction is serror", wall_Engine));
    mamdani->addRule(fl::Rule::parse("if WallDistance is far then speed is forward", wall_Engine));
    wall_Engine->addRuleBlock(mamdani);

    std::string status;
    if (not wall_Engine->isReady(&status))
        throw fl::Exception("[engine error] engine is not ready:\n" + status, FL_AT);


    //Set in/outputs
    wall_Direction     = wall_Engine->getInputVariable("WallDirection");
    wall_Distance      = wall_Engine->getInputVariable("WallDistance");
    steer_Direction    = wall_Engine->getOutputVariable("direction");
    speed              = wall_Engine->getOutputVariable("speed");
}


WallControlOutput wall_Controller::getControlOutput(float cent, float dist)
{

    wall_Direction->setValue(cent);
    wall_Distance->setValue(dist);

    std::cout << "hej:" << cent << std::endl;

    wall_Engine->process();

    WallControlOutput out;

    out.direction = ((int)(wall_Direction->getValue() * 100 + .5) / 100.0);
    out.speed     = speed->getValue();

    std::cout << "output:" << out.direction << std::endl;


    return out;
}
