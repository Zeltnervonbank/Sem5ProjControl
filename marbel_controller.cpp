#include "marbel_controller.h"

MarbleController::MarbleController()
{

}

void MarbleController::buildController()
{

    //Create fuzzy
    marbleEngine = new fl::Engine;
    marbleEngine->setName("FindMarble");
    marbleEngine->setDescription("");

    //Membership functions of input error
    fl::InputVariable* inputVariable1 = new fl::InputVariable;
    inputVariable1->setEnabled(true);
    inputVariable1->setName("marbleDirection");
    inputVariable1->setRange(-1.570, 1.570);
    inputVariable1->setLockValueInRange(false);
    inputVariable1->addTerm(new fl::Ramp("rigth", -0.100, -1.570));
    //inputVariable1->addTerm(new fl::Triangle("rigth", -0.000, -0.300, -0.935));
    inputVariable1->addTerm(new fl::Trapezoid("center", 0.200, 0.100, -0.100, -0.200));
    //inputVariable1->addTerm(new fl::Triangle("left", 0.935, 0.300, 0.000));
    inputVariable1->addTerm(new fl::Ramp("left", 0.100, 1.570));
    //inputVariable1->addTerm(new fl::Ramp("error", -1.600, -5.000));
    marbleEngine->addInputVariable(inputVariable1);

    //Membership functions of input error
    fl::InputVariable* inputVariable2 = new fl::InputVariable;
    inputVariable2->setEnabled(true);
    inputVariable2->setName("marbleDistance");
    inputVariable2->setRange(0, 200);
    inputVariable2->setLockValueInRange(false);
    inputVariable2->addTerm(new fl::Ramp("far", 20.000, 200.000));
    inputVariable2->addTerm(new fl::Triangle("close",0.000,20.000, 40.000));
    marbleEngine->addInputVariable(inputVariable2);


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
    outputVariable1->addTerm(new fl::Ramp("ssharprigth", 0.700, 1.570));
    outputVariable1->addTerm(new fl::Triangle("srigth",0.000, 0.700, 1.200));
    outputVariable1->addTerm(new fl::Triangle("sstraight", 0.010, 0.000, -0.010));
    outputVariable1->addTerm(new fl::Triangle("sleft", -1.200, -0.700,0.000 ));
    outputVariable1->addTerm(new fl::Ramp("ssharpleft", -0.700, -1.570));
    marbleEngine->addOutputVariable(outputVariable1);

    //Membership functions of outputspeed
    fl::OutputVariable* outputVariable2 = new fl::OutputVariable;
    outputVariable2->setEnabled(true);
    outputVariable2->setName("speed");
    outputVariable2->setRange(0, 1);
    outputVariable2->setLockValueInRange(false);
    outputVariable2->setAggregation(new fl::Maximum);
    outputVariable2->setDefaultValue(0);
    outputVariable2->setDefuzzifier(new fl::Centroid(100));
    outputVariable2->addTerm(new fl::Ramp("fast", 0.400, 1.000));
    outputVariable2->addTerm(new fl::Triangle("slow",0.300, 0.400, 0.500));
    marbleEngine->addOutputVariable(outputVariable2);

    //Rules
    fl::RuleBlock* mamdani = new fl::RuleBlock;
    mamdani->setName("mamdani");
    mamdani->setEnabled(true);
    mamdani->setConjunction(new fl::Minimum);
    mamdani->setDisjunction(new fl::Maximum);
    mamdani->setImplication(new fl::Minimum);
    mamdani->setActivation(new fl::General);
    mamdani->addRule(fl::Rule::parse("if marbleDirection is center then direction is sstraight", marbleEngine));
    mamdani->addRule(fl::Rule::parse("if marbleDirection is rigth and marbleDistance is far then direction is srigth", marbleEngine));
    mamdani->addRule(fl::Rule::parse("if marbleDirection is rigth and marbleDistance is close then direction is ssharprigth", marbleEngine));
    mamdani->addRule(fl::Rule::parse("if marbleDirection is left and marbleDistance is far then direction is sleft", marbleEngine));
    mamdani->addRule(fl::Rule::parse("if marbleDirection is left and marbleDistance is close then direction is ssharpleft", marbleEngine));
    mamdani->addRule(fl::Rule::parse("if marbleDistance is close then speed is slow", marbleEngine));
    mamdani->addRule(fl::Rule::parse("if marbleDistance is far then speed is fast", marbleEngine));
    //mamdani->addRule(fl::Rule::parse("if marbleDirection is left then direction is srigth", marbleEngine));
    //mamdani->addRule(fl::Rule::parse("if marbleDirection is rigth then direction is sleft",marbleEngine));
    //mamdani->addRule(fl::Rule::parse("if marbleDirection is error then direction is serror", marbleEngine));
    marbleEngine->addRuleBlock(mamdani);

    std::string status;
    if (not marbleEngine->isReady(&status))
        throw fl::Exception("[engine error] engine is not ready:\n" + status, FL_AT);


    //Set in/outputs
    marbleDirection = marbleEngine->getInputVariable("marbleDirection");
    marbleDistance  = marbleEngine->getInputVariable("marbleDistance");
    SteerDirection    = marbleEngine->getOutputVariable("direction");
    Speed             = marbleEngine->getOutputVariable("speed");
}


ControlOutput MarbleController::getControlOutput(float cent, float dist)
{

    marbleDirection->setValue(cent);
    marbleDistance->setValue(dist);

    //std::cout << "cent:" << cent << std::endl;
    //std::cout << "dist:" << dist << std::endl;

    marbleEngine->process();

    ControlOutput out;



    out.direction = ((int)(SteerDirection->getValue() * 100 + .5) / 100.0);
    out.speed     = Speed->getValue();

    //std::cout << "output:" << out.direction << std::endl;
    //std::cout << "speed:" << out.speed << std::endl;


    return out;
}
