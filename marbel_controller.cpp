#include "marbel_controller.h"

marbel_Controller::marbel_Controller()
{

}

void marbel_Controller::buildController()
{

    //Create fuzzy
    marbleEngine = new fl::Engine;
    marbleEngine->setName("FindMarble");
    marbleEngine->setDescription("");

    //Membership functions of input error
    fl::InputVariable* inputVariable1 = new fl::InputVariable;
    inputVariable1->setEnabled(true);
    inputVariable1->setName("marbleDirection");
    inputVariable1->setRange(-3.14, 3.14);
    inputVariable1->setLockValueInRange(true);
    inputVariable1->addTerm(new fl::Ramp("farleft", 0.030, 0.150));
    inputVariable1->addTerm(new fl::Triangle("left", 0.150, 0.03, 0.000));
    inputVariable1->addTerm(new fl::Triangle("center", -0.03, -0.000, 0.03));
    inputVariable1->addTerm(new fl::Triangle("rigth", -0.000, -0.03, -0.150));
    inputVariable1->addTerm(new fl::Ramp("farrigth", -0.03, -0.150));
    marbleEngine->addInputVariable(inputVariable1);

    //Membership functions of input error
    fl::InputVariable* inputVariable2 = new fl::InputVariable;
    inputVariable2->setEnabled(true);
    inputVariable2->setName("marbleDistance");
    inputVariable2->setRange(0, 200);
    inputVariable2->setLockValueInRange(true);
    inputVariable2->addTerm(new fl::Ramp("far", 5.000, 10.000));
    inputVariable2->addTerm(new fl::Trapezoid("close",0.1, 4.0, 6.0, 10.0));
    inputVariable2->addTerm(new fl::Ramp("veryclose", 4.0, 0.1));
    marbleEngine->addInputVariable(inputVariable2);


    //Membership functions of outputdirr
    fl::OutputVariable* outputVariable1 = new fl::OutputVariable;
    outputVariable1->setEnabled(true);
    outputVariable1->setName("direction");
    outputVariable1->setRange(-1.57, 1.57);
    outputVariable1->setLockValueInRange(true);
    outputVariable1->setDefuzzifier(new fl::Centroid(100));
    outputVariable1->setAggregation(new fl::Maximum);
    outputVariable1->setDefaultValue(fl::nan);
    //outputVariable1->addTerm(new fl::Triangle("serror", -0.100, 0.000, 0.100));
    outputVariable1->addTerm(new fl::Ramp("ssharpleft", 0.3, 0.6));
    outputVariable1->addTerm(new fl::Trapezoid("sleft",0.0, 0.05, 0.3, 0.6 ));
    outputVariable1->addTerm(new fl::Triangle("sstraight", -0.05, 0.00, 0.05));
    outputVariable1->addTerm(new fl::Trapezoid("srigth", -0.6, -0.3, -0.05, 0.0));
    outputVariable1->addTerm(new fl::Ramp("ssharprigth", -0.3, -0.6));
    marbleEngine->addOutputVariable(outputVariable1);


    //Membership functions of outputspeed
    fl::OutputVariable* outputVariable2 = new fl::OutputVariable;
    outputVariable2->setEnabled(true);
    outputVariable2->setName("speed");
    outputVariable2->setRange(0, 1);
    outputVariable2->setLockValueInRange(true);
    outputVariable2->setAggregation(new fl::Maximum);
    outputVariable2->setDefaultValue(fl::nan);
    outputVariable2->setDefuzzifier(new fl::Centroid(100));
    outputVariable2->addTerm(new fl::Ramp("fast", 0.300, 0.500));
    outputVariable2->addTerm(new fl::Trapezoid("slow",0.01, 0.05, 0.3, 0.5));
    outputVariable2->addTerm(new fl::Ramp("stop", 0.05, 0.01));
    marbleEngine->addOutputVariable(outputVariable2);

    //Rules
    fl::RuleBlock* mamdani = new fl::RuleBlock;
    mamdani->setName("mamdani");
    mamdani->setEnabled(true);
    mamdani->setConjunction(new fl::Minimum);
    mamdani->setDisjunction(new fl::Maximum);
    mamdani->setImplication(new fl::Minimum);
    mamdani->setActivation(new fl::General);
    mamdani->addRule(fl::Rule::parse("if marbleDirection is center then direction is sstraight and speed is fast", marbleEngine));
    mamdani->addRule(fl::Rule::parse("if marbleDirection is rigth and marbleDistance is far then direction is ssharprigth and speed is fast", marbleEngine));
    mamdani->addRule(fl::Rule::parse("if marbleDirection is rigth and marbleDistance is close then direction is ssharprigth and speed is slow", marbleEngine));
    mamdani->addRule(fl::Rule::parse("if marbleDirection is rigth and marbleDistance is veryclose then direction is ssharprigth and speed is stop", marbleEngine));
    mamdani->addRule(fl::Rule::parse("if marbleDirection is left and marbleDistance is far then direction is ssharpleft and speed is fast", marbleEngine));
    mamdani->addRule(fl::Rule::parse("if marbleDirection is left and marbleDistance is close then direction is ssharpleft and speed is slow", marbleEngine));
    mamdani->addRule(fl::Rule::parse("if marbleDirection is left and marbleDistance is veryclose then direction is ssharpleft and speed is stop", marbleEngine));
    mamdani->addRule(fl::Rule::parse("if marbleDirection is farleft then direction is ssharprigth and speed is stop", marbleEngine));
    mamdani->addRule(fl::Rule::parse("if marbleDirection is farrigth then direction is ssharpleft and speed is stop",marbleEngine));
    //mamdani->addRule(fl::Rule::parse("if marbleDirection is farleft then speed is stop", marbleEngine));
    //mamdani->addRule(fl::Rule::parse("if marbleDirection is farrigth then speed is stop",marbleEngine));
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


ControlOutput marbel_Controller::getControlOutput(float cent, float dist)
{

    marbleDirection->setValue(cent);
    marbleDistance->setValue(dist);

    //std::cout << "cent:" << cent << std::endl;
    //std::cout << "dist:" << dist << std::endl;

    marbleEngine->process();

    ControlOutput out;



    out.direction = ((int)(SteerDirection->getValue() * 10 + .5) / 100.0);
    out.speed     = Speed->getValue();

    //std::cout << "output:" << out.direction << std::endl;
    //std::cout << "speed:" << out.speed << std::endl;


    return out;
}
