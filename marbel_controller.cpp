#include "marbel_controller.h"

marbel_Controller::marbel_Controller()
{

}

void marbel_Controller::buildController()
{

    //Create fuzzy
    m_pcFLEngine = new fl::Engine;
    m_pcFLEngine->setName("FindBall");
    m_pcFLEngine->setDescription("");

    //Membership functions of input error
    fl::InputVariable* inputVariable1 = new fl::InputVariable;
    inputVariable1->setEnabled(true);
    inputVariable1->setName("BallDirection");
    inputVariable1->setRange(-1.600, 1.600);
    inputVariable1->setLockValueInRange(false);
    inputVariable1->addTerm(new fl::Ramp("farrigth", -0.1500, -1.570));
    //inputVariable1->addTerm(new fl::Triangle("rigth", -0.500, -0.750, -1.000));
    inputVariable1->addTerm(new fl::Triangle("center", 0.1500, 0.000, -0.150));
    //inputVariable1->addTerm(new fl::Triangle("left", 1.000, 0.750, 0.500));
    inputVariable1->addTerm(new fl::Ramp("farleft", 0.1500, 1.570));
    //inputVariable1->addTerm(new fl::Ramp("error", -1.600, -5.000));
    m_pcFLEngine->addInputVariable(inputVariable1);

    //Membership functions of input error
    fl::InputVariable* inputVariable2 = new fl::InputVariable;
    inputVariable2->setEnabled(true);
    inputVariable2->setName("BallDistance");
    inputVariable2->setRange(-1.600, 1.600);
    inputVariable2->setLockValueInRange(false);
    inputVariable2->addTerm(new fl::Ramp("close", 10.000, 100.000));
    m_pcFLEngine->addInputVariable(inputVariable2);


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
    m_pcFLEngine->addOutputVariable(outputVariable1);

    //Membership functions of outputspeed
    fl::OutputVariable* outputVariable2 = new fl::OutputVariable;
    outputVariable2->setEnabled(true);
    outputVariable2->setName("speed");
    outputVariable2->setRange(-1, 1);
    outputVariable2->setLockValueInRange(false);
    outputVariable2->setAggregation(new fl::Maximum);
    outputVariable2->setDefaultValue(0);
    outputVariable2->setDefuzzifier(new fl::Centroid(100));
    outputVariable2->addTerm(new fl::Ramp("forward", 0.050, 1.000));
    m_pcFLEngine->addOutputVariable(outputVariable2);

    //Rules
    fl::RuleBlock* mamdani = new fl::RuleBlock;
    mamdani->setName("mamdani");
    mamdani->setEnabled(true);
    mamdani->setConjunction(new fl::Minimum);
    mamdani->setDisjunction(new fl::Maximum);
    mamdani->setImplication(new fl::Minimum);
    mamdani->setActivation(new fl::General);
    mamdani->addRule(fl::Rule::parse("if BallDirection is center then direction is sstraight", m_pcFLEngine));
    mamdani->addRule(fl::Rule::parse("if BallDirection is farrigth then direction is ssharprigth", m_pcFLEngine));
    //mamdani->addRule(fl::Rule::parse("if BallDirection is left then direction is sleft", m_pcFLEngine));
    mamdani->addRule(fl::Rule::parse("if BallDirection is farleft then direction is ssharpleft", m_pcFLEngine));
    //mamdani->addRule(fl::Rule::parse("if BallDirection is rigth then direction is srigth",m_pcFLEngine));
    //mamdani->addRule(fl::Rule::parse("if BallDirection is error then direction is serror", m_pcFLEngine));
    mamdani->addRule(fl::Rule::parse("if BallDistance is close then speed is forward", m_pcFLEngine));
    m_pcFLEngine->addRuleBlock(mamdani);

    std::string status;
    if (not m_pcFLEngine->isReady(&status))
        throw fl::Exception("[engine error] engine is not ready:\n" + status, FL_AT);


    //Set in/outputs
    m_pflObstacleDirection = m_pcFLEngine->getInputVariable("BallDirection");
    m_pflObstacleDistance  = m_pcFLEngine->getInputVariable("BallDistance");
    m_pflSteerDirection    = m_pcFLEngine->getOutputVariable("direction");
    m_pflSpeed             = m_pcFLEngine->getOutputVariable("speed");
}


ControlOutput marbel_Controller::getControlOutput(float cent, float dist)
{

    m_pflObstacleDirection->setValue(cent);
    m_pflObstacleDistance->setValue(dist);

    //std::cout << "hej:" << cent << std::endl;

    m_pcFLEngine->process();

    ControlOutput out;



    out.direction = ((int)(m_pflSteerDirection->getValue() * 100 + .5) / 100.0);
    out.speed     = m_pflSpeed->getValue();

    //std::cout << "output:" << out.direction << std::endl;


    return out;
}
