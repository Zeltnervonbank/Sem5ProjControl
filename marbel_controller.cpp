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
    inputVariable1->setRange(0.000, 300.000);
    inputVariable1->setLockValueInRange(false);
    inputVariable1->addTerm(new fl::Ramp("noball", 20000.000, 40000.000));
    inputVariable1->addTerm(new fl::Ramp("farrigth", 195.000, 320.000));
    inputVariable1->addTerm(new fl::Triangle("rigth", 165.000, 185.000, 205.000));
    inputVariable1->addTerm(new fl::Triangle("center", 150.000, 160.000, 170.000));
    inputVariable1->addTerm(new fl::Triangle("left", 120.000,135.000, 155.000));
    inputVariable1->addTerm(new fl::Ramp("farleft", 120.000, 0.000));
    inputVariable1->addTerm(new fl::Ramp("mnoball", -0.000, -400000000.000));
    m_pcFLEngine->addInputVariable(inputVariable1);


    //Membership functions of outputdirr
    fl::OutputVariable* outputVariable1 = new fl::OutputVariable;
    outputVariable1->setEnabled(true);
    outputVariable1->setName("direction");
    outputVariable1->setRange(-1.57, 1.57);
    outputVariable1->setLockValueInRange(false);
    outputVariable1->setDefuzzifier(new fl::Centroid(100));
    outputVariable1->setAggregation(new fl::Maximum);
    outputVariable1->setDefaultValue(fl::nan);
    outputVariable1->addTerm(new fl::Ramp("ssharprigth", -1.570, -1.000));
    outputVariable1->addTerm(new fl::Triangle("srigth",-1.000, -0.600, -0.100 ));
    outputVariable1->addTerm(new fl::Triangle("sstraight", -0.100, 0.000, 0.100));
    outputVariable1->addTerm(new fl::Triangle("sleft", 0.100, 0.600, 1.000));
    outputVariable1->addTerm(new fl::Ramp("ssharpleft",1.570, 1.000));
    m_pcFLEngine->addOutputVariable(outputVariable1);

    //Membership functions of outputspeed
    fl::OutputVariable* outputVariable2 = new fl::OutputVariable;
    outputVariable2->setEnabled(true);
    outputVariable2->setName("speed");
    outputVariable2->setRange(-1, 1);
    outputVariable2->setLockValueInRange(false);
    outputVariable2->setAggregation(new fl::Maximum);
    outputVariable2->setDefaultValue(fl::nan);
    outputVariable2->setDefuzzifier(new fl::Centroid(100));
    outputVariable2->addTerm(new fl::Ramp("forward", 1.000, 0.000));
    outputVariable2->addTerm(new fl::Ramp("backward", 0.000,-1.000));
    outputVariable2->addTerm(new fl::Triangle("still", 0.010,-0.010));
    m_pcFLEngine->addOutputVariable(outputVariable2);

    //Rules
    fl::RuleBlock* mamdani = new fl::RuleBlock;
    mamdani->setName("mamdani");
    mamdani->setEnabled(true);
    mamdani->setConjunction(fl::null);
    mamdani->setDisjunction(fl::null);
    mamdani->setImplication(new fl::AlgebraicProduct);
    mamdani->setActivation(new fl::General);
    mamdani->addRule(fl::Rule::parse("if BallDirection is center then direction is sstraight", m_pcFLEngine));
    mamdani->addRule(fl::Rule::parse("if BallDirection is rigth then direction is srigth", m_pcFLEngine));
    mamdani->addRule(fl::Rule::parse("if BallDirection is farrigth then direction is ssharprigth", m_pcFLEngine));
    mamdani->addRule(fl::Rule::parse("if BallDirection is left then direction is sleft", m_pcFLEngine));
    mamdani->addRule(fl::Rule::parse("if BallDirection is farleft then direction is ssharpleft", m_pcFLEngine));
    m_pcFLEngine->addRuleBlock(mamdani);

    std::string status;
    if (not m_pcFLEngine->isReady(&status))
        throw fl::Exception("[engine error] engine is not ready:\n" + status, FL_AT);


    //Set inputs
    m_pflObstacleDirection = m_pcFLEngine->getInputVariable("BallDirection");
    //m_pflObstacleDistance  = m_pcFLEngine->getInputVariable("ObstacleDistance");
    m_pflSteerDirection    = m_pcFLEngine->getOutputVariable("direction");
    //m_pflSpeed             = m_pcFLEngine->getOutputVariable("Speed");
}


ControlOutput marbel_Controller::getControlOutput(int cent)
{
    m_pflObstacleDirection->setValue(cent);

    m_pcFLEngine->process();

    ControlOutput out;
    out.direction = ((int)(m_pflSteerDirection->getValue() * 100 + .5) / 100.0);
   //out.speed     = m_pflSpeed->getValue();



    return out;
}
