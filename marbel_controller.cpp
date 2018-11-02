#include "marbel_controller.h"

marbel_Controller::marbel_Controller()
{

}

float marbel_Controller::buildController(int cent)
{

    std::cout << "cent" << cent << std::endl;
    //Create fuzzy
    fl::Engine* engine = new fl::Engine;
    engine->setName("FindBall");
    engine->setDescription("");

    //Membership functions of input error
    fl::InputVariable* inputVariable1 = new fl::InputVariable;
    inputVariable1->setEnabled(true);
    inputVariable1->setName("BallDirection");
    inputVariable1->setRange(0.000, 300.000);
    inputVariable1->setLockValueInRange(false);
    inputVariable1->addTerm(new fl::Ramp("noball", 20000.000, 40000.000));
    inputVariable1->addTerm(new fl::Ramp("farrigth", 200.000, 320.000));
    inputVariable1->addTerm(new fl::Triangle("rigth", 165.000, 185.000, 200.000));
    inputVariable1->addTerm(new fl::Triangle("center", 150.000, 160.000, 170.000));
    inputVariable1->addTerm(new fl::Triangle("left", 120.000,135.000, 155.000));
    inputVariable1->addTerm(new fl::Ramp("farleft", 120.000, 0.000));
    inputVariable1->addTerm(new fl::Ramp("mnoball", -0.000, -400000000.000));
    engine->addInputVariable(inputVariable1);


    //Membership functions of outputdirr
    fl::OutputVariable* outputVariable1 = new fl::OutputVariable;
    outputVariable1->setEnabled(true);
    outputVariable1->setName("direction");
    outputVariable1->setRange(-1.57, 1.57);
    outputVariable1->setLockValueInRange(false);
    outputVariable1->setDefuzzifier(new fl::Centroid(100));
    outputVariable1->setAggregation(new fl::Maximum);
    outputVariable1->setDefaultValue(fl::nan);
    outputVariable1->addTerm(new fl::Ramp("ssharprigth", -1.000, -1.570));
    outputVariable1->addTerm(new fl::Triangle("srigth", -1.200, -0.600, 0.000));
    outputVariable1->addTerm(new fl::Triangle("sstraight", -0.200, 0.000, 0.200));
    outputVariable1->addTerm(new fl::Triangle("sleft", 0.000, 0.600, 1.200));
    outputVariable1->addTerm(new fl::Ramp("ssharpleft", 1.570, 1.000));
    engine->addOutputVariable(outputVariable1);

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
    engine->addOutputVariable(outputVariable2);

    //Rules
    fl::RuleBlock* mamdani = new fl::RuleBlock;
    mamdani->setName("mamdani");
    mamdani->setEnabled(true);
    mamdani->setConjunction(fl::null);
    mamdani->setDisjunction(fl::null);
    mamdani->setImplication(new fl::AlgebraicProduct);
    mamdani->setActivation(new fl::General);
    mamdani->addRule(fl::Rule::parse("if BallDirection is center then direction is sstraight", engine));
    mamdani->addRule(fl::Rule::parse("if BallDirection is farrigth then direction is ssharpleft", engine));
    mamdani->addRule(fl::Rule::parse("if BallDirection is rigth then direction is sleft", engine));
    mamdani->addRule(fl::Rule::parse("if BallDirection is left then direction is srigth", engine));
    mamdani->addRule(fl::Rule::parse("if BallDirection is farleft then direction is ssharprigth", engine));
    engine->addRuleBlock(mamdani);

    std::string status;
    if (not engine->isReady(&status))
        throw fl::Exception("[engine error] engine is not ready:\n" + status, FL_AT);


    //Set inputs
   inputVariable1->setValue(cent);;


      //Start fuzzy
      engine->process();

      //Defuzzification
      float out1 = ((int)(outputVariable1->getValue() * 100 + .5) / 100.0);

      std::cout << "out1" <<std::setprecision(2) << out1 << std::endl;
      std::cout << "speed out" << outputVariable2->getValue() << std::endl;

      return out1;

}


ControlOutput marbel_Controller::getControlOutput(int cent)
{
    m_pflObstacleDistance->setValue(cent);

    // std::cout << "FL - Distance " << m_pcLaserScanner->getClosestDistance(-1.57, 1.57) << ", direction " << m_pcLaserScanner->getClosestDirection(-1.57, 1.57) << std::endl;

    m_pcFLEngine->process();

    ControlOutput out;
    out.direction = m_pflSteerDirection->getValue();
    out.speed     = m_pflSpeed->getValue();

    return out;
}
