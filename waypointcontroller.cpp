#include "waypointcontroller.h"

waypointController::waypointController()
{

}

void waypointController::buildController()
{

    //Create fuzzy
    waypointEngine = new fl::Engine;
    waypointEngine->setName("Findwaypoint");
    waypointEngine->setDescription("");

    //Membership functions of input error
    fl::InputVariable* inputVariable1 = new fl::InputVariable;
    inputVariable1->setEnabled(true);
    inputVariable1->setName("waypointDirection");
    inputVariable1->setRange(-3.14, 3.14);
    inputVariable1->setLockValueInRange(false);
    inputVariable1->addTerm(new fl::Ramp("farleft", -0.300, -6.00));
    inputVariable1->addTerm(new fl::Triangle("left", -0.000, -0.300, -0.600));
    inputVariable1->addTerm(new fl::Triangle("center", 0.300, 0.000, -0.300));
    inputVariable1->addTerm(new fl::Triangle("rigth", 0.600, 0.300, 0.000));
    inputVariable1->addTerm(new fl::Ramp("farrigth", 0.300, 6.00));
    waypointEngine->addInputVariable(inputVariable1);

    //Membership functions of input error
    fl::InputVariable* inputVariable2 = new fl::InputVariable;
    inputVariable2->setEnabled(true);
    inputVariable2->setName("waypointDistance");
    inputVariable2->setRange(0, 200);
    inputVariable2->setLockValueInRange(false);
    inputVariable2->addTerm(new fl::Ramp("far", 10.000, 40.000));
    inputVariable2->addTerm(new fl::Triangle("close",5.000,10.000, 40.000));
    inputVariable2->addTerm(new fl::Ramp("veryclose", 0.000, 10.00));

    waypointEngine->addInputVariable(inputVariable2);


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
    outputVariable1->addTerm(new fl::Ramp("ssharprigth", 1.000, 0.400));
    outputVariable1->addTerm(new fl::Triangle("srigth",0.500, 0.300, 0.000 ));
    outputVariable1->addTerm(new fl::Triangle("sstraight", 0.100, -0.000, -0.100));
    outputVariable1->addTerm(new fl::Triangle("sleft", -0.000, -0.400, -0.500));
    outputVariable1->addTerm(new fl::Ramp("ssharpleft", -1.000, -0.400));
    waypointEngine->addOutputVariable(outputVariable1);

    //Membership functions of outputspeed
    fl::OutputVariable* outputVariable2 = new fl::OutputVariable;
    outputVariable2->setEnabled(true);
    outputVariable2->setName("speed");
    outputVariable2->setRange(0, 1);
    outputVariable2->setLockValueInRange(false);
    outputVariable2->setAggregation(new fl::Maximum);
    outputVariable2->setDefaultValue(0);
    outputVariable2->setDefuzzifier(new fl::Centroid(100));
    outputVariable2->addTerm(new fl::Ramp("fast", 0.300, 0.500));
    outputVariable2->addTerm(new fl::Triangle("slow",0.200, 0.300, 0.500));
    outputVariable2->addTerm(new fl::Ramp("stop", 0.200, 0.300));
    waypointEngine->addOutputVariable(outputVariable2);

    //Rules
    fl::RuleBlock* mamdani = new fl::RuleBlock;
    mamdani->setName("mamdani");
    mamdani->setEnabled(true);
    mamdani->setConjunction(new fl::Minimum);
    mamdani->setDisjunction(new fl::Maximum);
    mamdani->setImplication(new fl::Minimum);
    mamdani->setActivation(new fl::General);
    mamdani->addRule(fl::Rule::parse("if waypointDirection is center then direction is sstraight", waypointEngine));
    mamdani->addRule(fl::Rule::parse("if waypointDirection is rigth and waypointDistance is far then direction is srigth", waypointEngine));
    mamdani->addRule(fl::Rule::parse("if waypointDirection is rigth and waypointDistance is close then direction is ssharprigth", waypointEngine));
    mamdani->addRule(fl::Rule::parse("if waypointDirection is left and waypointDistance is far then direction is sleft", waypointEngine));
    mamdani->addRule(fl::Rule::parse("if waypointDirection is left and waypointDistance is close then direction is ssharpleft", waypointEngine));
    mamdani->addRule(fl::Rule::parse("if waypointDistance is close then speed is slow", waypointEngine));
    mamdani->addRule(fl::Rule::parse("if waypointDistance is far then speed is fast", waypointEngine));
    mamdani->addRule(fl::Rule::parse("if waypointDistance is veryclose then speed is stop", waypointEngine));
    mamdani->addRule(fl::Rule::parse("if waypointDirection is farleft then direction is ssharprigth and speed is stop", waypointEngine));
    mamdani->addRule(fl::Rule::parse("if waypointDirection is farrigth then direction is ssharpleft and speed is stop",waypointEngine));
    //mamdani->addRule(fl::Rule::parse("if waypointDirection is farleft then speed is stop", waypointEngine));
    //mamdani->addRule(fl::Rule::parse("if waypointDirection is farrigth then speed is stop",waypointEngine));
    waypointEngine->addRuleBlock(mamdani);

    std::string status;
    if (not waypointEngine->isReady(&status))
        throw fl::Exception("[engine error] engine is not ready:\n" + status, FL_AT);


    //Set in/outputs
    waypointDirection = waypointEngine->getInputVariable("waypointDirection");
    waypointDistance  = waypointEngine->getInputVariable("waypointDistance");
    SteerDirection    = waypointEngine->getOutputVariable("direction");
    Speed             = waypointEngine->getOutputVariable("speed");
}


ControlOutput waypointController::getControlOutput()
{
    /// Determine direction to next waypoint
    // Get the latest position
    RobotPosition position = Globals::LastPosition;

    // Calculate the yaw of the robot
    double yaw = fmod(2.0 * atan2(position.rotW, position.rotZ) + 3.0 * M_PI, 2.0 * M_PI);

    // Get X and Y components of the robot's yaw vector
    double yawX = cos(yaw);
    double yawY = sin(yaw);

    // Get displacement of waypoint in comparison to robot position
    double xDisplacement = Globals::CurrentWaypoint.x - position.posX;
    double yDisplacement = position.posY - Globals::CurrentWaypoint.y;

    // Get dot product of vectors
    double dot = yawX * xDisplacement + yawY * yDisplacement;

    // Get cross product of vectors
    double cross = yawX * yDisplacement - yawY * xDisplacement;

    // Get the distance to the waypoint (magnitude of second vector)
    double distance = Globals::GetDistanceToWaypoint();

    // Calculate angle between vectors
    double difference = cross < 0 ? -acos(dot / distance) : acos(dot / distance);

    std::cout << difference << " " << distance << std::endl;


    waypointDirection->setValue(difference);
    waypointDistance->setValue(2);

    //std::cout << "cent:" << cent << std::endl;
    //std::cout << "dist:" << dist << std::endl;

    waypointEngine->process();

    ControlOutput out;


//Movement::Move(Speed->getValue(),SteerDirection->getValue()) skal måske skrives sådan?
    out.direction = ((int)(SteerDirection->getValue() * 100 + .5) / 100.0);
    out.speed     = Speed->getValue();

    std::cout << "output:" << out.direction << std::endl;
    std::cout << "speed:" << out.speed << std::endl;
    std::cout << "point: " << Globals::CurrentWaypoint.x << " " << Globals::CurrentWaypoint.y << std::endl;


    return out;
}
