#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include "rev/CANSparkMax.h"
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/Phoenix.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/EntryListenerFlags.h"
#include <iostream>
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"
#include "rev/ColorSensorV3.h"
#include <frc/smartdashboard/SendableChooser.h>
#include "frc/smartdashboard/SendableChooser.inc"

class Robot : public frc::TimedRobot {

//colorsensor:
static constexpr auto i2cPort = frc::I2C::Port::kOnboard;

rev::ColorSensorV3 m_colorSensor{i2cPort};

enum Pos { ShooterAuton, Run, GET_BALL };

  /**
   * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
   * 
   * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
   * first parameter
   * 
   * The motor type is passed as the second parameter. Motor type can either be:
   *  rev::CANSparkMax::MotorType::kBrushless
   *  rev::CANSparkMax::MotorType::kBrushed
   * 
   * The example below initializes four brushless motors with CAN IDs 1, 2, 3 and 4. Change
   * these parameters to match your setup
   */
  static const int leftLeadDeviceID = 1, leftFollowDeviceID = 2, rightLeadDeviceID = 3, rightFollowDeviceID = 4;
  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  

  static const int intakeDeviceID = 1, innerIntakeDeviceID = 21, flyWheel1DeviceID = 5, flyWheel2DeviceID = 8, climberDeviceID = 12;
  ctre::phoenix::motorcontrol::can::TalonSRX m_intakeMotor{intakeDeviceID};
  ctre::phoenix::motorcontrol::can::TalonFX m_innerIntakeMotor{innerIntakeDeviceID};
  ctre::phoenix::motorcontrol::can::TalonFX m_flyWheelMotor1{flyWheel1DeviceID};
  ctre::phoenix::motorcontrol::can::TalonFX m_flyWheelMotor2{flyWheel2DeviceID};
  ctre::phoenix::motorcontrol::can::TalonSRX m_climberMotor{climberDeviceID};

  /**
   * In RobotInit() below, we will configure m_leftFollowMotor and m_rightFollowMotor to follow 
   * m_leftLeadMotor and m_rightLeadMotor, respectively. 
   * 
   * Because of this, we only need to pass the lead motors to m_robotDrive. Whatever commands are 
   * sent to them will automatically be copied by the follower motors
   */
  frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};

  frc::Joystick m_stick{1};
  frc::XboxController mainJoystick{0};
  frc::SendableChooser<std::string> sendChooser;
  frc::SendableChooser<Pos> m_chooser;

 public:
  //nt::NetworkTableEntry xEntry;
  //nt::NetworkTableEntry yEntry;
  std::shared_ptr<nt::NetworkTable> dtTable;

  double leftDriveSpeed, rightDriveSpeed;
  double distance;
  double minD = distance - 6;
  double maxD = distance + 6;
  void RobotInit() {
    m_chooser.SetDefaultOption("Shooter Auton", Pos::ShooterAuton);
    m_chooser.AddOption("Right", Pos::Run);
    m_chooser.AddOption("Center", Pos::GET_BALL);
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_leftLeadMotor.RestoreFactoryDefaults();
    m_rightLeadMotor.RestoreFactoryDefaults();
    m_leftFollowMotor.RestoreFactoryDefaults();
    m_rightFollowMotor.RestoreFactoryDefaults();

    //m_leftLeadMotor.SetInverted(true);
    
    auto dtInst = nt::NetworkTableInstance::GetDefault();
    dtTable = dtInst.GetTable("limelight");
    

    /**
     * In CAN mode, one SPARK MAX can be configured to follow another. This is done by calling
     * the Follow() method on the SPARK MAX you want to configure as a follower, and by passing
     * as a parameter the SPARK MAX you want to configure as a leader.
     * 
     * This is shown in the example below, where one motor on each side of our drive train is
     * configured to follow a lead motor.
     */
    m_leftFollowMotor.Follow(m_leftLeadMotor);
    m_rightFollowMotor.Follow(m_rightLeadMotor);
    leftDriveSpeed = mainJoystick.GetRightY()*0.75;
    rightDriveSpeed = -mainJoystick.GetRightY()*0.75;
  }
  
  void AutonomousPeriodic() override {
    m_rightLeadMotor.Set(-0.2);
    m_rightFollowMotor.Set(-0.2);
    m_leftLeadMotor.Set(0.2);
    m_leftFollowMotor.Set(0.2);
    //sleep(2);
    /*if ((m_colorSensor.GetColor() == frc::Color::kFirstRed) || (m_colorSensor.GetColor() == frc::Color::kFirstBlue)) {
      m_rightLeadMotor.Set(0);
      m_rightFollowMotor.Set(0);
      m_leftLeadMotor.Set(0);
      m_leftFollowMotor.Set(0);
      sleep(1);
      ShooterAuton();
    }*/
    //m_rightLeadMotor.Set(0);
    //m_rightFollowMotor.Set(0);
    //m_leftLeadMotor.Set(0);
    //m_leftFollowMotor.Set(0);
    //sleep(1);
    //ShooterAuton();
  }

  void TeleopPeriodic() override {
    auto xEntry = dtTable->GetEntry("x").GetDouble(0);
    auto yEntry = dtTable->GetEntry("y").GetDouble(0);

    /*if (m_stick.GetRawButtonReleased(0)) {
      leftDriveSpeed = -leftDriveSpeed;
      rightDriveSpeed = -rightDriveSpeed;
    }*/
    m_robotDrive.TankDrive(mainJoystick.GetLeftY()*0.75, -mainJoystick.GetRightY()*0.75);
    intake();
    ShooterOveride();
    climber();
    ShooterAdjustment();
    //shootingDistance();
    //opposite();
    auto tyEntry = dtTable->GetEntry("ty").GetDouble(0);
    double goalHeightInches = 104;
    double limelightLensHeightInches = 33.0; 
    double limelightMountAngleDegrees = 34.95675993;
    double angleToGoalDegrees = limelightMountAngleDegrees + tyEntry;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    distance = (goalHeightInches - limelightLensHeightInches)/tan(angleToGoalRadians);
    std::cout << distance << std::endl;
  
  }
  /*void opposite(){
    if(mainJoystick.GetRawButton(12) != 0){
      m_robotDrive.TankDrive(-mainJoystick.GetLeftY()*0.75, mainJoystick.GetRightY()*0.75);
    }
    else{
      m_robotDrive.TankDrive(mainJoystick.GetLeftY()*0.75, -mainJoystick.GetRightY()*0.75);
    }
  }*/
  void intake(){
    if (mainJoystick.GetRawAxis(3) > 0 ) { 
      m_intakeMotor.Set(ControlMode::PercentOutput, 0.5);
      m_innerIntakeMotor.Set(ControlMode::PercentOutput, -0.5);
    }
    else{
      if (mainJoystick.GetRawAxis(2) > 0) {
        m_intakeMotor.Set(ControlMode::PercentOutput, -0.5);
        m_innerIntakeMotor.Set(ControlMode::PercentOutput, 0.5);
      }
      else{
        if (mainJoystick.GetRawButton(5) != 0) {
          m_innerIntakeMotor.Set(ControlMode::PercentOutput, 0.5);
        }
        else {
          if (mainJoystick.GetRawButton(6) != 0) {
            m_innerIntakeMotor.Set(ControlMode::PercentOutput, -0.5);
          }          
          else {
            m_intakeMotor.Set(ControlMode::PercentOutput, 0);
            m_innerIntakeMotor.Set(ControlMode::PercentOutput, 0);
          }
        }
      }
    }
  }
  void ShooterOveride(){
    if(m_stick.GetRawButton(11) != 0){
      m_flyWheelMotor1.Set(ControlMode::PercentOutput,  (m_stick.GetRawAxis(3)-1)/2.0);
      m_flyWheelMotor2.Set(ControlMode::PercentOutput,  (m_stick.GetRawAxis(3)-1)/2.0);
       if(m_stick.GetRawButton(2) != 0){
        m_innerIntakeMotor.Set(ControlMode::PercentOutput, -0.5);
       }
       else{
         m_innerIntakeMotor.Set(ControlMode::PercentOutput, 0);
       }
    }
    else{
         m_flyWheelMotor1.Set(ControlMode::PercentOutput,  0);
         m_flyWheelMotor2.Set(ControlMode::PercentOutput,  0);
       }
    
  }
  void climber(){
    if (mainJoystick.GetRawButton(1) != 0) {
      m_climberMotor.Set(ControlMode::PercentOutput, .9);
    }
    else {
      if (mainJoystick.GetRawButton(2) != 0) {
        m_climberMotor.Set(ControlMode::PercentOutput, -.9);
      }
      else {
        m_climberMotor.Set(ControlMode::PercentOutput, 0);
      }
    }
  }
  /*int AdjusmentOfSpeed(){
    int a = b;
    if(m_stick.GetRawButton(10) != 0){
      a = a 1.01;
    }
    else if(m_stick.GetRawButton(12) != 0){
      a =  a * .99;
    }
    else{
      a = a 
    }
    return a; 
  }*/
  void shootingDistance(){
    if(m_stick.GetRawButtonPressed(1) > 1){
      int b = getRPM();
      m_flyWheelMotor1.Set(ControlMode::Velocity,  b);
      m_flyWheelMotor2.Set(ControlMode::Velocity,  b);
     }
     else{
      m_flyWheelMotor1.Set(ControlMode::PercentOutput, 0);
      m_flyWheelMotor2.Set(ControlMode::PercentOutput,  0);
     }
  }
  
  void ShooterAdjustment() {
    auto txEntry = dtTable->GetEntry("tx").GetDouble(0);
    //std::cout << txEntry << std::endl;
    if(m_stick.GetRawButton(3) != 0) {
      if (txEntry < 0) {
        m_rightLeadMotor.Set(0.25);
        m_rightFollowMotor.Set(0.25);
        m_leftLeadMotor.Set(0.25);
        m_leftFollowMotor.Set(0.25);
      }
      else {
        if (txEntry > 0) {
          m_rightLeadMotor.Set(-0.25);
          m_rightFollowMotor.Set(-0.25);
          m_leftLeadMotor.Set(-0.25);
          m_leftFollowMotor.Set(-0.25);
        }
        else {
          m_rightLeadMotor.Set(0);
          m_rightFollowMotor.Set(0);
          m_leftLeadMotor.Set(0);
          m_leftFollowMotor.Set(0);
          //m_flyWheelMotor1.Set(ControlMode::Velocity,  speed);
        }
      }
    }
  }
  /*void ShooterAuton() {
    auto txEntry = dtTable->GetEntry("tx").GetDouble(0);
    double distance = 0;
    double speed = distance / 2;
    //std::cout << txEntry << std::endl;
    if(mainJoystick.GetRawButton(1) > 0) {
      if (txEntry < -1) {
        m_rightLeadMotor.Set(0.25);
        m_rightFollowMotor.Set(0.25);
        m_leftLeadMotor.Set(0.25);
        m_leftFollowMotor.Set(0.25);
      }
      else {
        if (txEntry > 1) {
          m_rightLeadMotor.Set(-0.25);
          m_rightFollowMotor.Set(-0.25);
          m_leftLeadMotor.Set(-0.25);
          m_leftFollowMotor.Set(-0.25);
        }
        else {
          m_rightLeadMotor.Set(0);
          m_rightFollowMotor.Set(0);
          m_leftLeadMotor.Set(0);
          m_leftFollowMotor.Set(0);
          m_flyWheelMotor1.Set(ControlMode::Velocity,  -13350);
          m_flyWheelMotor2.Set(ControlMode::Velocity, -13350);
          sleep(2);
          m_innerIntakeMotor.Set(ControlMode::PercentOutput, -0.5);
        }
      }
    }
  }*/
  int getRPM(){
    int a;
    if(minD < 259.25 < maxD){
      a = -15000;
    }
    else if(minD < 247.25 < maxD){
      a = -14300;
    }
    else if(minD < 235.25 < maxD){
      a =-13500;
    }
    else if(minD< 221.75 < maxD){
      a = -13000;
      }
    else if (minD < 211.25 < maxD){
      a =-12500;
    }
    else if (minD < 199.25 < maxD){
       a= -12000;
    }
    else if (minD < 187.25 < maxD){
        a =-11500;
    }
    else if (minD < 175.25 < maxD){
       a = -11300;
    }
    else{
      std::cout << "Failure, switching to shooter overide" << std::endl;
      ShooterOveride();
    }
    return a;
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif