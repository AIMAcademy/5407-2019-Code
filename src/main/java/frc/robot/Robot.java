/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

	OI oi;
  RobotMap robotmap;
  Sensors sensors;

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

		oi = new OI();
    robotmap = new RobotMap();
    sensors = new Sensors();

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Gyro-NAVX", sensors.getPresentAngleNAVX());
    SmartDashboard.putNumber("left motor 1 temp", robotmap.leftMotor_1.getMotorTemperature());
    SmartDashboard.putNumber("left motor 2 temp", robotmap.leftMotor_2.getMotorTemperature());
    SmartDashboard.putNumber("right motor 1 temp", robotmap.rightMotor_1.getMotorTemperature());
    SmartDashboard.putNumber("right motor 2 temp", robotmap.rightMotor_2.getMotorTemperature());
		SmartDashboard.updateValues();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector",
    // defaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        sensors.setFollowAngleNAVX(0);
        robotmap.drive.arcadeDrive(0,
        (sensors.getFollowAngleNAVX() - sensors.getPresentAngleNAVX()) * 0.015);
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  //@Override
  public void teleopPeriodic() {

    //Only here to test the button inputs on Xbox controler

    oi.readValues();

    if (oi.getOPControlButton() == true) {
      climbTime();
    } else {
      robotmap.drive.arcadeDrive(oi.getThrottle(), oi.getTurn()); 
      robotmap.climbDrive.arcadeDrive(0, 0);
    }
    
    robotmap.motorSafetyCheck();

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public void climbTime() {
    robotmap.climbDrive.arcadeDrive(oi.getClimbThrottle(), oi.getClimbTurn());
    robotmap.drive.arcadeDrive(oi.getClimbThrottle()*robotmap.climbVsDrive, oi.getClimbTurn());

    if (oi.getExtendArmButton() == true) {robotmap.climberArm.set(0.5);} //TODO: Check if, ".setSpeed" or ".set" works
    else if (oi.getRetractArmButton() == true) {robotmap.climberArm.set(-0.5);}
    else {robotmap.climberArm.set(0);}

    if (oi.getExtendLegsButton() == true) {robotmap.climberLegs.set(0.5);}
    else if (oi.getRetractLegsButton() == true) {robotmap.climberLegs.set(-0.5);}
    else {robotmap.climberLegs.set(0);}
  }

  // public void driveStraight() {
  //   double turn = (sensors.kAngleSetpoint - sensors.gyro.getAngle()) * sensors.kP;
  //   turn = Math.copySign(turn, direction);
  //   robotmap.drive.arcadeDrive(-oi.getThrottle(), turn);
  // }


}
