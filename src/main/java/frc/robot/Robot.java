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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {
  Limelight limelight;
  OI oi;
  RobotMap robotmap;
  Sensors sensors;

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Talk to Limelight Network Tables
  // http://docs.limelightvision.io/en/latest/getting_started.html
  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry cameraTargetXAxis = limelightTable.getEntry("tx");
  NetworkTableEntry cameraTargetYAxis = limelightTable.getEntry("ty");
  NetworkTableEntry cameraTargetArea = limelightTable.getEntry("ta");

  // For Range
  double distance;
  double heading_error;
  double mounting_angle;

  // For Aim
  private double steering_adjust = 0.0;

  // For AimAndRange
  double steeringAdjust;
  double drivingAdjust;
  double left_command;
  double right_command;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    limelight = new Limelight();
    oi = new OI(limelight);
    robotmap = new RobotMap();
    sensors = new Sensors();

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    getMountingAngle();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Gyro-NAVX", sensors.getPresentAngleNAVX());
    SmartDashboard.updateValues();

    // Limelight read values periodically
    double x = cameraTargetXAxis.getDouble(0.0);
    double y = cameraTargetYAxis.getDouble(0.0);
    double area = cameraTargetArea.getDouble(0.0);

    // Limelight post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Distance", distance);
    SmartDashboard.putNumber("Mounting Angle", mounting_angle);
    SmartDashboard.putNumber("Heading Error", heading_error);
    SmartDashboard.putNumber("Steering Adjust", steering_adjust);
    SmartDashboard.putBoolean("oi.ledStatus", oi.ledStatus);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
  }

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
      robotmap.drive.arcadeDrive(0, (sensors.getFollowAngleNAVX() - sensors.getPresentAngleNAVX()) * 0.015);
      break;
    }
  }

  // @Override
  public void teleopPeriodic() {
    
    oi.readValues();
    getRange();

    if (oi.getOPControlButton()) {
      climbTime();
    } else if (oi.getOpLeftBumper()) {
      getAim();
      robotmap.drive.arcadeDrive(oi.getClimbThrottle(),steering_adjust);
      robotmap.climbDrive.arcadeDrive(0,0);
    } else if (oi.getOpRightBumper()) {
      getAimAndRange();
      //robotmap.drive.tankDrive(left_command, right_command);
      robotmap.drive.arcadeDrive(-drivingAdjust,steeringAdjust);
      robotmap.climbDrive.arcadeDrive(0,0);
    } else {
      robotmap.drive.arcadeDrive(oi.getThrottle(),oi.getTurn());
      robotmap.climbDrive.arcadeDrive(0,0);
    }

    robotmap.motorSafetyCheck();
  }

  @Override
  public void testPeriodic() {
  }

  public void basicOp() {
    robotmap.arm.set(oi.getBothTriggers());

    if (oi.getOpYButton()) {
      robotmap.leftPickupWheel.set(1);
      robotmap.rightPickupWheel.set(1);
    } else { 
      robotmap.leftPickupWheel.set(0);
      robotmap.rightPickupWheel.set(0);
    }
  }

  // Calculations
  public void getRange() {
    distance = Calculations.getRange(cameraTargetYAxis, mounting_angle);
  }
  public void getMountingAngle() {
    mounting_angle = Calculations.getMountingAngle(cameraTargetYAxis, 48);
  }
  public void getAim() {
    heading_error = Calculations.getHeadingError(cameraTargetXAxis);
    steering_adjust = Calculations.getAim(heading_error);
  }

  public void getAimAndRange() {
    // Code from http://docs.limelightvision.io/en/latest/cs_aimandrange.html
    // Get limelight table for reading tracking data
    double KpAim = 0.045;
    double KpDist = 0.09;
    double AimMinCmd = 0.095;

    double targetX = cameraTargetXAxis.getDouble(0.0);
    double targetY = -cameraTargetYAxis.getDouble(0.0);
    // double targetA = ta.getDouble(0.0); // Might need this if we start using area instead

    // Aim error and distance error based on calibrated limelight cross-hair
    double aim_error = targetX;
    double dist_error = targetY;

    // Steering adjust with a 0.2 degree deadband (close enough at 0.2deg)
    steeringAdjust = KpAim * aim_error;
    if (targetX > .2) {
      steeringAdjust = steeringAdjust + AimMinCmd;
    } else if (targetX < -.2f) {
      steeringAdjust = steeringAdjust - AimMinCmd;
    }

    // Distance adjust, drive to the correct distance from the goal
    drivingAdjust = KpDist * dist_error;

    left_command = drivingAdjust + steeringAdjust;
    right_command =  -(drivingAdjust + steeringAdjust);
  }

  public void climbTime() {
    robotmap.climbDrive.arcadeDrive(oi.getClimbThrottle(), oi.getClimbTurn());
    robotmap.drive.arcadeDrive(oi.getClimbThrottle() * robotmap.climbVsDrive, oi.getClimbTurn());

    if (oi.getExtendArmButton() == true) {
      robotmap.climberArm.set(0.5);
    } // TODO: Check if, ".setSpeed" or ".set" works
    else if (oi.getRetractArmButton() == true) {
      robotmap.climberArm.set(-0.5);
    } else {
      robotmap.climberArm.set(0);
    }

    if (oi.getOpAButton() == true) {
      robotmap.climberLegs.set(0.5);
    } else if (oi.getOpYButton() == true) {
      robotmap.climberLegs.set(-0.5);
    } else {
      robotmap.climberLegs.set(0);
    }
    robotmap.arm.set(oi.getBothTriggers());
  }

  // public void driveStraight() {
  // double turn = (sensors.kAngleSetpoint - sensors.gyro.getAngle()) *
  // sensors.kP;
  // turn = Math.copySign(turn, direction);
  // robotmap.drive.arcadeDrive(-oi.getThrottle(), turn);
  // }
}
