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
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  //For Range
  double distance;

  //For Aim
  private double left_command;
  private double right_command;
  private double Kp = -0.1f;
  private double min_command = 0.05f;
  double steering_adjust = 0.0f;

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
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    // Limelight post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Distance", distance);
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
    getAim();

    if (oi.getOPControlButton() == true) {
      climbTime();
    } else if (oi.getOpLeftBumper() == true) {
      robotmap.drive.tankDrive(left_command, right_command);
    } else {
      robotmap.drive.arcadeDrive(oi.getThrottle(), oi.getTurn());
      robotmap.climbDrive.arcadeDrive(0, 0);
    }

    robotmap.motorSafetyCheck();
  }

  @Override
  public void testPeriodic() {
  }

  public void getRange() {
    double radians = Math.toRadians(ty.getDouble(0.0)); // 39 may change
    distance=((39-23.375)/Math.tan(radians))*0.889149582; //TODO: Camera is not level. This will be a2
  }

  public void getAim() {
    double heading_error = -tx.getDouble(0.0);

    if (-heading_error > 1.0) {
      steering_adjust = Kp * heading_error - min_command;
    }
    else if (-heading_error < 1.0) {
     steering_adjust = Kp * heading_error + min_command;
    }
    left_command += steering_adjust;
    right_command -= steering_adjust;
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

    if (oi.getExtendLegsButton() == true) {
      robotmap.climberLegs.set(0.5);
    } else if (oi.getRetractLegsButton() == true) {
      robotmap.climberLegs.set(-0.5);
    } else {
      robotmap.climberLegs.set(0);
    }
  }

  // public void driveStraight() {
  // double turn = (sensors.kAngleSetpoint - sensors.gyro.getAngle()) *
  // sensors.kP;
  // turn = Math.copySign(turn, direction);
  // robotmap.drive.arcadeDrive(-oi.getThrottle(), turn);
  // }
}
