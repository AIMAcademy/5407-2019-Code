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
  Actions actions;
  Air air;
  Limelight currentLimelight;
  Limelight limelight10;
  Limelight limelight11;
  OI oi;
  RobotMap robotmap;
  Sensors sensors;

  public String hostNameTen = "limelight-ten";
  public String hostNameEleven = "limelight-eleven";

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private static final String kPipeline0 = "Pipeline 0";
  private static final String kPipeline1 = "Pipeline 1";
  private static final String kPipeline2 = "Pipeline 2";
  private String m_pipelineChoice;
  private final SendableChooser<String> m_pipeline = new SendableChooser<>();

  // Create Limelight Variables for vision processing
  private double cameraTargetXAxis;
  private double cameraTargetYAxis;
  private double cameraTargetArea;
  private boolean cameraTarget;

  // For Range
  double distance;
  double hard_mounting_angle;
  double soft_mounting_angle;

  // For Aim And Range Back
  double steeringAdjustBack;
  double drivingAdjustBack;

  // For Aim and Range Front
  double drivingAdjustFront;
  double steeringAdjustFront;

  // Potentiometer
  private double potValue;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    air = new Air();
    limelight10 = new Limelight(hostNameTen);
    limelight11 = new Limelight(hostNameEleven);
    oi = new OI();
    robotmap = new RobotMap();
    sensors = new Sensors();
    actions = new Actions(air, limelight10, limelight11, oi, robotmap, sensors);

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_pipeline.setDefaultOption("Front-Tape", kPipeline2);
    m_pipeline.addOption("Back-Tape", kPipeline0);
    m_pipeline.addOption("Ball", kPipeline1);
    SmartDashboard.putData("Pipeline", m_pipeline);

    // Turn off Limelight LEDs during init
    // actions.setLightsAndVision(limelight10, true);
    // actions.setLightsAndVision(limelight11, true);

    hard_mounting_angle = Calculations.getHardMountingAngle();
    final int threeFeet = 36; // Assume this distance from camera lens to target
    soft_mounting_angle = Calculations.getSoftMountingAngle(cameraTargetYAxis, threeFeet);
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
    System.out.println("ROBOT PERIODIC");
    System.out.println("drivestick: " + oi.getDriveThrottle());
    SmartDashboard.putNumber("Gyro-NAVX", sensors.getPresentAngleNAVX());
    SmartDashboard.updateValues();

    if (oi.getDriveLeftTrigger()) {
      // Back camera
      currentLimelight = limelight11;
    } else {
      // Front camera
      currentLimelight = limelight10;
    }

    // Use front camera to update limelight values
    cameraTargetXAxis = currentLimelight.getTx();
    cameraTargetYAxis = currentLimelight.getTy();
    cameraTargetArea = currentLimelight.getTa();
    cameraTarget = currentLimelight.isTarget();

    // Update arm potentiometer value
    potValue = sensors.getArmPotValue();

    // Limelight post to smart dashboard periodically
    SmartDashboard.putNumber("limelightX", cameraTargetXAxis);
    SmartDashboard.putNumber("limelightY", cameraTargetYAxis);
    SmartDashboard.putNumber("limelightArea", cameraTargetArea);
    SmartDashboard.putBoolean("limelightTarget", cameraTarget);
    SmartDashboard.putNumber("Distance", distance);
    SmartDashboard.putNumber("hardMA", hard_mounting_angle);
    SmartDashboard.putNumber("softMA", soft_mounting_angle);
    SmartDashboard.putBoolean("visionStatus", actions.visionStatus);
    SmartDashboard.putNumber("PotVal", potValue);

    m_pipelineChoice = m_pipeline.getSelected();
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
    // Zero the NAVX before auton
    sensors.zeroNAVX();

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

  @Override
  public void teleopInit() {
    // Zero the NAVX before teleop
    sensors.zeroNAVX();
  }

  @Override
  public void teleopPeriodic() {
    oi.readValues();
    distance = Calculations.getRange(cameraTargetYAxis);

    if (oi.getOpBackButton()) { // TODO Change to hardware switch on driver station
      actions.endGameOp();
    } else {
      actions.gameOp(cameraTargetXAxis, cameraTargetYAxis, cameraTargetArea, cameraTarget);
    }
  }

  public void updatePipelineChoice() {
    int pipeline = 0;
    switch (m_pipelineChoice) {
      case kPipeline0:
        pipeline = 0;
        break;
      case kPipeline1:
        pipeline = 1;
        break;
      case kPipeline2:
        pipeline = 2;
        break;
    }
    currentLimelight.setPipeline(pipeline);
  }
}
