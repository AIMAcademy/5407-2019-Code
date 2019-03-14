/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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

  // private static final String kDefaultAuto = "Default";
  // private static final String kCustomAuto = "My Auto";
  // private String m_autoSelected;
  // private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // private static final String kPipeline0 = "Pipeline 0";
  // private static final String kPipeline1 = "Pipeline 1";
  // private static final String kPipeline2 = "Pipeline 2";
  // private String m_pipelineChoice;
  // private final SendableChooser<String> m_pipeline = new SendableChooser<>();

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
  private double armPotValue;
  private double winchPotValue;

  // Tung open or closed
  private boolean isTungOpen;

  // Motor voltages
  private double LM0;
  private double LM1;
  private double LM2;
  private double RM0;
  private double RM1;
  private double RM2;

  private NetworkTable mLimelightTable;

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

    mLimelightTable = NetworkTableInstance.getDefault().getTable("limelight10");

    if (!robotmap.getIsFlow()) {
      air.airInit();
    }

    // Zero the NAVX
    sensors.zeroNAVX();

    // m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    // m_chooser.addOption("My Auto", kCustomAuto);
    // SmartDashboard.putData("Auto choices", m_chooser);

    // m_pipeline.setDefaultOption("Front-Tape", kPipeline2);
    // m_pipeline.addOption("Back-Tape", kPipeline0);
    // m_pipeline.addOption("Ball", kPipeline1);
    // SmartDashboard.putData("Pipeline", m_pipeline);

    // Turn off Limelight LEDs during init
    // actions.setLightsAndVision(limelight10, true);
    // actions.setLightsAndVision(limelight11, true);

    hard_mounting_angle = Calculations.getHardMountingAngle();
    final int threeFeet = 36; // Assume this distance from camera lens to target
    soft_mounting_angle = Calculations.getSoftMountingAngle(cameraTargetYAxis, threeFeet);

    // Start USB Camera
    CameraServer.getInstance().startAutomaticCapture();
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

    if (oi.getDriveLeftTrigger()) {
      // Back camera
      currentLimelight = limelight10;
    } else {
      // Front camera
      currentLimelight = limelight11;
    }

    // Update limelight values
    cameraTargetXAxis = currentLimelight.getTx();
    cameraTargetYAxis = currentLimelight.getTy();
    cameraTargetArea = currentLimelight.getTa();
    cameraTarget = currentLimelight.isTarget();

    // Update arm potentiometer value
    armPotValue = sensors.getArmHeight();
    winchPotValue = sensors.getSmallWinchPot();

    // Tung open or closed
    if (!robotmap.getIsFlow()) {
      isTungOpen = air.getSolenoid2();
      SmartDashboard.putBoolean("Tung", isTungOpen);
    }

    // Limelight post to smart dashboard periodically
    SmartDashboard.putNumber("limelightX", cameraTargetXAxis);
    SmartDashboard.putNumber("limelightY", cameraTargetYAxis);
    SmartDashboard.putNumber("limelightArea", cameraTargetArea);
    SmartDashboard.putBoolean("limelightTarget", cameraTarget);
    SmartDashboard.putNumber("Distance", distance);
    SmartDashboard.putNumber("hardMA", hard_mounting_angle);
    SmartDashboard.putNumber("softMA", soft_mounting_angle);
    SmartDashboard.putBoolean("visionStatus", actions.visionStatus);
    SmartDashboard.putNumber("ArmPot", armPotValue);
    SmartDashboard.putNumber("WinchPot", winchPotValue);
    SmartDashboard.putBoolean("limelightTarget", cameraTarget);
    SmartDashboard.putBoolean("DEFENSE", actions.defenseToggle);
    SmartDashboard.putBoolean("BACKWARDS", actions.isRobotDrivingBackwards);
    SmartDashboard.putNumber("DA", actions.drivingAdjust);
    SmartDashboard.putNumber("SA", actions.steeringAdjust);
    SmartDashboard.putBoolean("PIXY", sensors.getPixyOutput());

    // Skew from the right goes from -90 to -45
    // Skew from the left goes from 0 to -45
    SmartDashboard.putNumber("TS", currentLimelight.getTs());

    // double[] cornX = mLimelightTable.getEntry("tcornx").getDoubleArray(new double[0]);
    // double[] cornY = mLimelightTable.getEntry("tcorny").getDoubleArray(new double[0]);
    // double[] cornX = currentLimelight.getTcornX();
    // double[] cornY = currentLimelight.getTcornY();
    // System.out.println("X Array: " + Arrays.toString(cornX) + " | Y Array: " + Arrays.toString(cornY));
    // System.out.println(Arrays.toString(cornY));

    // Values go clockwise from bottom left of bounding box
    // double xValue0 = cornX[0];
    // double xValue1 = cornX[1];
    // double xValue2 = cornX[2];
    // double xValue3 = cornX[3];
    // double yValue0 = cornY[0];
    // double yValue1 = cornY[1];
    // double yValue2 = cornY[2];
    // double yValue3 = cornY[3];
    // SmartDashboard.putNumber("tcornx0", xValue0);
    // SmartDashboard.putNumber("tcornx1", xValue1);
    // SmartDashboard.putNumber("tcornx2", xValue2);
    // SmartDashboard.putNumber("tcornx3", xValue3);
    // SmartDashboard.putNumber("tcorny0", yValue0);
    // SmartDashboard.putNumber("tcorny1", yValue1);
    // SmartDashboard.putNumber("tcorny2", yValue2);
    // SmartDashboard.putNumber("tcorny3", yValue3);

    /*     
    // Get motor voltage values
    LM0 = robotmap.leftMotor_0.getOutputCurrent();
    LM1 = robotmap.leftMotor_1.getOutputCurrent();
    LM2 = robotmap.leftMotor_2.getOutputCurrent();
    RM0 = robotmap.rightMotor_0.getOutputCurrent();
    RM1 = robotmap.rightMotor_1.getOutputCurrent();
    RM2 = robotmap.rightMotor_2.getOutputCurrent();
    // Put motor voltage values on shuffleboard
    SmartDashboard.putNumber("LM0", LM0);
    SmartDashboard.putNumber("LM1", LM1);
    SmartDashboard.putNumber("LM2", LM2);
    SmartDashboard.putNumber("RM0", RM0);
    SmartDashboard.putNumber("RM1", RM1);
    SmartDashboard.putNumber("RM2", RM2);
    */
    // m_pipelineChoice = m_pipeline.getSelected();
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

    // m_autoSelected = m_chooser.getSelected();
    // System.out.println("Auto selected: " + m_autoSelected);

    actions.startGame();
  }

  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
    // switch (m_autoSelected) {
    // case kCustomAuto:
    //   // Put custom auto code here
    //   break;
    // case kDefaultAuto:
    // default:
    //   // Put default auto code here
    //   sensors.setFollowAngleNAVX(0);
    //   robotmap.drive.arcadeDrive(0, (sensors.getFollowAngleNAVX() - sensors.getPresentAngleNAVX()) * 0.015);
    //   break;
    // }
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

    // Check if in defense mode and only run defense function
    if (actions.checkDefenseMode()) {
      actions.defenseMode();
      return;
    }
    // If not in defense mode run either Game Operations or End Game Operations
    if (oi.getJoystickEmulatorButtonSwitch2()) {
      actions.endGameOp();
    } else {
      actions.gameOp(cameraTargetXAxis, cameraTargetYAxis, cameraTargetArea, cameraTarget);
    }
  }

  // public void updatePipelineChoice() {
  //   int pipeline = 0;
  //   switch (m_pipelineChoice) {
  //     case kPipeline0:
  //       pipeline = 0;
  //       break;
  //     case kPipeline1:
  //       pipeline = 1;
  //       break;
  //     case kPipeline2:
  //       pipeline = 2;
  //       break;
  //   }
  //   currentLimelight.setPipeline(pipeline);
  // }
}
