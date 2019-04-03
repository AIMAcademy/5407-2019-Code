/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Map;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  Actions actions;
  Air air;
  static Limelight backLimelight;
  static Limelight frontLimelight;
  LimelightProvider limelightProvider;
  OI oi;
  RobotMap robotmap;
  Sensors sensors;

  // Shuffleboard
  private boolean selectCodingTab;
  private ShuffleboardTab driveFrontTab = Shuffleboard.getTab("Front");
  private ShuffleboardTab driveBackTab = Shuffleboard.getTab("Back");
  private ShuffleboardTab codeTab = Shuffleboard.getTab("Coding");
  // Coding tab
  private NetworkTableEntry defenseModeEntry;
  private NetworkTableEntry cameraTargetXAxisEntry;
  private NetworkTableEntry cameraTargetYAxisEntry;
  private NetworkTableEntry cameraTargetAreaEntry;
  private NetworkTableEntry cameraTargetSkewEntry;
  private NetworkTableEntry cameraTargetEntry;
  private NetworkTableEntry armPotValueEntry;
  private NetworkTableEntry winchPotValueEntry;
  private NetworkTableEntry tungOpenEntry;
  private NetworkTableEntry drivingAdjustEntry;
  private NetworkTableEntry steeringAdjustEntry;
  private NetworkTableEntry pixyOutputEntry;
  private NetworkTableEntry hardMountingAngleEntry;
  private NetworkTableEntry softMountingAngleEntry;
  private NetworkTableEntry distanceEntry;
  private NetworkTableEntry reverseDriveEntry;
  // Front Tab
  private NetworkTableEntry FRONTdefenseModeEntry;
  private NetworkTableEntry FRONTtungOpenEntry;
  private NetworkTableEntry FRONTreverseDriveEntry;
  // Back tab
  private NetworkTableEntry BACKdefenseModeEntry;
  private NetworkTableEntry BACKtungOpenEntry;
  private NetworkTableEntry BACKreverseDriveEntry;

  // Shuffleboard values
  private boolean isDefenseModeEngaged;
  private double drivingAdjust;
  private double steeringAdjust;
  private boolean pixyOutput;
  private boolean isTungOpen;
  private double armPotValue;
  private double winchPotValue;

  // Create Limelight Variables for vision processing
  private double cameraTargetXAxis;
  private double cameraTargetYAxis;
  private double cameraTargetArea;
  private boolean cameraTarget;
  public double cameraTargetSkew;

  // For Range
  double distance;
  double hard_mounting_angle;
  double soft_mounting_angle;

  // Reverse drive
  private boolean isReverseDrive = true;

  // Limelight http camera feeds
  private HttpCamera limelightFeed10;
  private HttpCamera limelightFeed11;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    air = new Air();
    oi = new OI();
    robotmap = new RobotMap();
    sensors = new Sensors();
    actions = new Actions(air, oi, robotmap, sensors);

    // Create Limelight HTTP Cameras feeds
    limelightFeed10 = new HttpCamera("limelight-ten", "http://limelight-ten.local:5800/stream.mjpg");
    limelightFeed11 = new HttpCamera("limelight-eleven", "http://limelight-eleven.local:5800/stream.mjpg");

    // Set up shuffleboard
    // Driving Front tab
    driveFrontTab.add("eleven", limelightFeed11).withPosition(0, 0).withSize(15, 8).withProperties(Map.of("Show Crosshair", true, "Show Controls", false));
    FRONTreverseDriveEntry = driveFrontTab.add("BACK", isReverseDrive).withPosition(15, 0).withSize(2, 2).getEntry();
    FRONTtungOpenEntry = driveFrontTab.add("Tung", isTungOpen).withPosition(15, 2).withSize(2, 2).getEntry();
    FRONTdefenseModeEntry = driveFrontTab.add("DEFENSE", isDefenseModeEngaged).withPosition(15, 4).withSize(2, 2).getEntry();
    // Driving Back tab
    driveBackTab.add("ten", limelightFeed10).withPosition(0, 0).withSize(15, 8).withProperties(Map.of("Show Crosshair", true, "Show Controls", false));
    BACKreverseDriveEntry = driveBackTab.add("BACK", isReverseDrive).withPosition(15, 0).withSize(2, 2).getEntry();
    BACKtungOpenEntry = driveBackTab.add("Tung", isTungOpen).withPosition(15, 2).withSize(2, 2).getEntry();
    BACKdefenseModeEntry = driveBackTab.add("DEFENSE", isDefenseModeEngaged).withPosition(15, 4).withSize(2, 2).getEntry();
    // Coding tab
    codeTab.add("eleven", limelightFeed11).withPosition(0, 0).withSize(6, 8).withProperties(Map.of("Show Crosshair", true, "Show Controls", false));
    codeTab.add("ten", limelightFeed10).withPosition(6, 0).withSize(6, 8).withProperties(Map.of("Show Crosshair", true, "Show Controls", false));
    hardMountingAngleEntry = codeTab.add("HardMA", hard_mounting_angle).getEntry();
    softMountingAngleEntry = codeTab.add("SoftMA", soft_mounting_angle).getEntry();
    distanceEntry = codeTab.add("Distance", distance).getEntry();
    drivingAdjustEntry = codeTab.add("DA", drivingAdjust).getEntry();
    steeringAdjustEntry = codeTab.add("SA", steeringAdjust).getEntry();
    armPotValueEntry = codeTab.add("ArmPot", armPotValue).getEntry();
    winchPotValueEntry = codeTab.add("WinchPot", winchPotValue).getEntry();
    cameraTargetXAxisEntry = codeTab.add("LL X", cameraTargetXAxis).getEntry();
    cameraTargetYAxisEntry = codeTab.add("LL Y", cameraTargetYAxis).getEntry();
    cameraTargetAreaEntry = codeTab.add("LL A", cameraTargetArea).getEntry();
    cameraTargetSkewEntry = codeTab.add("LL S", cameraTargetSkew).getEntry();
    cameraTargetEntry = codeTab.add("LL T", cameraTarget).getEntry();
    reverseDriveEntry = codeTab.add("BACK", isReverseDrive).getEntry();
    tungOpenEntry = codeTab.add("Tung", isTungOpen).getEntry();
    defenseModeEntry = codeTab.add("DEFENSE", isDefenseModeEngaged).getEntry();
    
    // Instantiate limelights
    limelightProvider = LimelightProvider.getProvider();
    backLimelight = limelightProvider.getBackLimelight();
    frontLimelight = limelightProvider.getFrontLimelight();

    if (!robotmap.getIsFlow()) {
      air.airInit();
    }

    // Zero the NAVX
    sensors.zeroNAVX();

    hard_mounting_angle = Calculations.getHardMountingAngle();
    final int threeFeet = 36; // Assume this distance from camera lens to target
    soft_mounting_angle = Calculations.getSoftMountingAngle(cameraTargetYAxis, threeFeet);

    // Start USB Camera
    // CameraServer.getInstance().startAutomaticCapture();
  }
  
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Gyro-NAVX", sensors.getPresentAngleNAVX());
    SmartDashboard.updateValues();

    // Reverse drive when driver toggles the left bumper
    // This reverses driving motors, switches limelights, and switches shuffleboard tabs
    if (oi.getDriveLeftBumperPressed()) {
      isReverseDrive = !isReverseDrive;
    }
    Limelight currentLimelight = limelightProvider.getCurrentLimelight(isReverseDrive);

    // Select shuffleboard tab
    if (oi.getDriveBackButtonPressed()) {
      selectCodingTab = !selectCodingTab;
    }
    if (selectCodingTab) {
      Shuffleboard.selectTab("Coding");
    } else if (isReverseDrive) {
      Shuffleboard.selectTab("Back");
    } else if (!isReverseDrive) {
      Shuffleboard.selectTab("Front");
    } else {
      Shuffleboard.selectTab("SmartDashboard");
    }

    // Update limelight values
    cameraTargetXAxis = currentLimelight.getTx();
    cameraTargetYAxis = currentLimelight.getTy();
    cameraTargetArea = currentLimelight.getTa();
    cameraTarget = currentLimelight.isTarget();
    cameraTargetSkew = currentLimelight.getTs();

    // Update arm potentiometer value
    armPotValue = sensors.getArmHeight();
    winchPotValue = sensors.getSmallWinchPot();

    // Tung open or closed
    if (!robotmap.getIsFlow()) {
      isTungOpen = air.getSolenoid2();
    }

    // Update values
    isDefenseModeEngaged = actions.defenseModeToggle.get();
    drivingAdjust = actions.drivingAdjust;
    steeringAdjust = actions.steeringAdjust;
    pixyOutput = sensors.getPixyOutput();

    // Update shuffleboard
    // Driving tabs
    FRONTreverseDriveEntry.setBoolean(isReverseDrive);
    FRONTtungOpenEntry.setBoolean(isTungOpen);
    FRONTdefenseModeEntry.setBoolean(isDefenseModeEngaged);
    
    BACKreverseDriveEntry.setBoolean(isReverseDrive);
    BACKtungOpenEntry.setBoolean(isTungOpen);
    BACKdefenseModeEntry.setBoolean(isDefenseModeEngaged);
    // Coding tab
    pixyOutputEntry.setBoolean(pixyOutput);
    distanceEntry.setDouble(distance);
    hardMountingAngleEntry.setDouble(hard_mounting_angle);
    softMountingAngleEntry.setDouble(soft_mounting_angle);
    drivingAdjustEntry.setDouble(drivingAdjust);
    steeringAdjustEntry.setDouble(steeringAdjust);
    armPotValueEntry.setDouble(armPotValue);
    winchPotValueEntry.setDouble(winchPotValue);
    cameraTargetXAxisEntry.setDouble(cameraTargetXAxis);
    cameraTargetYAxisEntry.setDouble(cameraTargetYAxis);
    cameraTargetAreaEntry.setDouble(cameraTargetArea);
    cameraTargetSkewEntry.setDouble(cameraTargetSkew);
    cameraTargetEntry.setBoolean(cameraTarget);
    reverseDriveEntry.setBoolean(isReverseDrive);
    tungOpenEntry.setBoolean(isTungOpen);
    defenseModeEntry.setBoolean(isDefenseModeEngaged);
  }

  @Override
  public void autonomousInit() {
    // Zero the NAVX before auton
    sensors.zeroNAVX();
  
    // Set winch position
    actions.startGame();
  }

  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
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
}
