package frc.robot;

import frc.robot.Limelight.LightMode;
import frc.robot.Limelight.CameraMode;

/**
 * Robot actions
 */
public class Actions {
  private Air air;
  private Limelight limelight10;
  private Limelight limelight11;
  private OI oi;
  private RobotMap robotmap;
  private Sensors sensors;
  private Toggle lightsAndVisionToggle;

  private boolean useGyroNAVX = false;

  // Limelight vision
  private final boolean isFlow;
  private boolean areLightsAndVisionOn;
  public boolean visionStatus;

  // Potentiometer arm
  private double armThrottle;
  private double cargoWheelsThrottle;
  private static final String kHighHatch = "High Hatch";
  private static final String kMidHatch = "Mid Hatch";
  private static final String kLowHatch = "Low Hatch";
  private static final String kHighCargo = "High Ball";
  private static final String kMidCargo = "Mid Ball";
  private static final String kLowCargo = "Low Ball";
  private double armKp = 0.02;
  private double armMaxDrive = 0.7;  // TODO Needs tuning and arm only goes one way
  private double armError = 0;
  private double output;
  private double armDesiredHeight = 0;
  private double actualHeight;

  public Actions(
      Air air,
      Limelight limelight10,
      Limelight limelight11,
      OI oi,
      RobotMap robotmap,
      Sensors sensors
    ) {
    this.air = air;
    this.limelight10 = limelight10;
    this.limelight11 = limelight11;
    this.oi = oi;
    this.robotmap = robotmap;
    this.sensors = sensors;

    isFlow = robotmap.getIsFlow();

    lightsAndVisionToggle = new Toggle();
    areLightsAndVisionOn = false;
  }

  public void gameOp(
      double cameraTargetXAxis,
      double cameraTargetYAxis,
      double cameraTargetArea,
      boolean cameraTarget
    ) {
    
    /**
     * Operator controls during game operations
     */
    // Get Operator Left Stick Throttle
    final double op_throttle = oi.getOpThrottle();
    // Get left bumper to control Arm for pickup and deploy Hatch
    if (oi.getOpLeftBumper()) {
      if (oi.getOpButtonY()) {
        armControl(kHighHatch);
      } else if (oi.getOpButtonX()) {
        armControl(kMidHatch);
      } else if (oi.getOpButtonA()) {
        armControl(kLowHatch);
      } else {
        armThrottle = op_throttle;
      }
    } else {
      armThrottle = 0.0;
    }
    if (isFlow) {
      robotmap.armFlow.set(armThrottle);
    } else {
      robotmap.armKcap.set(armThrottle);
    }

    // Enable Cargo Mode
    if (oi.getOpRightBumper()) {
      if (oi.getOpButtonPressedY()) {  // Claw
        final boolean solenoidStatus4 = !air.getSolenoid4();  // Cargo Claw
        air.setSolenoid4(solenoidStatus4);
      } else if (oi.getOpButtonPressedX()) { // Fangs
        final boolean solenoidStatus1 = !air.getSolenoid1();  // Fangs
        air.setSolenoid1(solenoidStatus1);
      }
      cargoWheelsThrottle = op_throttle;
    } else {
      cargoWheelsThrottle = 0.0;
    }
    robotmap.cargoWheels.set(cargoWheelsThrottle);

    // Get B Button to control Arm Small Winch
    double winchThrottle;
    if (oi.getOpButtonB()) {
      if (oi.getOpLeftBumper() || oi.getOpRightBumper()) { return; }
      winchThrottle = -op_throttle;
    } else {
      winchThrottle = 0.0;
    }
    robotmap.smallWinchMotor.set(winchThrottle);

    // Get X Button Press to toggle front and back hatch solenoids
    if (oi.getOpButtonPressedX()) {
      if (oi.getOpLeftBumper() || oi.getOpRightBumper()) { return; }
      final boolean solenoidStatus0 = !air.getSolenoid0();  // Arm tri-grabber
      final boolean solenoidStatus2 = !air.getSolenoid2();  // Tung
      if (oi.getDriveLeftTrigger()) { // Returns true if driving backwards
        air.setSolenoid2(solenoidStatus2);
      } else {
        air.setSolenoid0(solenoidStatus0);
      }
    }

    // Get Right Trigger to fire back hatch tung pistons
    final boolean fireBackHatchTung = oi.getOpRightTrigger();
    air.setSolenoid3(fireBackHatchTung);

    /**
     * Driver controls during game operations
     */
    double drivingAdjust;
    double steeringAdjust;
    
    if (oi.getDriveLeftTrigger()) {
      drivingAdjust = -oi.getDriveThrottle();
      steeringAdjust = oi.getDriveTurn();
    } else {
      drivingAdjust = oi.getDriveThrottle();
      steeringAdjust = oi.getDriveTurn();
    }

    if (oi.getDriveRightTrigger()) {  // Auto targeting
      if (oi.getDriveLeftTrigger()) { // Drives backwards when returns true and will use back camera for targeting
        if (!areLightsAndVisionOn) {
          areLightsAndVisionOn = lightsAndVisionToggle.toggle();
          setLightsAndVision(limelight11, areLightsAndVisionOn);
        }
        AimAndRange aimAndRange = Calculations.getAimAndRangeBackArea(cameraTargetXAxis, cameraTargetArea, cameraTarget);
        drivingAdjust = aimAndRange.getDrivingAdjust();
        steeringAdjust = aimAndRange.getSteeringAdjust();
      } else {
          if (!areLightsAndVisionOn) {
            areLightsAndVisionOn = lightsAndVisionToggle.toggle();
            setLightsAndVision(limelight10, areLightsAndVisionOn);
          }
        AimAndRange aimAndRange = Calculations.getAimAndRangeFront(cameraTargetXAxis, cameraTargetYAxis);
        drivingAdjust = aimAndRange.getDrivingAdjust();
        steeringAdjust = aimAndRange.getSteeringAdjust();
      }
    }

    // // If driving only forward or back ward within a threshold enable NavX drive straight
    // if (oi.getDriveTurn() <= .05 && oi.getDriveTurn() >= -0.05) {
    //   if (useGyroNAVX == false) {
    //     sensors.setFollowAngleNAVX(0);
    //   }
    //   useGyroNAVX = true;
    //   steeringAdjust = (sensors.getFollowAngleNAVX() - sensors.getPresentAngleNAVX()) * sensors.kP;
    // } else if ((oi.getDriveTurn() <= .05 && oi.getDriveTurn() >= -0.05)) {
    //   useGyroNAVX = false;
    // }

    // Finally drive
    robotmap.drive.arcadeDrive(drivingAdjust, steeringAdjust);

    // Turn off Limelight lights and vision processing if not being used
    if (areLightsAndVisionOn && !oi.getDriveRightTrigger()) {
      areLightsAndVisionOn = lightsAndVisionToggle.toggle();
      setLightsAndVision(limelight10, areLightsAndVisionOn);
      setLightsAndVision(limelight11, areLightsAndVisionOn);
    }
  }

  public void endGameOp() {
    robotmap.drive.arcadeDrive(-oi.getDriveThrottle(), oi.getDriveTurn());

    if (isFlow) {
      /**
       * Flow Driver controls during end-game operations
       */
      // Driver control arm
      double armThrottle;
      if (oi.getDriveButtonY()) {
        armThrottle = 1;
      } else if (oi.getDriveButtonB()) {
        armThrottle = -1;
      } else {
        armThrottle = 0;
      }
      robotmap.armFlow.set(armThrottle);

      return;
    }

    /**
     * Kcap Driver controls during end-game operations
     */
    // Driver control climb wheels
    double leftClimbWheelThrottle;
    double rightClimbWheelThrottle;
    if (oi.getDriveButtonX()) {
      leftClimbWheelThrottle = 1.0;
      rightClimbWheelThrottle = -1.0;
    } else {
      leftClimbWheelThrottle = 0.0;
      rightClimbWheelThrottle = 0.0;
    }
    robotmap.leftClimberWheel.set(leftClimbWheelThrottle);
    robotmap.rightClimberWheel.set(rightClimbWheelThrottle);

    // Driver control arm
    double driverArmThrottle;
    if (oi.getDriveButtonY()) {
      driverArmThrottle = 1;
    } else if (oi.getDriveButtonB()) {
      driverArmThrottle = -1;
    } else {
      driverArmThrottle = 0;
    }
    robotmap.armKcap.set(driverArmThrottle);

    /**
     * Operator controls during end-game operations
     */
    // Dart climber arm up and down using B and X
    double climberArmThrottle;
    if (oi.getOpButtonB()) {
      climberArmThrottle = 1;
    } else if (oi.getOpButtonX()) {
      climberArmThrottle = -1;
    } else {
      climberArmThrottle = 0;
    }
    robotmap.climberArm.set(climberArmThrottle);

    // Climb legs up and down using A and Y
    double climberLegsThrottle;
    if (oi.getOpButtonA()) {
      climberLegsThrottle = -1;
    } else if (oi.getOpButtonY()) {
      climberLegsThrottle = 1;
    } else {
      climberLegsThrottle = 0;
    }
    robotmap.climberLegs.set(climberLegsThrottle);
  }

	public void setLightsAndVision(Limelight limelight, boolean areLightsAndVisionOn) {
    if (areLightsAndVisionOn) {
      limelight.setLedMode(LightMode.eOn);
      limelight.setCameraMode(CameraMode.eVision);
      visionStatus = true;
			return;
		}
    limelight.setLedMode(LightMode.eOff);
    limelight.setCameraMode(CameraMode.eDriver);
    visionStatus = false;
  }

  public void armControl(String m_armControl) {
    switch (m_armControl) {
      //Rocket
      case kHighHatch:
        armDesiredHeight = 160;  // 9 + 28 + 28;
        break;
      case kMidHatch:
        armDesiredHeight = 275;  // 19 + 28;
        break;
      case kLowHatch:
        armDesiredHeight = 305;  // 19;
        break;
      case kHighCargo:
        armDesiredHeight = 150;  // 27.5 + 28 + 28;
      case kMidCargo:
        armDesiredHeight = 265;  // 27.5 + 28;
        break;
      case kLowCargo:
        armDesiredHeight = 300;  // 27.5;
        break;
    }

    // Get and set error values to drive towards target height
    actualHeight = sensors.getArmPotValue();
    armError = armDesiredHeight - actualHeight;
    output = armKp * armError;
    // Don't let the arm drive too fast
    if (output > armMaxDrive) {
      output = armMaxDrive;
    } else if (output < -armMaxDrive) {
      output = -armMaxDrive;
    }
    // Move arm based on robot and reverse motor (positive is up, negative is down)
    if (robotmap.getIsFlow()) {
      robotmap.armFlow.set(-output);
      System.out.println("armDesiredHeight: " + armDesiredHeight + " | armError: " + armError + " | output: " + output);
    } else {
      robotmap.armKcap.set(-output);
    }
  }
}
