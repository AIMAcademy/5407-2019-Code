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
  private Toggle lightsAndVisionToggle;

  private final boolean isFlow;
  private boolean areLightsAndVisionOn;
  public boolean visionStatus;

  public Actions(Air air, Limelight limelight10, Limelight limelight11, OI oi, RobotMap robotmap) {
    this.air = air;
    this.limelight10 = limelight10;
    this.limelight11 = limelight11;
    this.oi = oi;
    this.robotmap = robotmap;

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
    
    // Get Operator Left Joystick Throttle
    final double op_throttle = oi.getOpThrottle();

    /**
     * Operator controls during game operations
     */
    // Get left bumper to control Arm
    double armThrottle;
    if (oi.getOpLeftBumper()) {
      if (oi.getOpButtonY() || oi.getOpButtonX() || oi.getOpButtonA()) { return; }
      armThrottle = op_throttle;
    } else {
      armThrottle = 0.0;
    }
    if (isFlow) {
      robotmap.armFlow.set(armThrottle);
    } else {
      robotmap.armKcap.set(armThrottle);
    }

    // Get B Button to control Arm Small Winch
    double winchThrottle;
    if (oi.getOpButtonB()) {
      winchThrottle = -op_throttle;
    } else {
      winchThrottle = 0.0;
    }
    robotmap.smallWinchMotor.set(winchThrottle);

    // Get X Button Press to toggle front and back hatch solenoids
    if (oi.getOpButtonPressedX()) {
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
    robotmap.drive.arcadeDrive(drivingAdjust, steeringAdjust);

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
}
