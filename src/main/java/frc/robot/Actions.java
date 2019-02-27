package frc.robot;

/**
 * Robot actions
 */
public class Actions {
  private Air air;
  private OI oi;
  private Robot robot;
  private RobotMap robotmap;
  private final boolean isFlow;

  private Toggle visionToggle;
  private Toggle ledToggle;

  public Actions(Air air, OI oi, RobotMap robotmap) {
    this.air = air;
    this.oi = oi;
    this.robotmap = robotmap;

    isFlow = robotmap.getIsFlow();

    visionToggle = new Toggle();
    ledToggle = new Toggle();
  }

  public void gameOp(
      double cameraTargetXAxis,
      double cameraTargetYAxis,
      double cameraTargetArea,
      boolean cameraTarget
    ) {

    double drive_throttle = oi.getDriveThrottle();
    if (oi.getDriveLeftTrigger()) {
      drive_throttle = -drive_throttle;
    }
    robotmap.drive.arcadeDrive(drive_throttle, oi.getDriveTurn());
    
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
      winchThrottle = op_throttle;
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
    if (oi.getDriveRightTrigger()) {
      if (oi.getDriveLeftTrigger()) { // Drives backwards when returns true
        // boolean isLedOn = ledToggle.toggle();
        AimAndRange aimAndRange = Calculations.getAimAndRangeBackArea(cameraTargetXAxis, cameraTargetArea, cameraTarget);
        drivingAdjust = -aimAndRange.getDrivingAdjust();
        steeringAdjust = aimAndRange.getSteeringAdjust();
      } else {
        AimAndRange aimAndRange = Calculations.getAimAndRangeFront(cameraTargetXAxis, cameraTargetYAxis);
        drivingAdjust = aimAndRange.getDrivingAdjust();
        steeringAdjust = aimAndRange.getSteeringAdjust();
      }
      robotmap.drive.arcadeDrive(drivingAdjust, steeringAdjust);
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

}
