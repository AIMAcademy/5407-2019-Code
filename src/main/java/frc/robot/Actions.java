package frc.robot;

/**
 * Robot actions
 */
public class Actions {

  private Air air;
  private OI oi;
  private RobotMap robotmap;

  private double op_throttle;
  private boolean isFlow;

  public Actions(Air air, OI oi, RobotMap robotmap) {
    this.air = air;
    this.oi = oi;
    this.robotmap = robotmap;

          if (robotmap.getIsFlow()) {
      isFlow = true;
    } else {
      isFlow = false;
    }
  }

  public void gameOp() {
    // Get Operator Left Joystick Throttle
    op_throttle = oi.getOpThrottle();

    /**
     * Operator controls during game operations
     */
    // Get left bumper to control Arm
    if (oi.getOpLeftBumper()) {
      if (oi.getOpButtonY()) { return; } // Set arm to top position
      else if (oi.getOpButtonX()) { return; } // Set arm to middle position
      else if (oi.getOpButtonA()) { return; } // Set arm to bottom position
      else {  // Manual arm control
        if (isFlow) {
          robotmap.armFlow.set(op_throttle);
        } else {
          robotmap.armKcap.set(op_throttle);
        }
      } 
    } else {
      if (isFlow) {
        robotmap.armFlow.set(0.0);
      } else {
        robotmap.armKcap.set(0.0);
      }
    }
    // Get B Button to control Arm Small Winch
    if (oi.getOpButtonB()) {
      robotmap.smallWinchMotor.set(op_throttle);
    } else {
      robotmap.smallWinchMotor.set(0.0);
    }
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
    if (oi.getOpRightTrigger()) {
      air.setSolenoid3(true);
    } else {
      air.setSolenoid3(false);
    }

    /**
     * Driver controls during game operations
     */
    // Nothing here yet
  }

  public void endGameOp() {
    robotmap.drive.arcadeDrive(-oi.getDriveThrottle(), oi.getDriveTurn());

    if (isFlow) {
    /**
     * Flow Driver controls during end-game operations
     */
    // Driver control arm
    if (oi.getDriveButtonY()) {
      robotmap.armFlow.set(1);
    } else if (oi.getDriveButtonB()) {
      robotmap.armFlow.set(-1);
    } else {
      robotmap.armFlow.set(0);
    }
      return;
    }

    /**
     * Kcap Driver controls during end-game operations
     */     
    // Driver control climb wheels 
    if (oi.getDriveButtonX()) {
      robotmap.leftClimberWheel.set(1.0);
      robotmap.rightClimberWheel.set(-1.0);
    } else {
      robotmap.leftClimberWheel.set(0.0);
      robotmap.rightClimberWheel.set(0.0);
    }
    // Driver control arm
    if (oi.getDriveButtonY()) {
      robotmap.armKcap.set(1);
    } else if (oi.getDriveButtonB()) {
      robotmap.armKcap.set(-1);
    } else {
      robotmap.armKcap.set(0);
    }

    /**
     * Operator controls during end-game operations
     */
    // Dart climber arm up and down using B and X     
    if (oi.getOpButtonB()) {
      robotmap.climberArm.set(1);
    } else if (oi.getOpButtonX()) {
      robotmap.climberArm.set(-1);
    } else {
      robotmap.climberArm.set(0);
    }
    // Climb legs up and down using A and Y
    if (oi.getOpButtonA()) {
      robotmap.climberLegs.set(-1);
    } else if (oi.getOpButtonY()) {
      robotmap.climberLegs.set(1);
    } else {
      robotmap.climberLegs.set(0);
    }
  }
}
