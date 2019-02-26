package frc.robot;

/**
 * Robot actions
 */
public class Actions {

    private Air air;
    private OI oi;
    private RobotMap robotmap;

    private double op_throttle;

    public Actions(Air air, OI oi, RobotMap robotmap) {
      this.air = air;
      this.oi = oi;
      this.robotmap = robotmap;
    }

    public void gameOp() {
      // Get Operator Left Joystick Throttle
      op_throttle = oi.getOpThrottle();

      /**
       * Operator controls during game operations
       */
      // Get left bumper to control Arm
      if (oi.getOpLeftBumper()) {
        if (oi.getOpButtonY()) {  } // Set arm to top position
        else if (oi.getOpButtonX()) {  } // Set arm to middle position
        else if (oi.getOpButtonA()) {  } // Set arm to bottom position
        else { robotmap.arm.set(op_throttle); } // Set arm to top position
      }
      // Get B Button to control Arm Small Winch
      if (oi.getOpButtonB()) {
        robotmap.smallWinchMotor.set(op_throttle);
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
    }

    public void endGameOp() {
      robotmap.drive.arcadeDrive(-oi.getDriveThrottle(), oi.getDriveTurn());
      if (oi.getOpBackButton()) { // Change to hardware switch on driver station
        /**
         * Driver controls during end-game operations
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
          robotmap.climberArm.set(1);
        } else if (oi.getDriveButtonB()) {
          robotmap.climberArm.set(-1);
        } else {
          robotmap.climberArm.set(0);
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
}
