package frc.robot;

import frc.robot.Limelight.LightMode;
import frc.robot.Limelight.CameraMode;

/**
 * Robot actions
 */
public class Actions {
  private Air air;
  private Limelight backLimelight;
  private Limelight frontLimelight;
  private OI oi;
  private RobotMap robotmap;
  private Sensors sensors;
  private Toggle lightsAndVisionToggle;

  // private boolean useGyroNAVX = false;

  private Limelight currentLimelight;
  private LimelightProvider limelightProvider;

  public boolean isRobotDrivingBackwards = false;
  public double drivingAdjust;
  public double steeringAdjust;

  // Limelight vision
  private final boolean isFlow;
  public boolean visionStatus;

  // Defense mode
  public Toggle defenseModeToggle;
  private boolean isDefensePositionSet;

  // Pixy2
  private boolean pixyWhiteLine;

  // Potentiometer arm
  private double armThrottle;
  private double cargoWheelsThrottle;
  private static enum ArmPosition {
    HIGHHATCH,
    MIDHATCH,
    LOWHATCH,
    HIGHCARGO,
    MIDCARGO,
    LOWCARGO,
    PICKUPCARGO,
    ENDGAME,
  };
  private double armKp = 0.05;
  private double armMaxDrive = 0.85;
  private double armError;
  private double output;
  private double armDesiredHeight;
  private double actualHeight;
  private double lowerArmLimit = 70;
  private double upperArmLimit = 505;
  private double endGameUpperArmLimit = 140;

  // Potentiometer smallWinch
  private double smallWinchThrottle;
  private static enum SmallWinchPosition {
    STOWEDLEFT,
    CARGOUP,
    HATCHRIGHT,
    CARGOPICKUP,
    CARGOTOP,
    CARGOMIDDLE,
    CARGOBOTTOM,
  };
  private double smallWinchkP = 0.05;
  private double smallWinchMaxDrive = 1;
  private double smallWinchError;
  private double smallWinchOutput;
  private double smallWinchDesiredHeight;
  private double smallWinchactualHeight;
  private double smallWinchLowerLimit = 455;
  private double smallWinchUpperLimit = 575;

  public Actions(
      Air air,
      OI oi,
      RobotMap robotmap,
      Sensors sensors
    ) {
    this.air = air;
    this.oi = oi;
    this.robotmap = robotmap;
    this.sensors = sensors;

    isFlow = robotmap.getIsFlow();

    limelightProvider = LimelightProvider.getProvider();
    backLimelight = limelightProvider.getBackLimelight();
    frontLimelight = limelightProvider.getFrontLimelight();

    lightsAndVisionToggle = new Toggle();
  }

  public void startGame() {
    smallWinchControl(SmallWinchPosition.HATCHRIGHT);
  }

  public void gameOp(
      double cameraTargetXAxis,
      double cameraTargetYAxis,
      double cameraTargetArea,
      boolean cameraTarget
    ) {
    
    // driver update both triggers and both bumpers
    if (oi.getDriveLeftBumperPressed()) {
      isRobotDrivingBackwards = !isRobotDrivingBackwards;
    }

    currentLimelight = limelightProvider.getCurrentLimelight(isRobotDrivingBackwards);

    if (!isFlow) {
      /**
       * Operator controls during game operations
       */
      // Get Operator Left Stick Throttle
      final double op_throttle = oi.getOpThrottle();
      final double op_rightThrottle = oi.getOpRightThrottle();
      /** CARGO MODE **/
      if (oi.getJoystickEmulatorButtonSwitch1()) {
        // Get left bumper to control arm
        if (oi.getOpLeftBumper()) {
          if (oi.getOpButtonY()) {
            armControl(ArmPosition.HIGHCARGO);
            smallWinchControl(SmallWinchPosition.CARGOTOP);
          } else if (oi.getOpButtonX()) {
            armControl(ArmPosition.MIDCARGO);
            smallWinchControl(SmallWinchPosition.CARGOMIDDLE);
          } else if (oi.getOpButtonA()) {
            armControl(ArmPosition.LOWCARGO);
            smallWinchControl(SmallWinchPosition.CARGOBOTTOM);
          } else if (oi.getOpButtonB()) {
            armControl(ArmPosition.PICKUPCARGO);
            smallWinchControl(SmallWinchPosition.CARGOPICKUP);
          } else {
            // Set small winch throttle to zero
            setSmallWinch(op_throttle);
            // Set arm motor to operator joystick throttle
            armThrottle = op_throttle;
            // Set arm limits
            if (sensors.getArmHeight() < lowerArmLimit && op_throttle < 0) {
              armThrottle = 0.0;
            } else if (sensors.getArmHeight() > upperArmLimit && op_throttle > 0){
              armThrottle = 0.0;
            }
          }
        } else {
          setSmallWinch(op_throttle);
          armThrottle = 0.0;
          // Get Y button to control Claw
          if (oi.getOpButtonPressedY()) {
            final boolean solenoidStatus4 = !air.getSolenoid4();
            air.setSolenoid4(solenoidStatus4);
          }
          // Get X Button to control Fangs
          if (oi.getOpButtonPressedX()) {
            final boolean solenoidStatus1 = !air.getSolenoid1();
            air.setSolenoid1(solenoidStatus1);
          }
        }
        // Get Driver Left Bumper for Cargo Wheels output
        if (oi.getDriveLeftTrigger()) {
          cargoWheelsThrottle = 1;
        // Get Driver Right Bumper for Cargo Wheels intake
        } else if (oi.getDriveRightTrigger()) {
          cargoWheelsThrottle = -1;
        } else {
          cargoWheelsThrottle = op_rightThrottle;
        }
        // Set cargo wheels motors
        robotmap.cargoWheels.set(cargoWheelsThrottle);
      /** HATCH MODE **/
      } else if (!oi.getJoystickEmulatorButtonSwitch1()) {
        if (oi.getOpLeftBumper()) {
          if (oi.getOpButtonY()) {
            armControl(ArmPosition.HIGHHATCH);
            smallWinchControl(SmallWinchPosition.HATCHRIGHT);
          } else if (oi.getOpButtonX()) {
            armControl(ArmPosition.MIDHATCH);
            smallWinchControl(SmallWinchPosition.HATCHRIGHT);
          } else if (oi.getOpButtonA()) {
            armControl(ArmPosition.LOWHATCH);
            smallWinchControl(SmallWinchPosition.HATCHRIGHT);
          } else {
            // Set small winch throttle to zero
            setSmallWinch(op_throttle);
            // Set arm motor to operator joystick throttle
            armThrottle = op_throttle;
            // Set arm limits
            if (sensors.getArmHeight() < lowerArmLimit && op_throttle < 0) {
              armThrottle = 0.0;
            } else if (sensors.getArmHeight() > upperArmLimit && op_throttle > 0){
              armThrottle = 0.0;
            }
          }
        } else {
          setSmallWinch(op_throttle);
          armThrottle = 0.0;
          // Get X Button to control hatch mechanisms
          if (oi.getOpButtonPressedX()) {
            final boolean solenoidStatus0 = !air.getSolenoid0();  // Arm tri-grabber
            final boolean solenoidStatus2 = !air.getSolenoid2();  // Tung
            if (isRobotDrivingBackwards) { // Returns true if driving backwards
              air.setSolenoid2(solenoidStatus2);
            } else {
              air.setSolenoid0(solenoidStatus0);
            }
          }
          // Get Right Trigger to fire back hatch tung pistons only if driving backwards
          if (isRobotDrivingBackwards) { // Returns true if driving backwards
            final boolean fireBackHatchTung = oi.getOpRightTrigger();
            air.setSolenoid3(fireBackHatchTung);
          }
        }
      }
      /** BOTH MODES **/
      // Set arm motor
      if (isFlow) {
        robotmap.armFlow.set(armThrottle);
      } else {
        robotmap.armKcap.set(armThrottle);
      }
      // Small Winch
      robotmap.smallWinchMotor.set(smallWinchThrottle);
    }

    /**
     * Driver controls during game operations
     */
    double steeringAdjustKp = 0.5;
    steeringAdjust = oi.getDriveTurn() * steeringAdjustKp;

    // Drive forwards or backwards
    drivingAdjust = isRobotDrivingBackwards
      ? -oi.getDriveThrottle()
      : oi.getDriveThrottle();

    // Aim and range forwards and backwards
    if (oi.getDriveRightBumper()) {  // Auto targeting
      if (isRobotDrivingBackwards) { // Drives backwards when returns true and will use back camera for targeting
        turnOnLightsAndVision(backLimelight);
        setPipelineBasedOnApproach(backLimelight);
        AimAndRange aimAndRange = Calculations.getAimAndRangeBackArea(cameraTargetXAxis, cameraTargetArea, cameraTarget);
        adjustSteering(aimAndRange, cameraTarget, steeringAdjustKp);
      } else {
          turnOnLightsAndVision(frontLimelight);
          if (cameraTargetArea > 15) {
            frontLimelight.setPipeline(3);
          } else {
            setPipelineBasedOnApproach(frontLimelight);
          }
        AimAndRange aimAndRange = Calculations.getAimAndRangeFront(cameraTargetXAxis, cameraTargetYAxis, cameraTarget);
        adjustSteering(aimAndRange, cameraTarget, steeringAdjustKp);
      }
    }
    // If driving only forward or backward within a threshold enable NavX drive straight
    // if (oi.getDriveThrottle() == 0 || oi.getDriveTurn() != 0){
    //   useGyroNAVX = false;
    // } else if (oi.getDriveTurn() == 0 && oi.getDriveThrottle() != 0){
    //   if (useGyroNAVX == false) {
    //     sensors.setFollowAngleNAVX(0);
    //   }
    //   useGyroNAVX = true;
    //   steeringAdjust = (sensors.getFollowAngleNAVX() - sensors.getPresentAngleNAVX()) * sensors.kP;
    // }
    // Finally drive
    if (isFlow) {
      drivingAdjust = -drivingAdjust;
    }
    robotmap.drive.arcadeDrive(drivingAdjust, steeringAdjust);

    /**
     * Turn off Limelight lights and vision processing if not being used
     */
    if (lightsAndVisionToggle.get() && !oi.getDriveRightBumper()) {
      final boolean areLightsAndVisionOn = lightsAndVisionToggle.toggle();
      setLightsAndVision(backLimelight, areLightsAndVisionOn);
      setLightsAndVision(frontLimelight, areLightsAndVisionOn);
    }

    // Set Defense Mode position variable to indicate not set
    isDefensePositionSet = false;
  }

  public void endGameOp() {
    robotmap.drive.arcadeDrive(oi.getDriveThrottle(), oi.getDriveTurn());

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
      // Set arm limits
      if (sensors.getArmHeight() > endGameUpperArmLimit && driverArmThrottle > 0){
        driverArmThrottle = 0.0;
      }
    } else if (oi.getDriveButtonB()) {
      driverArmThrottle = -1;
      if (sensors.getArmHeight() < lowerArmLimit && driverArmThrottle < 0) {
        driverArmThrottle = 0.0;
      }
    } else if (sensors.getArmHeight() > endGameUpperArmLimit) {
      armControl(ArmPosition.ENDGAME);
      driverArmThrottle = armThrottle;
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

  public boolean checkDefenseMode() {
    return oi.getJoystickEmulatorButtonPressed3()
     ? defenseModeToggle.toggle()
     : defenseModeToggle.get();
  }

  public void defenseMode() {
    double armHeight = sensors.getArmHeight();
    double winchPosition = sensors.getSmallWinchPot();
    if (isDefensePositionSet) {
      // Stop arm and winch motors)
      if (armHeight < 60) { // Value of kPickupCargo
        robotmap.armKcap.set(0.0);
      }
      if (winchPosition < 455) {  // Value of ksmallWinchStowedLeft
        robotmap.smallWinchMotor.set(0.0);
      }
      // Just Drive
      double drivingAdjust;
      double steeringAdjust;
      double steeringAdjustKp = 0.5;

      // Drive forwards or backwards
      drivingAdjust = isRobotDrivingBackwards ? -oi.getDriveThrottle() : oi.getDriveThrottle();
      steeringAdjust = oi.getDriveTurn() * steeringAdjustKp;
      robotmap.drive.arcadeDrive(drivingAdjust, steeringAdjust);
      return;
    }
      // Set arm position
      armControl(ArmPosition.PICKUPCARGO);
      robotmap.armKcap.set(armThrottle);
      // Set winch position
      smallWinchControl(SmallWinchPosition.STOWEDLEFT);
      robotmap.smallWinchMotor.set(smallWinchThrottle);
      // Set all pistons to off
      air.setSolenoid0(false);
      air.setSolenoid1(false);
      air.setSolenoid2(false);
      air.setSolenoid3(false);
      air.setSolenoid4(false);
      // Set LEDs
      // TODO create this code
      // Set defense position variable so that this code only runs once each time Defense Mode is engaged
      isDefensePositionSet = true;
  }

  /**
   * Toggle the limelight lights and camera modes
   */
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

  /**
   * Arm positioning using the potentiometer
   */
  public double armControl(ArmPosition armPosition) {
    switch (armPosition) {
      // Hatch values
      case HIGHHATCH:
        armDesiredHeight = 500;
        break;
      case MIDHATCH:
        armDesiredHeight = 291;
        break;
      case LOWHATCH:
        armDesiredHeight = 112;
        break;
      // Cargo values
      case HIGHCARGO:
        armDesiredHeight = 465;
        break;
      case MIDCARGO:
        armDesiredHeight = 315;
        break;
      case LOWCARGO:
        armDesiredHeight = 175;
        break;
      case PICKUPCARGO:
        armDesiredHeight = 75;
        break;
      // End Game
      case ENDGAME:
        armDesiredHeight = 140;
        break;
    }

    // Get and set error values to drive towards target height
    actualHeight = sensors.getArmHeight();
    armError = armDesiredHeight - actualHeight;
    output = armKp * armError;

    // Don't let the arm drive too fast
    if (output > armMaxDrive) {
      output = armMaxDrive;
    } else if (output < -armMaxDrive) {
      output = -armMaxDrive;
    }
    
    // Convert to armThrottle to send back to arm control function
    armThrottle = output;

    return armThrottle;
  }

  private void adjustSteering(AimAndRange aimAndRange, boolean cameraTarget, double steeringAdjustKp) {
    final int inverter = isRobotDrivingBackwards ? -1 : 1;
    // drivingAdjust = aimAndRange.getDrivingAdjust();
    drivingAdjust = oi.getDriveThrottle() * inverter;
    if (oi.getDriveButtonA()) {
      steeringAdjust = aimAndRange.getSteeringAdjust();
      if (!cameraTarget) {
        // Look for white line
        pixyWhiteLine = sensors.getPixyOutput();
        if (pixyWhiteLine) {
          steeringAdjust = 0.5 * inverter;
        } else {
          steeringAdjust = oi.getDriveTurn() * steeringAdjustKp;
        }
      }
    } else {
      steeringAdjust = oi.getDriveTurn() * steeringAdjustKp;
    }
  }

  private void setPipelineBasedOnApproach(Limelight limelight) {
    if (limelight.getTs() < -2 && limelight.getTs() > -45) { // Approaching from the left
      limelight.setPipeline(1);
    } else if (limelight.getTs() > -88 && limelight.getTs() < -45) {  // Approaching from the right
      limelight.setPipeline(2);
    } else {
      limelight.setPipeline(0);
    }
  }

  private void setSmallWinch(Double op_throttle) {
    if (oi.getOpDpadLeft()) {
      // Set winch to stowed
      smallWinchControl(SmallWinchPosition.STOWEDLEFT);
    } else if (oi.getOpDpadUp()) {
      // Set winch to Cargo mode
      smallWinchControl(SmallWinchPosition.CARGOUP);
    } else if (oi.getOpDpadRight()) {
      // Set winch to Hatch mode
      smallWinchControl(SmallWinchPosition.HATCHRIGHT);
    } else if (oi.getOpButtonB()) {
        // Set winch motor to operator joystick throttle
        smallWinchThrottle = -op_throttle;
        // Set winch limits
        if (sensors.getSmallWinchPot() < smallWinchLowerLimit && op_throttle < 0) {
          smallWinchThrottle = 0.0;
        } else if (sensors.getSmallWinchPot() > smallWinchUpperLimit && op_throttle > 0){
          smallWinchThrottle = 0.0;
        }
    } else {
      smallWinchThrottle = 0.0;
    }
  }

  private void smallWinchControl(SmallWinchPosition smallWinchPosition) {
    switch (smallWinchPosition) {
      case STOWEDLEFT:
        smallWinchDesiredHeight = 455;
        break;
      case CARGOUP:
        smallWinchDesiredHeight = 510;
        break;
      case HATCHRIGHT:
        smallWinchDesiredHeight = 575;
        break;
      case CARGOPICKUP:
        smallWinchDesiredHeight = 515;
        break;
      case CARGOTOP:
        smallWinchDesiredHeight = 580;
        break;
      case CARGOMIDDLE:
        smallWinchDesiredHeight = 510;
        break;
      case CARGOBOTTOM:
        smallWinchDesiredHeight = 510;
        break;
    }

    // Get and set error values to drive towards target height
    smallWinchactualHeight = sensors.getSmallWinchPot();
    smallWinchError = smallWinchDesiredHeight - smallWinchactualHeight;
    smallWinchOutput = smallWinchkP * smallWinchError;

    // Don't let the winch drive too fast
    if (smallWinchOutput > smallWinchMaxDrive) {
      smallWinchOutput = smallWinchMaxDrive;
    } else if (smallWinchOutput < -smallWinchMaxDrive) {
      smallWinchOutput = -smallWinchMaxDrive;
    }

    smallWinchThrottle = -smallWinchOutput;
  }

  private void turnOnLightsAndVision(Limelight limelight) {
    if (!lightsAndVisionToggle.get()) {
      setLightsAndVision(limelight, lightsAndVisionToggle.toggle());
    }
  }
}
