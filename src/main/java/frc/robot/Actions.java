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

  // private boolean useGyroNAVX = false;

  // Limelight vision
  private final boolean isFlow;
  private boolean areLightsAndVisionOn;
  public boolean visionStatus;

  // Defense mode
  public boolean defenseToggle = false;
  private boolean isDefensePositionSet;

  // Potentiometer arm
  private double armThrottle;
  private double cargoWheelsThrottle;
  private static final String kHighHatch = "High Hatch";
  private static final String kMidHatch = "Mid Hatch";
  private static final String kLowHatch = "Low Hatch";
  private static final String kHighCargo = "High Cargo";
  private static final String kMidCargo = "Mid Cargo";
  private static final String kLowCargo = "Low Cargo";
  private static final String kPickupCargo = "Pick up Cargo";
  private static final String kEndGame = "End Game";
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
  private static final String ksmallWinchStowedLeft = "Small Winch Up";
  private static final String ksmallWinchCargoUp = "Small Winch Mid";
  private static final String ksmallWinchHatchRight = "Small Winch Down";
  private static final String ksmallWinchCargoPickup = "Small Winch Cargo Pickup";
  private static final String ksmallWinchCargoTop = "Small Winch Cargo Top";
  private static final String ksmallWinchCargoMiddle = "Small Winch Cargo Middle";
  private static final String ksmallWinchCargoBottom = "Small Winch Cargo Bottom";
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
    final double op_rightThrottle = oi.getOpRightThrottle();
    /** CARGO MODE **/
    if (oi.getJoystickEmulatorButtonSwitch1()) {
      // Get left bumper to control arm
      if (oi.getOpLeftBumper()) {
        if (oi.getOpButtonY()) {
          armControl(kHighCargo);
          smallWinchControl(ksmallWinchCargoTop);
        } else if (oi.getOpButtonX()) {
          armControl(kMidCargo);
          smallWinchControl(ksmallWinchCargoMiddle);
        } else if (oi.getOpButtonA()) {
          armControl(kLowCargo);
          smallWinchControl(ksmallWinchCargoBottom);
        } else if (oi.getOpButtonB()) {
          armControl(kPickupCargo);
          smallWinchControl(ksmallWinchCargoPickup);
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
      if (oi.getDriveLeftBumper()) {
        cargoWheelsThrottle = 1;
      // Get Driver Right Bumper for Cargo Wheels intake
      } else if (oi.getDriveRightBumper()) {
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
          armControl(kHighHatch);
          smallWinchControl(ksmallWinchHatchRight);
        } else if (oi.getOpButtonX()) {
          armControl(kMidHatch);
          smallWinchControl(ksmallWinchHatchRight);
        } else if (oi.getOpButtonA()) {
          armControl(kLowHatch);
          smallWinchControl(ksmallWinchHatchRight);
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
          if (oi.getDriveLeftTrigger()) { // Returns true if driving backwards
            air.setSolenoid2(solenoidStatus2);
          } else {
            air.setSolenoid0(solenoidStatus0);
          }
        }
        // Get Right Trigger to fire back hatch tung pistons only if driving backwards
        if (oi.getDriveLeftTrigger()) { // Returns true if driving backwards
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

    /**
     * Driver controls during game operations
     */
    double drivingAdjust;
    double steeringAdjust;
    double steeringAdjustKp = 0.5;
    // Drive forwards or backwards
    if (oi.getDriveLeftTrigger()) {
      drivingAdjust = -oi.getDriveThrottle();
      steeringAdjust = oi.getDriveTurn();
    } else {
      drivingAdjust = oi.getDriveThrottle();
      steeringAdjust = oi.getDriveTurn();
    }
    // Aim and range forwards and backwards
    if (oi.getDriveRightTrigger()) {  // Auto targeting
      if (oi.getDriveLeftTrigger()) { // Drives backwards when returns true and will use back camera for targeting
        if (!areLightsAndVisionOn) {
          areLightsAndVisionOn = lightsAndVisionToggle.toggle();
          setLightsAndVision(limelight10, areLightsAndVisionOn);
        }
        AimAndRange aimAndRange = Calculations.getAimAndRangeBackArea(cameraTargetXAxis, cameraTargetArea, cameraTarget);
        // drivingAdjust = aimAndRange.getDrivingAdjust();
        drivingAdjust = -oi.getDriveThrottle();
        steeringAdjust = aimAndRange.getSteeringAdjust();
      } else {
          if (!areLightsAndVisionOn) {
            areLightsAndVisionOn = lightsAndVisionToggle.toggle();
            setLightsAndVision(limelight11, areLightsAndVisionOn);
          }
        AimAndRange aimAndRange = Calculations.getAimAndRangeFront(cameraTargetXAxis, cameraTargetYAxis, cameraTarget);
        // drivingAdjust = aimAndRange.getDrivingAdjust();
        drivingAdjust = oi.getDriveThrottle();
        steeringAdjust = aimAndRange.getSteeringAdjust();
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
    steeringAdjust = steeringAdjust * steeringAdjustKp;
    robotmap.drive.arcadeDrive(drivingAdjust, steeringAdjust);

    /**
     * Turn off Limelight lights and vision processing if not being used
     */
    if (areLightsAndVisionOn && !oi.getDriveRightTrigger()) {
      areLightsAndVisionOn = lightsAndVisionToggle.toggle();
      setLightsAndVision(limelight10, areLightsAndVisionOn);
      setLightsAndVision(limelight11, areLightsAndVisionOn);
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
      armControl(kEndGame);
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
    if (oi.getJoystickEmulatorButtonPressed3()) {
      defenseToggle = !defenseToggle;
    }
    return defenseToggle;
  }

  public void defenseMode() {
    double armHeight = sensors.getArmHeight();
    double winchPosition = sensors.getSmallWinchPot();
    if (isDefensePositionSet) {
      // Stop arm and winch motors)
      if (armHeight < 60) {
        robotmap.armKcap.set(0.0);
      }
      if (winchPosition < 468) {
        robotmap.smallWinchMotor.set(0.0);
      }
      // Just Drive
      double drivingAdjust;
      double steeringAdjust;
      double steeringAdjustKp = 0.5;
      // Drive forwards or backwards
      if (oi.getDriveLeftTrigger()) {
        drivingAdjust = -oi.getDriveThrottle();
        steeringAdjust = oi.getDriveTurn();
      } else {
        drivingAdjust = oi.getDriveThrottle();
        steeringAdjust = oi.getDriveTurn();
      }
      steeringAdjust = steeringAdjust * steeringAdjustKp;
      robotmap.drive.arcadeDrive(drivingAdjust, steeringAdjust);
      return;
    }
      // Set arm position
      armControl(kPickupCargo);
      robotmap.armKcap.set(armThrottle);
      // Set winch position
      smallWinchControl(ksmallWinchStowedLeft);
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

  private void smallWinchControl(String m_smallWinchControl) {
    switch (m_smallWinchControl) {
      case ksmallWinchStowedLeft:
        smallWinchDesiredHeight = 468;
        break;
      case ksmallWinchCargoUp:
        smallWinchDesiredHeight = 510;
        break;
      case ksmallWinchHatchRight:
        smallWinchDesiredHeight = 575;
        break;
      case ksmallWinchCargoPickup:
        smallWinchDesiredHeight = 525;
        break;
      case ksmallWinchCargoTop:
        smallWinchDesiredHeight = 580;
        break;
      case ksmallWinchCargoMiddle:
        smallWinchDesiredHeight = 510;
        break;
      case ksmallWinchCargoBottom:
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

  /**
   * Arm positioning using the potentiometer
   */
  public double armControl(String m_armControl) {
    switch (m_armControl) {
      // Hatch values
      case kHighHatch:
        armDesiredHeight = 495;
        break;
      case kMidHatch:
        armDesiredHeight = 280;
        break;
      case kLowHatch:
        armDesiredHeight = 90;
        break;
      // Cargo values
      case kHighCargo:
        armDesiredHeight = 460;
        break;
      case kMidCargo:
        armDesiredHeight = 310;
        break;
      case kLowCargo:
        armDesiredHeight = 170;
        break;
      case kPickupCargo:
        armDesiredHeight = 60;
        break;
      // End Game
      case kEndGame:
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

  private void setSmallWinch(Double op_throttle) {
    if (oi.getOpDpadLeft()) {
      // Set winch to stowed
      smallWinchControl(ksmallWinchStowedLeft);
    } else if (oi.getOpDpadUp()) {
      // Set winch to Cargo mode
      smallWinchControl(ksmallWinchCargoUp);
    } else if (oi.getOpDpadRight()) {
      // Set winch to Hatch mode
      smallWinchControl(ksmallWinchHatchRight);
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
}
