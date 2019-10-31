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

  public boolean isRobotDrivingBackwards = true;
  public double drivingAdjust;
  public double steeringAdjust;

  // Limelight vision
  private final boolean isFlow;
  public boolean visionStatus;

  // Defense mode
  public boolean defenseToggle = false;
  public Toggle defenseModeToggle;
  public boolean isDefensePositionSet;

  // Pixy2
  private boolean pixyWhiteLine;

  // Blinkin
  private double ledColor;

  // Potentiometer arm
  private double armThrottle;
  private double cargoWheelsThrottle;
  private static enum ArmPosition {
    HighHatch,
    MidHatch,
    LowHatch,
    HighCargo,
    MidCargo,
    LowCargo,
    PickUpCargo,
    ShipCargo,
    HumanPlayerCargo,
    EndGame,
  };
  private double armKp = 0.05;
  private double armMaxDrive = 0.85;
  private double armError;
  private double output;
  private double armDesiredHeight;
  private double actualHeight;
  private double lowerArmLimit = 70;
  private double upperArmLimit = 505;
  private double endGameUpperArmLimit = 150;

  // Potentiometer smallWinch
  private double smallWinchThrottle;
  private static enum SmallWinchPosition {
    StowedLeft,
    CargoUp,
    HatchRight,
    HatchLow,
    HatchMid,
    HatchHigh,
    CargoPickup,
    CargoTop,
    CargoMiddle,
    CargoBottom,
  };
  private double smallWinchMaxDrive = 1;
  private double smallWinchError;
  private double smallWinchOutput;
  private double smallWinchDesiredHeight;
  private double smallWinchActualHight;
  private double smallWinchLowerLimit = 440;
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

    defenseModeToggle = new Toggle();
    lightsAndVisionToggle = new Toggle();
    lightsAndVisionToggle.toggle();
    setLightsAndVision(backLimelight, false);
    setLightsAndVision(frontLimelight, false);
  }

  public void startGame() {
    smallWinchControl(SmallWinchPosition.HatchRight);
  }

  public void gameOp(
      double cameraTargetXAxis,
      double cameraTargetYAxis,
      double cameraTargetArea,
      boolean cameraTarget
    ) {
    
    // Blinkin
    robotmap.blinkin.set(0.43);
    
    // Toggle reverse drive
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
            armControl(ArmPosition.HighCargo);
            smallWinchControl(SmallWinchPosition.CargoTop);
          } else if (oi.getOpButtonX()) {
            armControl(ArmPosition.MidCargo);
            smallWinchControl(SmallWinchPosition.CargoMiddle);
          } else if (oi.getOpButtonA()) {
            armControl(ArmPosition.LowCargo);
            smallWinchControl(SmallWinchPosition.CargoBottom);
          } else if (oi.getOpButtonB()) {
            armControl(ArmPosition.PickUpCargo);
            smallWinchControl(SmallWinchPosition.CargoPickup);
          } else if (oi.getOpRightBumper()) {
            armControl(ArmPosition.ShipCargo);
            smallWinchControl(SmallWinchPosition.CargoUp);
          } else if (oi.getOpDpadUp() || oi.getOpDpadRight() || oi.getOpDpadDown() || oi.getOpDpadLeft()) {
            armControl(ArmPosition.HumanPlayerCargo);
            smallWinchControl(SmallWinchPosition.CargoTop);
          } else {
            // Set arm motor to operator joystick throttle
            armThrottle = op_throttle;
            // Set small winch throttle to zero
            setSmallWinch(op_throttle, op_rightThrottle);
            // Set arm limits
            if (sensors.getArmHeight() < lowerArmLimit && op_throttle < 0) {
              armThrottle = 0.0;
            } else if (sensors.getArmHeight() > upperArmLimit && op_throttle > 0){
              armThrottle = 0.0;
            }
          }
        } else {
          armThrottle = 0.0;
          setSmallWinch(op_throttle, op_rightThrottle);
          // Get Y button to control Claw
          if (oi.getOpButtonPressedY()) {
            final boolean solenoidStatus4 = !air.getSolenoid4();
            air.setSolenoid4(solenoidStatus4);
          }
        }
        // Get Triggers for Cargo Wheels
        if (oi.getOpLeftTriggerValue() != 0) {
          cargoWheelsThrottle = -oi.getOpLeftTriggerValue();
        } else if (oi.getOpRightTriggerValue() != 0) {
          cargoWheelsThrottle = oi.getOpRightTriggerValue();
        } else if (oi.getDriveLeftTrigger()) {
          cargoWheelsThrottle = -1;
        } else if (oi.getDriveRightTrigger()) {
          cargoWheelsThrottle = 1;
        } else {
          cargoWheelsThrottle = 0;
        }
        // Set cargo wheels motors
        robotmap.cargoWheels.set(cargoWheelsThrottle);
        // Fang Servo
        double fangServoPower;
        if (oi.getOpButtonX()) {
          fangServoPower = 1;
        } else if (oi.getOpButtonA()) {
          fangServoPower = 0;
        } else {
          fangServoPower = 0.5;
        }
      } else {
        armThrottle = 0.0;
        // Get X Button to control hatch mechanisms
        if (oi.getOpButtonPressedX()) {
          if (oi.getDriveLeftTrigger()) { // Returns true if driving backwards
            final boolean solenoidStatus2 = !air.getSolenoid2();  // Tung
            air.setSolenoid2(solenoidStatus2);
          } else {
            final boolean solenoidStatus0 = !air.getSolenoid0();  // Beak
            air.setSolenoid0(solenoidStatus0);
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
    final double steeringAdjustKp = isFlow ? 1 : 0.5;
    steeringAdjust = oi.getDriveTurn() * steeringAdjustKp;
    // Drive forwards or backwards
    drivingAdjust = isRobotDrivingBackwards
      ? -oi.getDriveThrottle()
      : oi.getDriveThrottle();

    // Aim and range forwards and backwards
    if (oi.getDriveRightBumper()) {  // Auto targeting
      if (isRobotDrivingBackwards) { // Drives backwards when returns true and will use back camera for targeting
        turnOnLightsAndVision(currentLimelight);
        robotmap.blinkin.set(-0.71);
        // setPipelineBasedOnApproach(currentLimelight);
        currentLimelight.setPipeline(0);  // Always look for center
        AimAndRange aimAndRange = Calculations.getAimAndRangeBackArea(cameraTargetXAxis, cameraTargetArea, cameraTarget);
        adjustSteering(aimAndRange, cameraTarget, steeringAdjustKp);
      } else {
          turnOnLightsAndVision(currentLimelight);
          robotmap.blinkin.set(-0.71);
          currentLimelight.setPipeline(1);  // Always look for left contour
          // if (cameraTargetArea > 15) {
          //   currentLimelight.setPipeline(3);
          // } else {
          //   currentLimelight.setPipeline(2);
          //   // setPipelineBasedOnApproach(currentLimelight);
          // }
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
    final double steeringAdjustKp = 0.5;
    double endGameThrottle = oi.getDriveThrottle();

    if (endGameThrottle < -0.25) {
      endGameThrottle = -0.25;
    }

    robotmap.drive.arcadeDrive(endGameThrottle, oi.getDriveTurn() * steeringAdjustKp);

    // Blinkin
    robotmap.blinkin.set(-0.83);  // Shot Climb

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
      armControl(ArmPosition.EndGame);
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
      if (armHeight < 70) { // Value of kPickupCargo
        robotmap.armKcap.set(0.0);
      }
      if (winchPosition < 440) {  // Value of ksmallWinchStowedLeft
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

      // LED Control
      if (oi.getOpButtonA()) {
        ledColor = -0.05; // Strobe, White
      } else if (oi.getOpButtonB()) {
        ledColor = -0.11; // Strobe, Red
      } else if (oi.getOpButtonX()) {
        ledColor = -0.09; // Strobe, Blue
      } else if (oi.getOpButtonY()) {
        ledColor = -0.07; // Strobe, Gold
      } else {
        ledColor = -0.59;  // Fire, Large
      }
      robotmap.blinkin.set(ledColor);

      return;
    }
      // Set arm position
      if (armHeight > 65) {
        armControl(ArmPosition.PickUpCargo);
        robotmap.armKcap.set(armThrottle);
      }
      // Set winch position
      if (winchPosition > 444) {
        smallWinchControl(SmallWinchPosition.StowedLeft);
        robotmap.smallWinchMotor.set(smallWinchThrottle);
      }
      // Set all pistons to off
      air.setSolenoid0(false);
      air.setSolenoid1(false);
      air.setSolenoid2(false);
      air.setSolenoid3(false);
      air.setSolenoid4(false);
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

  private void adjustSteering(AimAndRange aimAndRange, boolean cameraTarget, double steeringAdjustKp) {
    final int inverter = isRobotDrivingBackwards ? -1 : 1;
    // drivingAdjust = aimAndRange.getDrivingAdjust();
    drivingAdjust = oi.getDriveThrottle() * inverter;
    if (oi.getDriveButtonA()) {
      steeringAdjust = aimAndRange.getSteeringAdjust();
      // if (!cameraTarget) {
        // Look for white line
        // pixyWhiteLine = sensors.getPixyOutput();
        // if (pixyWhiteLine) {
        //   // steeringAdjust = 0.5 * inverter;
        //   steeringAdjust = oi.getDriveTurn() * steeringAdjustKp;
        // } else {
        //   steeringAdjust = oi.getDriveTurn() * steeringAdjustKp;
        // }
      // }
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
  private void setPipelineBasedOnApproachFront(Limelight limelight) {
    if (limelight.getTs() < -2 && limelight.getTs() > -45) { // Approaching from the left
      limelight.setPipeline(2);
    } else if (limelight.getTs() > -88 && limelight.getTs() < -45) {  // Approaching from the right
      limelight.setPipeline(2);
    } else {
      limelight.setPipeline(2);
    }
  }

  private void setSmallWinch(Double op_throttle, Double op_rightThrottle) {
    if (oi.getOpButtonB()) {
      // Set winch motor to operator joystick throttle
      smallWinchThrottle = -op_throttle;
      // Set winch limits
      if (sensors.getSmallWinchPot() < smallWinchLowerLimit && op_throttle < 0) {
        smallWinchThrottle = 0.0;
      } else if (sensors.getSmallWinchPot() > smallWinchUpperLimit && op_throttle > 0){
        smallWinchThrottle = 0.0;
      }
    } else if (op_rightThrottle !=0) {
      // Set winch motor to operator joystick right throttle
      smallWinchThrottle = -op_rightThrottle;
      // Set winch limits
      if (sensors.getSmallWinchPot() < smallWinchLowerLimit && op_rightThrottle < 0) {
        smallWinchThrottle = 0.0;
      } else if (sensors.getSmallWinchPot() > smallWinchUpperLimit && op_rightThrottle > 0){
        smallWinchThrottle = 0.0;
      }
    } else {
    smallWinchThrottle = 0.0;
    }
  }

  /**
   * Arm positioning using the potentiometer
   */
  public double armControl(ArmPosition armPosition) {
    switch (armPosition) {
      // Hatch values
      case HighHatch:
        armDesiredHeight = 500;
        break;
      case MidHatch:
        armDesiredHeight = 291;
        break;
      case LowHatch:
        armDesiredHeight = 100;
        break;
      // Cargo values
      case HighCargo:
        armDesiredHeight = 525; //465;
        break;
      case MidCargo:
        armDesiredHeight = 340; //315;
        break;
      case LowCargo:
        armDesiredHeight = 145; //175;
        break;
      case PickUpCargo:
        armDesiredHeight = 75;
        break;
      case ShipCargo:
        armDesiredHeight = 305;
        break;
      case HumanPlayerCargo:
        armDesiredHeight = 230;
        break;
      // End Game
      case EndGame:
        armDesiredHeight = 150;
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

  private void smallWinchControl(SmallWinchPosition smallWinchPosition) {
    switch (smallWinchPosition) {
      case StowedLeft:
        smallWinchDesiredHeight = 444;
        break;
      case CargoUp: //using this for cargo ship cargo deploy
        smallWinchDesiredHeight = 510;
        break;
      case HatchRight:
        smallWinchDesiredHeight = 575;
        break;
      case HatchLow:
        smallWinchDesiredHeight = 555;
        break;
      case HatchMid:
        smallWinchDesiredHeight = 560;
        break;
      case HatchHigh:
        smallWinchDesiredHeight = 555;
        break;
      case CargoPickup:
        smallWinchDesiredHeight = 500;
        break;
      case CargoTop:
        smallWinchDesiredHeight = 440;
        break;
      case CargoMiddle:
        smallWinchDesiredHeight = 450;
        break;
      case CargoBottom:
        smallWinchDesiredHeight = 450;
        break;
    }

    // Get and set error values to drive towards target height
    smallWinchActualHight = sensors.getSmallWinchPot();
    smallWinchError = smallWinchDesiredHeight - smallWinchActualHight;
    smallWinchOutput = smallWinchError;

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
