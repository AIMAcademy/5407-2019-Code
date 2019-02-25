/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Limelight.CameraMode;
import frc.robot.Limelight.LightMode;

/**
 * Add your docs here.
 */
public class OI {
	public Toggle ledToggle;
	private Toggle visionToggle;

	private Joystick driveStick;
	private double drive_throttle;
	private double drive_turn;
	private boolean drive_buttonA;
	private boolean drive_buttonB;
	private boolean drive_buttonX;
	private boolean drive_buttonY;
	private boolean drive_leftBumper;
	private boolean drive_rightBumper;
	private boolean drive_back;
	private boolean drive_start;

	private double xDriveFrontBackSwitch;  // change this

	private Joystick opStick;
	private double op_throttle;
	private boolean op_buttonA;
	private boolean op_buttonB;
	private boolean op_buttonX;
	private boolean op_buttonY;
	private boolean op_leftBumper;
	private boolean op_rightBumper;
	private boolean op_back;
	private boolean op_start;

	public boolean ledStatus;
	public boolean visionStatus;

	private Limelight limelight;

	public OI(Limelight limelight) {
		this.limelight = limelight;
		ledToggle = new Toggle();
		visionToggle = new Toggle();

		// driveStick = new Joystick(0);
		driveStick = new Joystick(0);
		opStick = new Joystick(1);
	}

	public void readValues() {	
		/**
		 * Read Driver joystick axes
		 */
		// Driver Left Stick Y Axis 1 for Driver Throttle
		if (driveStick.getRawAxis(1) < -0.2 || opStick.getRawAxis(1) > 0.2) {
			drive_throttle = -opStick.getRawAxis(1);
		} else {
			drive_throttle = 0;
		}	
		
		// Driver Left Stick R Axis 4 for Driver Turn
		if (driveStick.getRawAxis(4) < -0.2 || opStick.getRawAxis(4) > 0.2) {
			drive_turn = -opStick.getRawAxis(4);
		} else {
			drive_turn = 0;
		}	

		/**
		 * Read Operator joystick axes
		 */
		// Operator Left Stick Y Axis 1
		if (opStick.getRawAxis(1) < -0.2 || opStick.getRawAxis(1) > 0.2) {
			op_throttle = -opStick.getRawAxis(1);
		} else {
			op_throttle = 0;
		}	

		/**
		 * Read Driver joystick buttons
		 */
		drive_buttonA = driveStick.getRawButton(1);
		drive_buttonB = driveStick.getRawButton(2);
		drive_buttonX = driveStick.getRawButton(3);
		drive_buttonY = driveStick.getRawButton(4);
		drive_leftBumper = driveStick.getRawButton(5);
		drive_rightBumper = driveStick.getRawButton(6);
		drive_back = driveStick.getRawButton(7);
		drive_start = driveStick.getRawButton(8);

		/**
		 * Read Operator joystick buttons
		 */
		op_buttonA = opStick.getRawButton(1);
		op_buttonB = opStick.getRawButton(2);
		op_buttonX = opStick.getRawButton(3);
		op_buttonY = opStick.getRawButton(4);
		op_leftBumper = opStick.getRawButton(5);
		op_rightBumper = opStick.getRawButton(6);
		op_back = opStick.getRawButton(7);
		op_start = opStick.getRawButton(8);

		/**
		 * Read button presses - fix this
		 */
		if (opStick.getRawButtonPressed(8)) {
			boolean isLedOn = ledToggle.toggle();
			setLed(isLedOn);
		}

		if (getDriveRightBumper()) {
			boolean isDriverVisionOn = visionToggle.toggle();
			setVision(isDriverVisionOn);
		}
	}

	// fix these
	public double getGameOpThrottle() { return op_throttle; }
	public double getDriveVision() { return xDriveFrontBackSwitch; }

	/**
	 * Get and return Driver buttons
	 */
	public double getDriveThrottle() { return drive_throttle; }
	public double getDriveTurn() { return drive_turn; }
	public boolean getDriveButtonA() { return drive_buttonA; }
	public boolean getDriveButtonB() { return drive_buttonB; }
	public boolean getDriveButtonX() { return drive_buttonX; }
	public boolean getDriveButtonY() { return drive_buttonY; }
	public boolean getDriveLeftBumper() { return drive_leftBumper; }
	public boolean getDriveRightBumper() { return drive_rightBumper; }
	public boolean getDriveBackButton() { return drive_back; }
	public boolean getDriveStartButton() { return drive_start; }

	/**
	 * Get and return Operator buttons
	 */
	public boolean getOpButtonA() { return op_buttonA; }
	public boolean getOpButtonB() { return op_buttonB; }
	public boolean getOpButtonX() { return op_buttonX; }
	public boolean getOpButtonY() { return op_buttonY; }
	public boolean getOpLeftBumper() { return op_leftBumper; }
	public boolean getOpRightBumper() { return op_rightBumper; }
	public boolean getOpBackButton() { return op_back; }
	public boolean getOpStartButton() { return op_start; }
	
	/**
	 * Toggle LED and Driver Vision
	 * Move these?
	 */
	public void setLed(boolean isLedOn) {
		if (isLedOn) {
			limelight.setLedMode(LightMode.eOff);
			ledStatus = false;
			return;
		}
		limelight.setLedMode(LightMode.eOn);
		ledStatus = true;
	}

	public void setVision(boolean isDriverVisionOn) {
		if (isDriverVisionOn) {
			limelight.setCameraMode(CameraMode.eDriver); 
			visionStatus = false;
			return;
		}
		limelight.setCameraMode(CameraMode.eVision);
		visionStatus = true;
	}
}
