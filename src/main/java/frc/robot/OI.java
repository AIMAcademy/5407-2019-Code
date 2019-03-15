/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Add your docs here.
 */
public class OI {
	private XboxController driveStick;
	private double drive_throttle;
	private double drive_turn;
	private boolean drive_buttonA;
	private boolean drive_buttonB;
	private boolean drive_buttonX;
	private boolean drive_buttonY;
	private boolean drive_leftBumper;
	private boolean drive_leftBumperPressed;
	private boolean drive_rightBumper;
	private boolean drive_rightBumperPressed;
	private boolean drive_back;
	private boolean drive_start;
	private boolean drive_leftTrigger;
	private boolean drive_rightTrigger;	

	private XboxController opStick;
	private double op_throttle;
	private double op_rightThrottle;
	private boolean op_buttonA;
	private boolean op_buttonB;
	private boolean op_buttonX;
	private boolean op_buttonY;
	private boolean op_buttonPressedA;
	private boolean op_buttonPressedB;
	private boolean op_buttonPressedX;
	private boolean op_buttonPressedY;
	private boolean op_leftBumper;
	private boolean op_rightBumper;
	private boolean op_back;
	private boolean op_start;
	private boolean op_leftTrigger;
	private boolean op_rightTrigger;
	private boolean op_dpadUp;
	private boolean op_dpadRight;
	private boolean op_dpadDown;
	private boolean op_dpadLeft;

	private Joystick joystickEmulator;
	private boolean em_buttonSwitch1;
	private boolean em_buttonSwitch2;
	private boolean em_buttonPressed3;

	public OI() {
		driveStick = new XboxController(0);
		opStick = new XboxController(1);
		joystickEmulator = new Joystick(2);
	}

	public void readValues() {	
		/**
		 * Read Driver joystick axes
		 */
		// Driver Left Stick Y Axis 1 for Driver Throttle
		if (driveStick.getRawAxis(1) < -0.2 || driveStick.getRawAxis(1) > 0.2) {
			drive_throttle = -driveStick.getRawAxis(1);
		} else {
			drive_throttle = 0;
		}
		// Driver Left Stick R Axis 4 for Driver Turn
		if (driveStick.getRawAxis(4) < -0.2 || driveStick.getRawAxis(4) > 0.2) {
			drive_turn = driveStick.getRawAxis(4);
		} else {
			drive_turn = 0;
		}
		// Driver Left Trigger Axis 2
		drive_leftTrigger = (driveStick.getRawAxis(2) > .5);
		// Driver Right Trigger Axis 3
		drive_rightTrigger = (driveStick.getRawAxis(3) > .5);

		/**
		 * Read Operator joystick axes
		 */
		// Operator Left Stick Y Axis 1 for Operator Throttle
		if (opStick.getRawAxis(1) < -0.2 || opStick.getRawAxis(1) > 0.2) {
			op_throttle = -opStick.getRawAxis(1);
		} else {
			op_throttle = 0;
		}
		// Operator Right Stick Y Axis 5 for Operator Right Throttle
		if (opStick.getRawAxis(5) < -0.2 || opStick.getRawAxis(5) > 0.2) {
			op_rightThrottle = -opStick.getRawAxis(5);
		} else {
			op_rightThrottle = 0;
		}		
		// Operator Left Trigger Axis 2
		op_leftTrigger = (opStick.getRawAxis(2) > .5);
		// Operator Right Trigger Axis 3
		op_rightTrigger = (opStick.getRawAxis(3) > 0.5);
		// Operator Dpad Up 
		op_dpadUp = ((opStick.getPOV(0) >= 315 && opStick.getPOV(0) <= 45) && opStick.getPOV(0) != -1);
		// Operator Dpad Right
		op_dpadRight = ((opStick.getPOV(0) >= 45 && opStick.getPOV(0) <= 135) && opStick.getPOV(0) != -1);
		// Operator Dpad Down
		op_dpadDown = ((opStick.getPOV(0) >= 135 && opStick.getPOV(0) <= 225) && opStick.getPOV(0) != -1);
		// Operator Dpad Left
		op_dpadLeft = ((opStick.getPOV(0) >= 225 && opStick.getPOV(0) <= 315) && opStick.getPOV(0) != -1);

		/**
		 * Read Driver joystick buttons
		 */
		drive_buttonA = driveStick.getRawButton(1);
		drive_buttonB = driveStick.getRawButton(2);
		drive_buttonX = driveStick.getRawButton(3);
		drive_buttonY = driveStick.getRawButton(4);
		drive_leftBumperPressed = driveStick.getRawButtonPressed(5);
		drive_rightBumperPressed = driveStick.getRawButtonPressed(6);
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
		op_buttonPressedA = opStick.getRawButtonPressed(1);
		op_buttonPressedB = opStick.getRawButtonPressed(2);
		op_buttonPressedX = opStick.getRawButtonPressed(3);
		op_buttonPressedY = opStick.getRawButtonPressed(4);
		op_leftBumper = opStick.getRawButton(5);
		op_rightBumper = opStick.getRawButton(6);
		op_back = opStick.getRawButton(7);
		op_start = opStick.getRawButton(8);

		/**
		 * Read Joystick Emulator buttons
		 */
		em_buttonSwitch1 = joystickEmulator.getRawButton(1);
		em_buttonSwitch2 = joystickEmulator.getRawButton(2);
		em_buttonPressed3 = joystickEmulator.getRawButtonPressed(3);
	}

	/**
	 * Get and return Driver buttons
	 */
	public double getDriveThrottle() { return drive_throttle; }
	public double getDriveTurn() { return drive_turn; }
	public boolean getDriveButtonA() { return drive_buttonA; }
	public boolean getDriveButtonB() { return drive_buttonB; }
	public boolean getDriveButtonX() { return drive_buttonX; }
	public boolean getDriveButtonY() { return drive_buttonY; }
	public boolean getDriveLeftBumperPressed() { return drive_leftBumperPressed; }
	public boolean getDriveLeftBumper() { return drive_leftBumper; }
	public boolean getDriveRightBumperPressed() { return drive_rightBumperPressed; }
	public boolean getDriveRightBumper() { return drive_rightBumper; }
	public boolean getDriveBackButton() { return drive_back; }
	public boolean getDriveStartButton() { return drive_start; }
	public boolean getDriveLeftTrigger() { return drive_leftTrigger; }
	public boolean getDriveRightTrigger() { return drive_rightTrigger; }

	/**
	 * Get and return Operator buttons
	 */
	public double getOpThrottle() { return op_throttle; }
	public double getOpRightThrottle() { return op_rightThrottle; }
	public boolean getOpButtonA() { return op_buttonA; }
	public boolean getOpButtonB() { return op_buttonB; }
	public boolean getOpButtonX() { return op_buttonX; }
	public boolean getOpButtonY() { return op_buttonY; }
	public boolean getOpButtonPressedA() { return op_buttonPressedA; }
	public boolean getOpButtonPressedB() { return op_buttonPressedB; }
	public boolean getOpButtonPressedX() { return op_buttonPressedX; }
	public boolean getOpButtonPressedY() { return op_buttonPressedY; }
	public boolean getOpLeftBumper() { return op_leftBumper; }
	public boolean getOpRightBumper() { return op_rightBumper; }
	public boolean getOpBackButton() { return op_back; }
	public boolean getOpStartButton() { return op_start; }
	public boolean getOpLeftTrigger() { return op_leftTrigger; }
	public boolean getOpRightTrigger() { return op_rightTrigger; }
	public boolean getOpDpadUp() { return op_dpadUp; }
	public boolean getOpDpadRight() { return op_dpadRight; }
	public boolean getOpDpadDown() { return op_dpadDown; }
	public boolean getOpDpadLeft() { return op_dpadLeft; }

	/**
	 * Get and return Joystick Emulator buttons
	 */
	public boolean getJoystickEmulatorButtonSwitch1() { return em_buttonSwitch1; }
	public boolean getJoystickEmulatorButtonSwitch2() { return em_buttonSwitch2; }
	public boolean getJoystickEmulatorButtonPressed3() { return em_buttonPressed3; }
}
