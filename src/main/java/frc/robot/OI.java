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
	private Toggle ledToggle;
	private Toggle visionToggle;

	private Joystick driveStick;
	private double throttle;
	private double turn;
	private boolean ds_2Button;
	private boolean ds_3Button;
	private boolean ds_4Button;
	private boolean ds_5Button;
	private boolean ds_6Button;

	private Joystick xboxDriveStick;
	private double xDriveThrottle;
	private double xDriveTurn;
	private boolean xDriveButtonY;
	private boolean xDriveRightBumper;

	private Joystick opStick;
	private double climbThrottle;
	private double climbTurn;
	private boolean op_Control;
	private boolean op_yButton;
	private boolean op_bButton;
	private boolean op_aButton;
	private boolean op_xButton;
	private boolean op_leftBumper;
	private boolean op_rightBumper;
	private boolean op_Start;
	private double op_BothTriggers;
	private double gameOpThrottle;

	public boolean ledStatus;
	public boolean visionStatus;

	private Limelight limelight;

	public OI(Limelight limelight) {
		this.limelight = limelight;
		ledToggle = new Toggle();
		visionToggle = new Toggle();

		driveStick = new Joystick(0);
		opStick = new Joystick(1);
		xboxDriveStick = new Joystick(2);
	}

	public void readValues() {
		if (driveStick.getY() < -0.1 || driveStick.getY() > 0.1) {
			throttle = driveStick.getY();
		} else {
			throttle = 0;
		}

		if (driveStick.getX() < -0.1 || driveStick.getX() > 0.1) {
			turn = driveStick.getX();
		} else {
			turn = 0;
		}

		// Right Stick Y Axis
		if (opStick.getRawAxis(5) < -0.1 || opStick.getRawAxis(5) > 0.1) {
			climbThrottle = opStick.getRawAxis(5);
		} else {
			climbThrottle = 0;
		}

		// Right Stick X Axis
		if (opStick.getRawAxis(4) < -0.1 || opStick.getRawAxis(4) > 0.1) {
			climbTurn = opStick.getRawAxis(4);
		} else {
			climbTurn = 0;
		}

		if (opStick.getRawAxis(3) >= 0.2) {
			op_BothTriggers = opStick.getRawAxis(3);
		} else if (opStick.getRawAxis(2) >= 0.2) {
			op_BothTriggers = -opStick.getRawAxis(2);
		} else {
			op_BothTriggers = 0;
		}
		
		// Left Stick Y Axis
		if (opStick.getRawAxis(1) < -0.2 || opStick.getRawAxis(1) > 0.2) {
			gameOpThrottle = -opStick.getRawAxis(1);
		} else {
			gameOpThrottle = 0;
		}

		xDriveThrottle = xboxDriveStick.getRawAxis(1);
		xDriveTurn = xboxDriveStick.getRawAxis(4);
		xDriveButtonY = xboxDriveStick.getRawButton(4);
		xDriveRightBumper = xboxDriveStick.getRawButtonPressed(6);

		ds_2Button = driveStick.getRawButtonPressed(2);
		ds_3Button = driveStick.getRawButton(3);
		ds_4Button = driveStick.getRawButtonPressed(4);
		ds_5Button = driveStick.getRawButtonPressed(5);
		ds_6Button = driveStick.getRawButtonPressed(6);

		op_yButton = opStick.getRawButton(4);
		op_bButton = opStick.getRawButton(2);
		op_aButton = opStick.getRawButton(1);
		op_xButton = opStick.getRawButton(3);
		op_leftBumper = opStick.getRawButton(5);
		op_rightBumper = opStick.getRawButton(6);
		op_Control = opStick.getRawButton(7);
		op_Start = opStick.getRawButton(8);


		if (opStick.getRawButtonPressed(8)) {
			boolean isLedOn = ledToggle.toggle();
			setLed(isLedOn);
		}

		if (getXDriveBumper()) {
			boolean isDriverVisionOn = visionToggle.toggle();
			setVision(isDriverVisionOn);
		}
	}

	public boolean getDSbutton2() { return ds_2Button; }
	public boolean getDSbutton3() { return ds_3Button; }
	public boolean getDSbutton4() { return ds_4Button; }
	public boolean getDSbutton5() { return ds_5Button; }
	public boolean getDSbutton6() { return ds_6Button; }

	public double getGameOpThrottle() { return gameOpThrottle; }

	public double getThrottle() { return throttle; }
	public double getTurn() { return turn; }
	public double getClimbThrottle() { return climbThrottle; }
	public double getClimbTurn() { return climbTurn; }
	public boolean getOpYButton() { return op_yButton; }
	public boolean getOpBButton() { return op_bButton; }
	public boolean getOpAButton() { return op_aButton; }
	public boolean getOpXButton() { return op_xButton; }
	public boolean getOPControlButton() { return op_Control; }
	public boolean getOpLeftBumper() { return op_leftBumper; }
	public boolean getOpRightBumper() { return op_rightBumper; }
	public boolean getOPStart() { return op_Start; }
	public double getBothTriggers() { return op_BothTriggers; }

	public double getXDriveTurn() { return xDriveTurn; }
	public double getXDriveThrottle() { return xDriveThrottle; }
	public boolean getXDriveBumper() { return xDriveRightBumper; }
	public boolean getxDriveButtonY() { return xDriveButtonY; }
	
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
