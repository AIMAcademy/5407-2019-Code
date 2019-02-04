/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Limelight.LightMode;

/**
 * Add your docs here.
 */
public class OI {
	public boolean ledToggle = true;

	private Joystick driveStick;
	private double throttle;
	private double turn;

	private Joystick opStick;
	private double climbThrottle;
	private double climbTurn;
	private boolean op_Control;
	private boolean op_yButton;
	private boolean op_bButton;
	private boolean op_aButton;
	private boolean op_xButton;
	private boolean op_leftBumper;
	private boolean op_Start;

	private Limelight limelight;

	public OI(Limelight limelight) {
		this.limelight = limelight;

		driveStick = new Joystick(0);
		opStick = new Joystick(1);
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

		op_yButton = opStick.getRawButton(4);
		op_bButton = opStick.getRawButton(2);
		op_aButton = opStick.getRawButton(1);
		op_xButton = opStick.getRawButton(3);
		op_leftBumper = opStick.getRawButton(5);
		op_Control = opStick.getRawButton(7);
		op_Start = opStick.getRawButton(8);

		if(opStick.getRawButtonPressed(8)) {
			ledToggle = !ledToggle;
			ledToggleButton();
		}
	}

	public double getThrottle() { return throttle; }
	public double getTurn() { return turn; }
	public double getClimbThrottle() { return climbThrottle; }
	public double getClimbTurn() { return climbTurn; }
	public boolean getRetractLegsButton() { return op_yButton; }
	public boolean getExtendArmButton() { return op_bButton; }
	public boolean getExtendLegsButton() { return op_aButton; }
	public boolean getRetractArmButton() { return op_xButton; }
	public boolean getOPControlButton() { return op_Control; }
	public boolean getOpLeftBumper() { return op_leftBumper; }
	public boolean getOPStart() { return op_Start; }

	public void ledToggleButton() {
		if (ledToggle) {
			limelight.setLedMode(LightMode.eOff);
		} else if (!ledToggle) {
			limelight.setLedMode(LightMode.eOn);
		}
	}
}
