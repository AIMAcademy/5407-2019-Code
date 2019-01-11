/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Add your docs here.h
 */
public class OI {

	public Joystick driveStick;
	private double throttle;
    private double turn;
    
    public OI() {
        driveStick = new Joystick(0);
    }

    public void readValues() {
		if (driveStick.getY() < -0.1 || driveStick.getY() > 0.1) {
			throttle = driveStick.getY();
		} 
		else {
			throttle = 0;
		}
		
		if (driveStick.getX() < -0.1  || driveStick.getX() > 0.1) {
			turn = driveStick.getX();
		} else {
			turn = 0;
		}
				
	}
	
	public double getThrottle() {
		return throttle;
	}
	
	public double getTurn() {
		return turn;
	}

}
