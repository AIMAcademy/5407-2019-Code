/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DriverStation;

public class Sensors {
  private AHRS ahrs;
  private double followAngle;

  private int armPotentiometer_ID = 0;
  private int smallWinchPotentiometer_ID = 1;

  private DigitalInput pixy2;
  private int pixy2_ID = 1;
  
  private Potentiometer s_WinchPot;
  private double s_winchPotValue;

  public final double kAngleSetpoint = 0.0;
  public final double kP = 0.05; // propotional turning constant
  public final double kVoltsPerDegreePerSecond = 0.0128; //adjust

  private Potentiometer armPot;
  private double armDegrees;
  private double armHeight;
  // public CANEncoder leftEncoder = new CANEncoder(robotmap.leftMotor_1);
  // public CANEncoder rightEncoder = new CANEncoder(robotmap.rightMotor_1);

  public Sensors() {

    try {
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex ) {
      DriverStation.reportError("Error instantiating navX MXP: " + ex.getMessage(), true);
    }

    AnalogInput ai = new AnalogInput(armPotentiometer_ID);
    armPot = new AnalogPotentiometer(ai, 1000, 0);

    AnalogInput ai2 = new AnalogInput(smallWinchPotentiometer_ID);
    s_WinchPot = new AnalogPotentiometer(ai2, 1000, 0);

    pixy2 = new DigitalInput(pixy2_ID);
  }

  public boolean getPixyOutput() {
    return pixy2.get();
  }

  public void setFollowAngleNAVX(double offset){
		this.followAngle = this.ahrs.getAngle() + offset;
	}
	
	// Makes public and gets the follow angle
	public double getFollowAngleNAVX() {
		return this.followAngle;
	}
	
	// Makes public and gets the present angles
	public double getPresentAngleNAVX(){
		return this.ahrs.getAngle();
  }

  public void zeroNAVX() {
    ahrs.reset();
  }

  public double getArmPotValue() {
    armDegrees = armPot.get();
    return armDegrees;
  }

  public double getArmHeight() {
    // double stringlength = 24;
    // double inperDegree;

    // inperDegree = stringlength / 360;
    // armHeight = getArmPotValue() * inperDegree;
    armHeight = 1000 - getArmPotValue();
    return armHeight;
  }

  public double getSmallWinchPot() {
    s_winchPotValue = 1000 - s_WinchPot.get();
    return s_winchPotValue;
  }
}
