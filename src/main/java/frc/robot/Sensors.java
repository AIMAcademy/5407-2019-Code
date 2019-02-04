/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;

public class Sensors {
  private AHRS ahrs;
  private RobotMap robotmap;
  private double followAngle;
  // private double rotateToAngleRate;

  public final double kAngleSetpoint = 0.0;
  public final double kP = 0.005; // propotional turning constant
  public final double kVoltsPerDegreePerSecond = 0.0128; //adjust

  // public CANEncoder leftEncoder = new CANEncoder(robotmap.leftMotor_1);
  // public CANEncoder rightEncoder = new CANEncoder(robotmap.rightMotor_1);

  public Sensors() {

    try {
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex ) {
      DriverStation.reportError("Error instantiating navX MXP: " + ex.getMessage(), true);
    }
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
  
}