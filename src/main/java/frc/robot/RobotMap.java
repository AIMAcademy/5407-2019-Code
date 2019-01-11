/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


/**
 * Add your docs here.
 */

public class RobotMap {

     //Left Side Speed Controlers
int leftTalonID = 16;
int leftVictorID_1 = 22;
int leftVictorID_2 = 21;
//Right Side Speed Controlers
int rightTalonID = 13;
int rightVictorID_1 = 17;
int rightVictorID_2 = 18;

public WPI_TalonSRX leftMiniCim1, rightMiniCim1;
public WPI_VictorSPX leftMiniCim2, leftMiniCim3, rightMiniCim2, rightMiniCim3;

public DifferentialDrive drive;

    public RobotMap() {

        leftMiniCim1 = new WPI_TalonSRX(leftTalonID);
        rightMiniCim1 = new WPI_TalonSRX(rightTalonID);
        
        leftMiniCim2 = new WPI_VictorSPX(leftVictorID_1);
        leftMiniCim3 = new WPI_VictorSPX(leftVictorID_2);
        rightMiniCim2 = new WPI_VictorSPX(rightVictorID_1);
        rightMiniCim3 = new WPI_VictorSPX(rightVictorID_2);

        leftMiniCim2.follow(leftMiniCim1);
	    leftMiniCim3.follow(leftMiniCim1);	
	    rightMiniCim2.follow(rightMiniCim1);
	    rightMiniCim3.follow(rightMiniCim1);

        drive = new DifferentialDrive(leftMiniCim1, rightMiniCim1);
    }

}
