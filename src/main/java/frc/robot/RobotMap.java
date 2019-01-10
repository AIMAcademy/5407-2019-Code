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
        drive = new DifferentialDrive(PWMVictorSPX(0), PWMVictorSPX(1));
    }

}
