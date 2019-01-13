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

public class RobotMap {

//Left Side Speed Controlers
int leftMotorID_1 = 1;
int leftMotorID_2 = 1;
int leftMotorID_3 = 1;

//Right Side Speed Controlers
int rightMotorID_1 = 1;
int rightMotorID_2 = 1;
int rightMotorID_3 = 1;

public DifferentialDrive drive;

    public RobotMap() {


        //drive = new DifferentialDrive(leftMiniCim1, rightMiniCim1);
    }

}
