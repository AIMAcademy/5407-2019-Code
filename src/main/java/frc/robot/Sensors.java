/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.AnalogGyro;

public class Sensors {

    RobotMap robotmap;

    public final double kAngleSetpoint = 0.0;
    public final double kP = 0.005; // propotional turning constant
  
    public final double kVoltsPerDegreePerSecond = 0.0128; //adjust
  
    private static final int kGyroPort = 0;
  
    public final AnalogGyro gyro = new AnalogGyro(kGyroPort);
  
    //public CANEncoder leftEncoder = new CANEncoder(robotmap.leftMotor_1);
   // public CANEncoder rightEncoder = new CANEncoder(robotmap.rightMotor_1);

    public void sensors() {
    }

  }