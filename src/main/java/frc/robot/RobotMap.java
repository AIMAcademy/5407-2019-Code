/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class RobotMap {

    // Digital IO Port for switching between robot flow and kcap chassis
    public DigitalInput flowKcap;
    public static final int flowKcapPort = 0;

    // Difference between power of climb motors to the brushless (needs to be
    // adjusted or removed)
    double climbVsDrive = 0.5;

    // Left Side Speed Controlers
    private int leftMotorID_1 = 1;
    private int leftMotorID_2 = 2;

    // Right Side Speed Controlers
    private int rightMotorID_1 = 0;
    private int rightMotorID_2 = 3;

    // Climber Sparks
    private int dartSpark_ID = 0;
    private int climberLegs_ID = 3;
    private int leftClimberWheel_ID = 2;
    private int rightClimberWheel_ID = 1;

    //More
    private int arm_ID = 4;
    private int leftPickupWheel_ID = 5;
    private int rightPickupWheel_ID = 6;

    public CANSparkMax leftMotor_1;
    public CANSparkMax leftMotor_2;

    public CANSparkMax rightMotor_1;
    public CANSparkMax rightMotor_2;

    public Spark climberArm;
    public Spark climberLegs;
    public Spark leftClimberWheel;
    public Spark rightClimberWheel;

    public Spark arm;
    public Spark leftPickupWheel;
    public Spark rightPickupWheel;

    public SpeedControllerGroup speedControllerGroupLeft, speedControllerGroupRight;

    public DifferentialDrive drive;
    public DifferentialDrive climbDrive;

    public RobotMap() {
        flowKcap = new DigitalInput(flowKcapPort);

        if (getFlowKcap()) {
            // Use Flow chassis

            return;
        }
        
        // Use Kcap Chassis
        leftMotor_1 = new CANSparkMax(leftMotorID_1, MotorType.kBrushless);
        leftMotor_2 = new CANSparkMax(leftMotorID_2, MotorType.kBrushless);

        rightMotor_1 = new CANSparkMax(rightMotorID_1, MotorType.kBrushless);
        rightMotor_2 = new CANSparkMax(rightMotorID_2, MotorType.kBrushless);

        climberArm = new Spark(dartSpark_ID);
        climberLegs = new Spark(climberLegs_ID);

        arm = new Spark(arm_ID);
        leftPickupWheel = new Spark(leftPickupWheel_ID);
        rightPickupWheel = new Spark(rightPickupWheel_ID);

        leftClimberWheel = new Spark(leftClimberWheel_ID);
        rightClimberWheel = new Spark(rightClimberWheel_ID);

        speedControllerGroupLeft = new SpeedControllerGroup(leftMotor_1, leftMotor_2);
        speedControllerGroupRight = new SpeedControllerGroup(rightMotor_1, rightMotor_2);

        drive = new DifferentialDrive(speedControllerGroupLeft, speedControllerGroupRight);
        climbDrive = new DifferentialDrive(leftClimberWheel, rightClimberWheel);
    }

    /*
     * Brushed motor setting can be selected manualy on the controlers... maybe this
     * will help save the motor if someone messes with it... or just waste bandwith
     */
    public void motorSafetyCheck() {
        if (leftMotor_1.getMotorType() == MotorType.kBrushed || leftMotor_2.getMotorType() == MotorType.kBrushed
                || rightMotor_1.getMotorType() == MotorType.kBrushed
                || rightMotor_2.getMotorType() == MotorType.kBrushed) {
            System.out.println("Brushed motor selected");
            System.exit(0);
        }
    }

    public boolean getFlowKcap() {
        return !flowKcap.get();
    }
}
