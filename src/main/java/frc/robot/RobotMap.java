/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.VictorSP;

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

    /*
     * Motor ports
     */
    // Left Side Speed Controlers
    private int leftMotorID_0 = 0;
    private int leftMotorID_1 = 1;
    private int leftMotorID_2 = 2;
    // Right Side Speed Controlers
    private int rightMotorID_0 = 3;
    private int rightMotorID_1 = 4;
    private int rightMotorID_2 = 5;
    // Climber Sparks
    private int climberArm_ID = 6; // FRT-DART (CAN)
    private int climberLegs_ID = 7; // LEG-LIFT (CAN)
    private int leftClimberWheel_ID = 0; // LLEGGING (PWM)
    private int rightClimberWheel_ID = 1; // RLEGGING (PWM)
    // Arm components
    private int arm_ID = 5; // (PWM)
    private int rollerWheel_ID = 4; // ROLLER (PWM)
    private int smallWinchMotor_ID = 2; // S-WINCH (PWM)
    // The Tung
    private int tungMotor_ID = 3; // THE TUNG (PWM)

    /*
     * Motor Controllers
     */
    // Powertrain
    public CANSparkMax leftMotor_0;
    public CANSparkMax leftMotor_1;
    public CANSparkMax leftMotor_2;
    public CANSparkMax rightMotor_0;
    public CANSparkMax rightMotor_1;
    public CANSparkMax rightMotor_2;
    // Climbing System
    public WPI_TalonSRX climberArm; // FRT-DART
    public WPI_TalonSRX climberLegs; // LEG-LIFT (Dart)
    public Spark leftClimberWheel; // LLEGGING
    public Spark rightClimberWheel; // RLEGGING
    // Arm System
    // public WPI_VictorSPX arm;
    public VictorSP arm;
    public Spark rollerWheel; // ROLLER
    public Spark smallWinchMotor; // S-WINCH
    // The Tung
    public Spark tungMotor; // THE TUNG
    // Flow Motors
    public WPI_TalonSRX leftTalon;
    public WPI_TalonSRX rightTalon;
    public WPI_VictorSPX leftVictor;
    public WPI_VictorSPX rightVictor;

    // Kcap Speed Controller Group
    public SpeedControllerGroup speedControllerGroupLeft, speedControllerGroupRight;
    
    // Differential drive variables
    public DifferentialDrive drive;
    public DifferentialDrive climbDrive;

    public RobotMap() {
        // Check for jumper cable to determine which chassis to drive
        flowKcap = new DigitalInput(flowKcapPort);

        if (getFlowKcap()) {
            // Use Flow chassis
            leftTalon = new WPI_TalonSRX(13);
            rightTalon = new WPI_TalonSRX(16);
            leftVictor = new WPI_VictorSPX(17);
            rightVictor = new WPI_VictorSPX(21);

            // Set slave motors
            leftVictor.follow(leftTalon);
            rightVictor.follow(rightTalon);

            drive = new DifferentialDrive(leftTalon, rightTalon);
            // climbDrive = new DifferentialDrive(leftTalon, rightTalon);

            return;
        }
        
        // Use Kcap Chassis
        leftMotor_0 = new CANSparkMax(leftMotorID_0, MotorType.kBrushless);
        leftMotor_1 = new CANSparkMax(leftMotorID_1, MotorType.kBrushless);
        leftMotor_2 = new CANSparkMax(leftMotorID_2, MotorType.kBrushless);
        rightMotor_0 = new CANSparkMax(rightMotorID_0, MotorType.kBrushless);
        rightMotor_1 = new CANSparkMax(rightMotorID_1, MotorType.kBrushless);
        rightMotor_2 = new CANSparkMax(rightMotorID_2, MotorType.kBrushless);

        // Climb system
        climberArm = new WPI_TalonSRX(climberArm_ID);
        climberLegs = new WPI_TalonSRX(climberLegs_ID);
        leftClimberWheel = new Spark(leftClimberWheel_ID);
        rightClimberWheel = new Spark(rightClimberWheel_ID);

        // Other components
        // arm = new WPI_VictorSPX(arm_ID);
        arm = new VictorSP(arm_ID);
        rollerWheel = new Spark(rollerWheel_ID);
        smallWinchMotor = new Spark(smallWinchMotor_ID);
        tungMotor = new Spark(tungMotor_ID);

        // Speed controller groups
        speedControllerGroupLeft = new SpeedControllerGroup(leftMotor_0, leftMotor_1, leftMotor_2);
        speedControllerGroupRight = new SpeedControllerGroup(rightMotor_0, rightMotor_1, rightMotor_2);

        drive = new DifferentialDrive(speedControllerGroupLeft, speedControllerGroupRight);
        // climbDrive = new DifferentialDrive(leftClimberWheel, rightClimberWheel);
    }

    /*
     * Brushed motor setting can be selected manualy on the controlers... maybe this
     * will help save the motor if someone messes with it... or just waste bandwith
     */
    public void motorSafetyCheck() {
        if (leftMotor_0.getMotorType() == MotorType.kBrushed
                || leftMotor_1.getMotorType() == MotorType.kBrushed
                || leftMotor_2.getMotorType() == MotorType.kBrushed
                || rightMotor_0.getMotorType() == MotorType.kBrushed
                || rightMotor_1.getMotorType() == MotorType.kBrushed
                || rightMotor_2.getMotorType() == MotorType.kBrushed) {
            System.out.println("Brushed motor selected");
            System.exit(0);
        }
    }

    /**
     * Returns which robot is being driven. True for Flow, False for Kcap.
     * @return A boolean that checks for a jumper cable in a digital IO port
     */
    public boolean getFlowKcap() {
        return !flowKcap.get();
    }
}
