package frc.robot;

/**
 * Robot actions
 */
public class Actions {

    private Air air;
    private OI oi;
    private RobotMap robotmap;

    private double gameOpThrottle;

    public Actions(Air air, OI oi, RobotMap robotmap) {
        this.air = air;
        this.oi = oi;
        this.robotmap = robotmap;
    }

    public void gameOp() {
        gameOpThrottle = oi.getGameOpThrottle();
        
        // Operator controls during game operations
        if (oi.getOpYButton()) { // Y Button = Arm
            robotmap.arm.set(gameOpThrottle);
        } else if (oi.getOpAButton()) { // A Button = Roller
            robotmap.rollerWheel.set(gameOpThrottle);
        } else if (oi.getOpBButton()) { // B Button = S-Winch
            robotmap.smallWinchMotor.set(gameOpThrottle);
        } else if (oi.getOpXButton()) { // X Button = Tung
            robotmap.tungMotor.set(gameOpThrottle * .75);
        }

        // DriveStick buttons to extend solenoids while held
        if (oi.getDSbutton3()) { // DriveStick button 3 = Back hatch pistons
          air.setSolenoid3(true);
        } else {
          air.setSolenoid3(false);
        }

        // DriveStick buttons to toggle solenoids
        if (oi.getDSbutton2()) { // DriveStick button 2 = Arm grabber
            final boolean flippedStatus = !air.getSolenoid0();
            air.setSolenoid0(flippedStatus);
        }
        if (oi.getDSbutton4()) { // DriveStick button 4 = Fangs
            final boolean flippedStatus = !air.getSolenoid1();
            air.setSolenoid1(flippedStatus);
        }
        if (oi.getDSbutton5()) { // DriveStick button 5 = Tung
            final boolean flippedStatus = !air.getSolenoid2();
            air.setSolenoid2(flippedStatus);
        }
    }

    public void endGameOp() {
        robotmap.drive.arcadeDrive(-oi.getThrottle(), oi.getTurn());
    
        // Operator controls during end-game operations
        if (oi.getDSbutton6()) {
          robotmap.leftClimberWheel.set(-1.0);
          robotmap.rightClimberWheel.set(1.0);
        } else { 
          robotmap.leftClimberWheel.set(0.0);
          robotmap.rightClimberWheel.set(0.0);
        }

        if (oi.getOpBButton()) {
          robotmap.climberArm.set(1);
        } else if (oi.getOpXButton()) {
          robotmap.climberArm.set(-1);
        } else {
          robotmap.climberArm.set(0);
        }
    
        if (oi.getOpAButton()) {
          robotmap.climberLegs.set(-1);
        } else if (oi.getOpYButton()) {
          robotmap.climberLegs.set(1);
        } else {
          robotmap.climberLegs.set(0);
        }

        robotmap.arm.set(oi.getBothTriggers());
    }
}
