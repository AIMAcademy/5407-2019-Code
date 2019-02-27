/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

public class Air {
    private final int solenoid0_ID = 0; // Beak
    private final int solenoid1_ID = 1; // Roller Fangs
    private final int solenoid2_ID = 2; // Tung
    private final int solenoid3_ID = 3; // Back hatch tung pistons
    private final int solenoid4_ID = 4; // Cargo claw

    private final Solenoid solenoid0;
    private final Solenoid solenoid1;
    private final Solenoid solenoid2;
    private final Solenoid solenoid3;
    private final Solenoid solenoid4;

    public Air() {
        solenoid0 = new Solenoid(solenoid0_ID);
        solenoid1 = new Solenoid(solenoid1_ID);
        solenoid2 = new Solenoid(solenoid2_ID);
        solenoid3 = new Solenoid(solenoid3_ID);
        solenoid4 = new Solenoid(solenoid4_ID);

        airInit();
    }

    public void airInit() {
        solenoid0.set(false);
        solenoid1.set(false);
        solenoid2.set(false);
        solenoid3.set(false);
        solenoid4.set(false);
    }

    public boolean getSolenoid0() { return solenoid0.get(); }
    public boolean getSolenoid1() { return solenoid1.get(); }
    public boolean getSolenoid2() { return solenoid2.get(); }
    public boolean getSolenoid3() { return solenoid3.get(); }
    public boolean getSolenoid4() { return solenoid4.get(); }

    public void setSolenoid0(boolean isOn) { solenoid0.set(isOn); }
    public void setSolenoid1(boolean isOn) { solenoid1.set(isOn); }
    public void setSolenoid2(boolean isOn) { solenoid2.set(isOn); }
    public void setSolenoid3(boolean isOn) { solenoid3.set(isOn); }
    public void setSolenoid4(boolean isOn) { solenoid4.set(isOn); }
}
