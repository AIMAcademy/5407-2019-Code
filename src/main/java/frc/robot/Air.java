/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

public class Air {
    private int piston0_ID = 0; // Arm tri-grabber
    private int piston1_ID = 1; // Back hatch pistons // make these while held and then return
    private int piston2_ID = 2; // Back hatch pistons
    private int piston3_ID = 3; // Roller deploy
    private int piston4_ID = 4; // Roller Fangs

    private Solenoid piston0;
    private Solenoid piston1;
    private Solenoid piston2;
    private Solenoid piston3;
    private Solenoid piston4;

    public Air() {
        piston0 = new Solenoid(piston0_ID);
        piston1 = new Solenoid(piston1_ID);
        piston2 = new Solenoid(piston2_ID);
        piston3 = new Solenoid(piston3_ID);
        piston4 = new Solenoid(piston4_ID);

        airInit();
    }

    public void airInit() {
        piston0.set(false);
        piston1.set(false);
        piston2.set(false);
        piston3.set(false);
        piston4.set(false);
    }

    public void setSol(Solenoid sol, boolean isOn) {
        sol.set(isOn);
    }
}
