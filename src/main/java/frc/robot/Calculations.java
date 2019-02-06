package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;

/**
 * Provides essential calculations for the robot.
 * This is immediately accessible after import.
 * NOTE: Do not instantiate using "new". This will cause a runtime error.
 */
public final class Calculations {
    // Constructor
    private Calculations() {}

    /**
     * Calculates the current range.
     * @param cameraTargetYAxis The Y axis of the camera's current target
     * @return The distance between the camera and its current target
     */
    public static double getRange(NetworkTableEntry cameraTargetYAxis) {
        // The value of h2 may change as we adjust the physical camera.
        final int h2 = 39;
        final double h1 = 23.375;
        final double radians = Math.toRadians(cameraTargetYAxis.getDouble(0.0));
        final int a2 = 5;

        double distance = ((h2 - h1) / Math.tan(radians + a2));
        return distance;
      }
}
