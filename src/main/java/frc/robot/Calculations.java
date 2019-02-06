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
     * Calculates the aim.
     * @param headingError The heading error
     * @return The steering adjustment
     */
    public static double getAim(double headingError) {
        // TODO: Rename Kp to something random people would understand.
        final double Kp = -0.15;
        final double steeringSpeed = 0.3;
        final double minCommand = 0.2f;
        double steeringAdjust = 0.0;

        if (-headingError > 1.0) {
            steeringAdjust = Kp * headingError - minCommand;
        }
        else if (-headingError < 1.0) {
            steeringAdjust = Kp * headingError + minCommand;
        }

        steeringAdjust = steeringAdjust * steeringSpeed;
        return steeringAdjust;
    }

    /**
     * Calculates the heading error.
     * @param cameraTargetXAxis The X axis of the camera's current target
     * @return The heading error
     */
    public static double getHeadingError(NetworkTableEntry cameraTargetXAxis) {
        final double headingError = -cameraTargetXAxis.getDouble(0.0);
        return headingError;
    }

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

        final double distance = ((h2 - h1) / Math.tan(radians + a2));
        return distance;
      }
}
