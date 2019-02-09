package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;

/**
 * Provides essential calculations for the robot.
 * This is immediately accessible after import.
 * NOTE: Do not instantiate using "new". This will cause a runtime error.
 */
public final class Calculations {
    private final static double h2 = 28.5; // inches from floor to center of target

    // Update these when camera moves.
    private final static double a1 = 0.3859; // the camera's mounting angle in radians
    private final static double h1 = 9; // inches from floor to camera lens

    // Constructor
    private Calculations() {}

    /**
     * Calculates the aim.
     * @param headingError The heading error
     * @return The steering adjustment
     */
    public static double getAim(double headingError) {
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
     * Get the current, hardcoded mounting angle.
     * @return The angle we think the camera is mounted at
     */
    public static double getHardMountingAngle() {
        return a1;
    }

    /**
     * Attempts to calculate the current mounting angle.
     * @param cameraTargetYAxis The Y axis of the camera's current target
     * @param distance The distance between the camera and its current target
     * @return The angle the robot believes the camera is mounted at
     */
    public static double getSoftMountingAngle(NetworkTableEntry cameraTargetYAxis, double distance) {
        final double angleToTarget = Math.toRadians(cameraTargetYAxis.getDouble(0.0));
        final double mountingAngle = Math.atan((h2 - h1) / distance) - angleToTarget;
        return mountingAngle;
    }

    /**
     * Calculates the current range.
     * @param cameraTargetYAxis The Y axis of the camera's current target
     * @param mountingAngle The angle at which the camera is mounted
     * @return The distance between the camera and its current target
     */
    public static double getRange(NetworkTableEntry cameraTargetYAxis) {
        final double angleToTarget = Math.toRadians(cameraTargetYAxis.getDouble(0.0));
        final double distance = (h2 - h1) / Math.tan(a1 + angleToTarget);
        return distance;
    }
}
