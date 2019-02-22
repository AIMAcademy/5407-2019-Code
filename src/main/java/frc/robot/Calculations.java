package frc.robot;

import frc.robot.AimAndRange;

/**
 * Provides essential calculations for the robot.
 * This is immediately accessible after import.
 * NOTE: Do not instantiate using "new". This will cause a runtime error.
 */
public final class Calculations {
    // Front camera properties
    private final static double h2 = 28.5; // inches from floor to center of target
    // Update these when the front camera moves.
    private final static double a1 = 0.5017; // the camera's mounting angle in radians
    private final static double h1 = 8.75; // inches from floor to camera lens

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
    public static double getHeadingError(Double cameraTargetXAxis) {
        final double headingError = -cameraTargetXAxis;
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
    public static double getSoftMountingAngle(Double cameraTargetYAxis, double distance) {
        final double angleToTarget = Math.toRadians(cameraTargetYAxis);
        final double mountingAngle = Math.atan((h2 - h1) / distance) - angleToTarget;
        return mountingAngle;
    }

    /**
     * Calculates the current range.
     * @param cameraTargetYAxis The Y axis of the camera's current target
     * @param mountingAngle The angle at which the camera is mounted
     * @return The distance between the camera and its current target
     */
    public static double getRange(Double cameraTargetYAxis) {
        final double angleToTarget = Math.toRadians(cameraTargetYAxis);
        final double distance = (h2 - h1) / Math.tan(a1 + angleToTarget);
        final double correction = distance / 12 * 2;
        return distance - correction;
    }

    /**
     * Returns tracking data based on the back camera.
     * @param cameraTargetXAxis The X axis of the camera's current target
     * @param cameraTargetYAxis The Y axis of the camera's current target
     * @return An object containing drivingAdjust and steeringAdjust
     */
    public static AimAndRange getAimAndRangeBack(Double cameraTargetXAxis, Double cameraTargetYAxis) {
        // Code from http://docs.limelightvision.io/en/latest/cs_aimandrange.html
        double KpAim = 0.1;
        double KpDist = 0.1;
        double AimMinCmd = 0.05;

        // Aim error and distance error based on calibrated limelight cross-hair
        double aim_error = cameraTargetXAxis;
        double dist_error = -cameraTargetYAxis;

        // Steering adjust with a 0.2 degree deadband (close enough at 0.2deg)
        double steeringAdjustBack = KpAim * aim_error;
        if (aim_error > .2) {
          steeringAdjustBack = steeringAdjustBack + AimMinCmd;
        } else if (aim_error < -.2) {
          steeringAdjustBack = steeringAdjustBack - AimMinCmd;
        }

        // Distance adjust, drive to the correct distance from the goal
        double drivingAdjustBack = KpDist * dist_error;

        return new AimAndRange(drivingAdjustBack, steeringAdjustBack);
    }

    public static AimAndRange getAimAndRangeBackArea(Double cameraTargetXAxis, Double cameraTargetArea, boolean cameraTarget) {
        boolean thisCameraTarget = cameraTarget;

        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.05;                    // how hard to turn toward the target
        final double DRIVE_K = 0.15;                     // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 15.0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

        // Start with proportional steering
        double steer_cmd = cameraTargetXAxis * STEER_K;
        double steeringAdjustBack = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - cameraTargetArea) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
            drive_cmd = MAX_DRIVE;
        }

        double drivingAdjustBack = -drive_cmd;

        double drivingErrorThreshold = Math.abs(DESIRED_TARGET_AREA - cameraTargetArea);
        if (drivingErrorThreshold < 5) { drivingAdjustBack = 0.0; }

        if (thisCameraTarget == false)
        {
            // boolean m_LimelightHasValidTarget = false;
            drivingAdjustBack = 0.0;
            steeringAdjustBack = 0.0;
        }

        return new AimAndRange(drivingAdjustBack, steeringAdjustBack);
    }

    /**
     * Returns tracking data based on the front camera.
     * @param cameraTargetXAxis The X axis of the camera's current target
     * @param cameraTargetYAxis The Y axis of the camera's current target
     * @return An object containing drivingAdjust and steeringAdjust
     */
    public static AimAndRange getAimAndRangeFront(Double cameraTargetXAxis, Double cameraTargetYAxis) {
        // Code from http://docs.limelightvision.io/en/latest/cs_aimandrange.html
        double KpAim = 0.045;
        double KpDist = 0.017;
        double AimMinCmd = 0.095;
        double distMinCmd = 0.04;

        // Aim error and distance error based on calibrated limelight cross-hair
        double aim_error = cameraTargetXAxis;
        double dist_error = getRange(cameraTargetYAxis) - 20;

        // Steering adjust with a 0.2 degree deadband (close enough at 0.2deg)
        double steeringAdjustFront = KpAim * aim_error;
        if (aim_error > .2) {
          steeringAdjustFront = steeringAdjustFront + AimMinCmd;
        } else if (aim_error < -.2f) {
          steeringAdjustFront = steeringAdjustFront - AimMinCmd;
        }

        // Distance adjust, drive to the correct distance from the goal
        double drivingAdjustFront = (KpDist * dist_error) + distMinCmd;

        return new AimAndRange(drivingAdjustFront, steeringAdjustFront);
    }
}
