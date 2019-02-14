package frc.robot;

/**
 * A data model for transporting aim and range calculations
 */ 
public final class AimAndRange {
    private final double drivingAdjust;
    private final double steeringAdjust;

    public AimAndRange(double drivingAdjust, double steeringAdjust) {
        this.drivingAdjust = drivingAdjust;
        this.steeringAdjust = steeringAdjust;
    }

    public double getDrivingAdjust() { return drivingAdjust; }
    public double getSteeringAdjust() { return steeringAdjust; }
}
