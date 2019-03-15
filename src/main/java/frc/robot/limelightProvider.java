package frc.robot;

/**
 * A singleton that provides centralized instances of limelights
 */
class LimelightProvider
{
    private final String backLimelightName = "limelight-ten";
    private final String frontLimelightName = "limelight-eleven";
    private static Limelight backLimelight = null;
    private static Limelight frontLimelight = null;
    private static LimelightProvider thisLimelightProvider = null;

    // private constructor means clients make their own LimelightProviders using "new"
    private LimelightProvider() {
        backLimelight = new Limelight(backLimelightName);
        frontLimelight = new Limelight(frontLimelightName);
    }

    /**
     * Get the sole provider
     */
    public static LimelightProvider getProvider()
    {
        return thisLimelightProvider == null
            ? new LimelightProvider()
            : thisLimelightProvider;
    }

    // Limelight retrieval
    public Limelight getBackLimelight() { return backLimelight; }
    public Limelight getFrontLimelight() { return frontLimelight; }
    public Limelight getCurrentLimelight(boolean isReverseDrive)
    {
        return isReverseDrive
            ? backLimelight
            : frontLimelight;
    }
}
