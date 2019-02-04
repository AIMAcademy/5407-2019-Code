package frc.robot;

/**
 * Toggles a boolean. Defaults to "off" when instantiated.
 */
public class Toggle {
    private boolean isOnOrOff;

    public boolean get() {
        return isOnOrOff;
    }

    public boolean toggle() {
        isOnOrOff = !isOnOrOff;
        return isOnOrOff;
    }
}
