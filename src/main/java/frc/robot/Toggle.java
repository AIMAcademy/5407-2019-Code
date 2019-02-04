package frc.robot;

/**
 * Toggles a boolean. Defaults to "off" when instantiated.
 */
public class Toggle {
    private boolean isOn;

    public boolean get() {
        return isOn;
    }

    public boolean toggle() {
        isOn = !isOn;
        return isOn;
    }
}
