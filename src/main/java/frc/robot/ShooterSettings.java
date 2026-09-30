package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Dashboardで編集する射撃出力。符号は既存の向きを維持する。 */
public final class ShooterSettings {
    public static final String FLYWHEEL_OUTPUT = "Shooter/FlywheelOutput";
    public static final String FEED_OUTPUT = "Shooter/FeedOutput";

    public ShooterSettings() {
        init(FLYWHEEL_OUTPUT, 1.0);
        init(FEED_OUTPUT, 0.5);
    }

    public double flywheelOutput() {
        return read(FLYWHEEL_OUTPUT, 1.0);
    }

    public double feedOutput() {
        return read(FEED_OUTPUT, 0.5);
    }

    private static void init(String key, double defaultValue) {
        SmartDashboard.setDefaultNumber(key, defaultValue);
        SmartDashboard.setPersistent(key);
    }

    private static double read(String key, double defaultValue) {
        double value = SmartDashboard.getNumber(key, defaultValue);
        double bounded = Double.isFinite(value) ? MathUtil.clamp(value, 0, 1) : defaultValue;
        if (Double.compare(value, bounded) != 0) {
            SmartDashboard.putNumber(key, bounded);
        }
        return bounded;
    }
}
