package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** 走行設定。Teleopは毎周期、Autoは開始時に設定を読む。 */
public final class DriveSettings {
    public static final String MAX_SPEED_MPS = "Drive/MaxSpeedMps";
    public static final String MAX_ANGULAR_RATE = "Drive/MaxAngularRateRadPerSec";
    public static final String AUTO_SPEED_MPS = "Drive/AutoSpeedMps";
    private final double maxSpeed;
    private final double maxAngularRate;

    public DriveSettings(double maxSpeed, double maxAngularRate) {
        this.maxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;
        init(MAX_SPEED_MPS, maxSpeed);
        init(MAX_ANGULAR_RATE, maxAngularRate);
        init(AUTO_SPEED_MPS, 0.5);
    }

    public double maxSpeedMps() {
        return read(MAX_SPEED_MPS, maxSpeed, maxSpeed);
    }

    public double maxAngularRateRadPerSec() {
        return read(MAX_ANGULAR_RATE, maxAngularRate, maxAngularRate);
    }

    public double autoSpeedMps() {
        return read(AUTO_SPEED_MPS, 0.5, Math.min(2.0, maxSpeed));
    }

    private static void init(String key, double defaultValue) {
        SmartDashboard.setDefaultNumber(key, defaultValue);
        SmartDashboard.setPersistent(key);
    }

    private static double read(String key, double defaultValue, double maximum) {
        double value = SmartDashboard.getNumber(key, defaultValue);
        double bounded = Double.isFinite(value) ? MathUtil.clamp(value, 0, maximum) : defaultValue;
        if (Double.compare(value, bounded) != 0) {
            SmartDashboard.putNumber(key, bounded);
        }
        return bounded;
    }
}
