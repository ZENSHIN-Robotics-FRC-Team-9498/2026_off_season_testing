package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Field;
import java.time.Duration;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.*;

class ShooterControlTest {
    static RobotContainer robot;
    static XboxControllerSim joystick;
    static CommandScheduler scheduler;
    static final String[] MOTORS = {
        "shooterMotor1", "shooterMotor2", "shooterMoter5", "shooterMotor3", "shooterMotor4"
    };

    @BeforeAll static void init() {
        assertTrue(HAL.initialize(500, 0));
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        joystick = new XboxControllerSim(0);
        joystick.setAxisCount(6);
        joystick.setButtonCount(10);
        joystick.setPOVCount(1);
        joystick.setPOV(-1);
        scheduler = CommandScheduler.getInstance();
        robot = new RobotContainer();
    }

    @BeforeEach void reset() {
        scheduler.cancelAll();
        joystick.setRightBumperButton(false);
        mode(false, false, false);
        tick();
        SmartDashboard.putNumber(ShooterSettings.FLYWHEEL_OUTPUT, 1);
        SmartDashboard.putNumber(ShooterSettings.FEED_OUTPUT, 0.5);
        mode(true, false, false);
        tick();
    }

    static void mode(boolean enabled, boolean auto, boolean test) {
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.setAutonomous(auto);
        DriverStationSim.setTest(test);
        DriverStationSim.notifyNewData();
    }

    static void tick() {
        DriverStationSim.notifyNewData();
        scheduler.run();
    }

    static Object motor(String name) throws Exception {
        Field f = robot.shooter.getClass().getDeclaredField(name);
        f.setAccessible(true);
        return f.get(robot.shooter);
    }

    static double[] demands() throws Exception {
        double[] values = new double[MOTORS.length];
        for (int i = 0; i < values.length; i++) {
            Object m = motor(MOTORS[i]);
            values[i] = m instanceof TalonFX
                ? ((DutyCycleOut) ((TalonFX) m).getAppliedControl()).Output
                : ((SparkMax) m).get();
        }
        return values;
    }

    static void press() {
        joystick.setRightBumperButton(true);
        tick();
        tick();
    }

    static void finishSpinup() {
        SimHooks.stepTimingAsync(1.1);
        tick();
        tick();
    }

    @Test void spinupDoesNotBlockSchedulerAndFeedsOnlyAfterDelay() throws Exception {
        assertTimeout(Duration.ofMillis(300), ShooterControlTest::press);
        assertArrayEquals(new double[] {0, 0, 0, 1, -1}, demands(), 1e-9);
        finishSpinup();
        assertArrayEquals(new double[] {-0.5, 0.5, 0.5, 1, -1}, demands(), 1e-9);
        assertNotNull(robot.shooter.getCurrentCommand());
    }

    @Test void releasingDuringSpinupStopsAllMotors() throws Exception {
        press();
        joystick.setRightBumperButton(false);
        tick();
        finishSpinup();
        assertArrayEquals(new double[5], demands(), 1e-9);
        assertNull(robot.shooter.getCurrentCommand());
    }

    @Test void releasingAfterSpinupStopsAllMotors() throws Exception {
        press();
        finishSpinup();
        joystick.setRightBumperButton(false);
        tick();
        assertArrayEquals(new double[5], demands(), 1e-9);
    }

    @Test void disabledReleaseCannotLeaveLatchedDemand() throws Exception {
        press();
        finishSpinup();
        mode(false, false, false);
        tick();
        assertArrayEquals(new double[5], demands(), 1e-9);
        joystick.setRightBumperButton(false);
        tick();
        mode(true, false, false);
        tick();
        assertArrayEquals(new double[5], demands(), 1e-9);
        assertNull(robot.shooter.getCurrentCommand());
    }

    @Test void cancelAllStopsAllMotors() throws Exception {
        press();
        finishSpinup();
        scheduler.cancelAll();
        assertArrayEquals(new double[5], demands(), 1e-9);
    }

    @Test void dashboardChangesApplyToNextShotWithExistingDirections() throws Exception {
        SmartDashboard.putNumber(ShooterSettings.FLYWHEEL_OUTPUT, 0.7);
        SmartDashboard.putNumber(ShooterSettings.FEED_OUTPUT, 0.3);
        press();
        SmartDashboard.putNumber(ShooterSettings.FLYWHEEL_OUTPUT, 0.9);
        SmartDashboard.putNumber(ShooterSettings.FEED_OUTPUT, 0.4);
        finishSpinup();
        assertArrayEquals(new double[] {-0.3, 0.3, 0.3, 0.7, -0.7}, demands(), 1e-9);
        joystick.setRightBumperButton(false);
        tick();
        press();
        finishSpinup();
        assertArrayEquals(new double[] {-0.4, 0.4, 0.4, 0.9, -0.9}, demands(), 1e-9);
    }

    @Test void shooterCannotStartInAutoOrTest() throws Exception {
        for (boolean auto : new boolean[] {true, false}) {
            mode(true, auto, !auto);
            press();
            assertArrayEquals(new double[5], demands(), 1e-9);
            assertNull(robot.shooter.getCurrentCommand());
            joystick.setRightBumperButton(false);
            tick();
        }
    }

    @Test void settingsAreBoundedFinitePersistentAndNotOverwritten() {
        ShooterSettings settings = new ShooterSettings();
        SmartDashboard.putNumber(ShooterSettings.FLYWHEEL_OUTPUT, 1.5);
        SmartDashboard.putNumber(ShooterSettings.FEED_OUTPUT, -0.5);
        assertEquals(1, settings.flywheelOutput());
        assertEquals(0, settings.feedOutput());
        SmartDashboard.putNumber(ShooterSettings.FLYWHEEL_OUTPUT, Double.NaN);
        SmartDashboard.putNumber(ShooterSettings.FEED_OUTPUT, Double.POSITIVE_INFINITY);
        assertEquals(1, settings.flywheelOutput());
        assertEquals(0.5, settings.feedOutput());
        SmartDashboard.putNumber(ShooterSettings.FLYWHEEL_OUTPUT, 0.6);
        assertEquals(0.6, new ShooterSettings().flywheelOutput());
        assertTrue(SmartDashboard.isPersistent(ShooterSettings.FLYWHEEL_OUTPUT));
        assertTrue(SmartDashboard.isPersistent(ShooterSettings.FEED_OUTPUT));
    }

    @AfterAll static void close() throws Exception {
        scheduler.cancelAll();
        scheduler.getDefaultButtonLoop().clear();
        scheduler.unregisterAllSubsystems();
        Field f = robot.drivetrain.getClass().getDeclaredField("m_simNotifier");
        f.setAccessible(true);
        ((Notifier) f.get(robot.drivetrain)).close();
        robot.drivetrain.close();
        for (String name : MOTORS) ((AutoCloseable) motor(name)).close();
    }
}
