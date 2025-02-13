package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.KDoublePreferences.PElevator;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Elevator subsystem responsible for controlling the lifting mechanism.
 * Uses PID control for precise movement and prevents unsafe operation
 * via limit switches and software constraints.
 */
public class Elevator extends SubsystemBase {

    @AutoLogOutput private double currentHeight = 0.0;
    @AutoLogOutput private double targetHeight = 0.0;
    @AutoLogOutput private boolean isOnTarget = false;

    private final ElevatorModuleIO moduleIO;
    private final DigitalInput homeSwitch;
    private final ProfiledPIDController pid;
    private boolean manualOverride = false;
    private int loopCount = 0; // Counter to reduce SmartDashboard updates

    /**
     * Constructor for the Elevator subsystem.
     *
     * @param moduleIO Hardware interface for elevator motors.
     * @param homeSwitch Digital input limit switch for homing.
     */
    public Elevator(ElevatorModuleIO moduleIO, DigitalInput homeSwitch) {
        this.moduleIO = moduleIO;
        this.homeSwitch = homeSwitch;

        pid = new ProfiledPIDController(
            PElevator.proportional.getValue(),
            PElevator.integral.getValue(),
            PElevator.derivative.getValue(),
            new TrapezoidProfile.Constraints(
                PElevator.speedlimit.getValue(), 
                PElevator.accelerationLimit.getValue()
            )
        );

        targetHeight = moduleIO.getHeightMeters();
        pid.setGoal(targetHeight);
    }

    @Override
    public void periodic() {
        moduleIO.updateInputs();
        currentHeight = moduleIO.getHeightMeters();

        // Reset encoder if home switch is pressed
        if (!homeSwitch.get()) {
            moduleIO.resetEncoder();
            currentHeight = 0.0;
            targetHeight = 0.0;
            pid.reset(0.0);
        }

        if (manualOverride) return;

        // Clamp target height to prevent exceeding limits
        double maxHeight = PElevator.MaxHeight.getValue();
        targetHeight = Math.max(0.0, Math.min(targetHeight, maxHeight));
        pid.setGoal(targetHeight);

        // Compute PID output and prevent downward motion at home
        double pidOutput = pid.calculate(currentHeight, targetHeight);
        if (pidOutput < 0 && !homeSwitch.get()) {
            pidOutput = 0;
        }

        if (Math.abs(currentHeight - targetHeight) < PElevator.tolerance.getValue()) {
            isOnTarget = true;
            moduleIO.stop();
        } else {
            isOnTarget = false;
            moduleIO.setVoltage(pidOutput);
        }

        // Increment loop counter and update SmartDashboard every 5 cycles (~100ms)
        loopCount++;
        if (loopCount % 5 == 0) {
            SmartDashboard.putNumber("Elevator Height", currentHeight);
            SmartDashboard.putNumber("Target Height", targetHeight);
            SmartDashboard.putBoolean("At Target", isOnTarget);
            SmartDashboard.putBoolean("Manual Override", manualOverride);
        }
    }

    /** Sets a new target height for the elevator using PID control. */
    public void setTargetHeight(double height) {
        targetHeight = Math.max(0.0, Math.min(height, PElevator.MaxHeight.getValue()));
        pid.setGoal(targetHeight);
        manualOverride = false;
    }

    /** Allows manual control of the elevator, bypassing PID. */
    public void setManualSpeed(double speed) {
        manualOverride = true;
        if (speed < 0 && !homeSwitch.get()) {
            speed = 0;
        }
        moduleIO.setSpeed(speed);
    }

    /** Holds the current position using PID control. */
    public void holdPositionPID() {
        manualOverride = false;
        if (Math.abs(targetHeight - currentHeight) > PElevator.tolerance.getValue()) {
            targetHeight = currentHeight;
            pid.setGoal(currentHeight);
        }
    }

    /** Holds the current position using braking mode. */
    public void holdPositionBrake() {
        manualOverride = true;
        moduleIO.stop();
    }

    /** Emergency stop function that immediately disables motor output. */
    public void emergencyStop() {
        moduleIO.stop();
        manualOverride = true;
    }

    public double getCurrentHeight() {
        return currentHeight;
    }

    public boolean isOnTarget() {
        return isOnTarget;
    }
}
