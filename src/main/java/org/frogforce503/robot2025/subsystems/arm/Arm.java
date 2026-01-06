package org.frogforce503.robot2025.subsystems.arm;

import org.frogforce503.lib.logging.LoggedTunableNumber;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.hardware.subsystem.ArmHardware;
import org.frogforce503.robot2025.subsystems.arm.io.ArmIO;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.frogforce503.robot2025.subsystems.arm.io.ArmIOInputsAutoLogged;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    // Constants
    private ArmHardware armHardware = Robot.bot.armHardware;

    // IO
    private ArmIO armIO;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    
    // Control Scheme: Spark Max PID + WPILib FF + WPILib Trapezoidal Profile
    private ArmFeedforward feedforward = new ArmFeedforward(
        armHardware.kS(),
        armHardware.kG(),
        armHardware.kV(),
        armHardware.kA()
    );

    private TrapezoidProfile profile = new TrapezoidProfile(ArmConstants.FAST_PROFILE_CONSTRAINTS);

    private State setpointState = new State(ArmConstants.START_POSITION, 0);
    private State goalState = new State(ArmConstants.START_POSITION, 0);

    // Logic
    private boolean runClosedLoop = false;
    private boolean inBrakeMode = true;
    private boolean atGoal = false;

    // Logging
    private LoggedNetworkBoolean brakeModeOverride = new LoggedNetworkBoolean("/Tuning/Arm/Enable Brake Mode", true);
    private LoggedNetworkBoolean tuningModeOverride = new LoggedNetworkBoolean("/Tuning/Arm/Enable Tuning Mode", false);

    private LoggedTunableNumber kP = new LoggedTunableNumber("Arm/PID/kP", armHardware.kP());
    private LoggedTunableNumber kI = new LoggedTunableNumber("Arm/PID/kI", armHardware.kI());
    private LoggedTunableNumber kD = new LoggedTunableNumber("Arm/PID/kD", armHardware.kD());

    private LoggedTunableNumber kS = new LoggedTunableNumber("Arm/FF/kS", armHardware.kS());
    private LoggedTunableNumber kG = new LoggedTunableNumber("Arm/FF/kG", armHardware.kG());
    private LoggedTunableNumber kV = new LoggedTunableNumber("Arm/FF/kV", armHardware.kV());
    private LoggedTunableNumber kA = new LoggedTunableNumber("Arm/FF/kA", armHardware.kA());

    private LoggedTunableNumber maxVelocity = new LoggedTunableNumber(
        "Arm/Profile/Max Velocity",
        ArmConstants.FAST_PROFILE_CONSTRAINTS.maxVelocity
    );

    private LoggedTunableNumber maxAcceleration = new LoggedTunableNumber(
        "Arm/Profile/Max Acceleration",
        ArmConstants.FAST_PROFILE_CONSTRAINTS.maxAcceleration
    );

    private LoggedTunableNumber positionSetpoint = new LoggedTunableNumber(
        "Arm/Setpoint/Position Setpoint", 
        ArmConstants.START_POSITION
    );

    private LoggedTunableNumber velocitySetpoint = new LoggedTunableNumber(
        "Arm/Setpoint/Velocity Setpoint", 
        0
    );

    public Arm(ArmIO armIO) {
        this.armIO = armIO;
    }

    @Override
    public void periodic() {
        // IO Inputs
        armIO.updateInputs(inputs);
        Logger.processInputs("Arm/Inputs", inputs);

        // At Goal Check
        atGoal = atPosition(goalState.position, ArmConstants.POSITION_TOLERANCE)
            && atVelocity(goalState.velocity, ArmConstants.VELOCITY_TOLERANCE);

        // Control Loop
        if (runClosedLoop) {
            double previousVelocity = setpointState.velocity; // save last setpoint velocity before it updated by the profile

            setpointState = profile.calculate(LoggedRobot.defaultPeriodSecs, setpointState, goalState); // update setpoint based on profile towards goal

            double acceleration = (setpointState.velocity - previousVelocity) / LoggedRobot.defaultPeriodSecs;
            double feedforwardOutput = feedforward.calculate(setpointState.position, setpointState.velocity, acceleration); // calculate feedforward based on new setpoint
            
            armIO.runPosition(setpointState.position, feedforwardOutput);
        }

        // Dashboard Overrides
        if (RobotState.isDisabled() && !brakeModeOverride.get()) { // Only allow disabling brake mode while robot is disabled
            if (inBrakeMode) {
                armIO.setBrakeMode(false);
                inBrakeMode = false;
            }
        } else { // Enable brake mode during enabled or if override is true
            if (!inBrakeMode) {
                armIO.setBrakeMode(true);
                inBrakeMode = true;
            }
        }

        setTuningMode(tuningModeOverride.get());

        // Dashboard Tuning
        if (kP.hasChanged(kP.hashCode()) || 
            kI.hasChanged(kI.hashCode()) || 
            kD.hasChanged(kD.hashCode())
        ) { // If any PID value changed in the dashboard, update the motor with new gains
            armIO.setPID(
                kP.get(), 
                kI.get(), 
                kD.get()
            );
        }

        if (kS.hasChanged(kS.hashCode()) || 
            kG.hasChanged(kG.hashCode()) || 
            kV.hasChanged(kV.hashCode()) || 
            kA.hasChanged(kA.hashCode())
        ) { // If any FF gain changed in the dashboard, update the feedforward variable with new gains
            feedforward = new ArmFeedforward(
                kS.get(),
                kG.get(),
                kV.get(),
                kA.get()
            );
        }

        if (maxVelocity.hasChanged(maxVelocity.hashCode())|| 
            maxAcceleration.hasChanged(maxAcceleration.hashCode())
        ) { // If profile constraints changed in the dashboard, update the trapezoidal profile with new constraints
            profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    maxVelocity.get(),
                    maxAcceleration.get()
                )
            );
        }

        if (RobotState.isEnabled() && 
            (positionSetpoint.hasChanged(positionSetpoint.hashCode()) || velocitySetpoint.hasChanged(velocitySetpoint.hashCode()))
        ) { // If setpoints changed in the dashboard, update the goal state with new setpoints
            setArmGoal(Units.degreesToRadians(positionSetpoint.get()), Units.degreesToRadians(velocitySetpoint.get()));
        } 

        // Clamp Setpoint to Safe Range
        goalState.position = MathUtil.clamp(goalState.position, ArmConstants.MIN_POSITION, ArmConstants.MAX_POSITION);

        // Logging
        Logger.recordOutput("Arm/Running Closed Loop", runClosedLoop);
        Logger.recordOutput("Arm/Closed Loop/Setpoint Position", setpointState.position);
        Logger.recordOutput("Arm/Closed Loop/Setpoint Velocity", setpointState.velocity);
        Logger.recordOutput("Arm/Closed Loop/Goal Position", goalState.position);
        Logger.recordOutput("Arm/Closed Loop/Goal Velocity", goalState.velocity);

        Logger.recordOutput("Arm/At Goal", atGoal);
        Logger.recordOutput("Arm/In Brake Mode", inBrakeMode);
        Logger.recordOutput("Arm/Current Command", getCurrentCommand() == null ? "None" : getCurrentCommand().getName());
    }

    // Public Methods
    /**
     * @return The current position of the arm in radians. Arm angle is zero when horizontal.
     */
    public double getPosition() {
        return inputs.data.positionRads();
    }

    /**
     * @return The current velocity of the arm in radians per second.
     */
    public double getVelocity() {
        return inputs.data.velocityRadsPerSecond();
    }

    /**
     * Set the Arm goal state with position in radians and velocity as 0 radians per second.
     * 
     * @param positionRads The goal position in radians.
     */
    public void setArmGoal(double positionRads) {
        runClosedLoop = true;

        goalState.position = positionRads;
        goalState.velocity = 0;
    }

    /**
     * Set the Arm goal state with position in radians and velocity in radians per second.
     * 
     * @param positionRads The goal position in radians.
     * @param velocityRadsPerSecond The goal velocity in radians per second.
     */
    public void setArmGoal(double positionRads, double velocityRadsPerSecond) {
        runClosedLoop = true;

        goalState.position = positionRads;
        goalState.velocity = velocityRadsPerSecond;
    }

    /**
     * Run the arm in percent output mode.
     * @param percentOutput The percent output to run the arm at (-1.0 to 1.0).
     */
    public void runPercentOutput(double percentOutput) {
        runClosedLoop = false; // disable closed-loop control when running open-loop control like percent output
        armIO.runPercentOutput(percentOutput);
    }

    /**
     * Run the arm in voltage mode.
     * @param voltage The voltage to run the arm at.
     */
    public void runVoltage(double voltage) {
        runClosedLoop = false; // disable closed-loop control when running open-loop control like voltage
        armIO.runVoltage(voltage);
    }

    /**
     * Stop the arm motor.
     */
    public void stop() {
        runClosedLoop = false;
        armIO.stop();
    }

    /**
     * Check if the arm is at a given position within a tolerance.
     * @param targetPositionRads 
     * @param toleranceRads
     * @return True if the arm is at the target position within the tolerance, false otherwise.
     */
    public boolean atPosition(double targetPositionRads, double toleranceRads) {
        return Math.abs(getPosition() - targetPositionRads) <= toleranceRads;
    }

    /**
     * Check if the arm is at a given velocity within a tolerance.
     * @param targetVelocityRadsPerSec 
     * @param toleranceRadsPerSec
     * @return True if the arm is at the target velocity within the tolerance, false otherwise.
     */
    public boolean atVelocity(double targetVelocityRadsPerSec, double toleranceRadsPerSec) {
        return Math.abs(getVelocity() - targetVelocityRadsPerSec) <= toleranceRadsPerSec;
    }

    /**
     * @return True if the arm is at the goal position and velocity, false otherwise.
     */
    public boolean atGoal() {
        return atGoal;
    }

     // Private Methods
     private void setTuningMode(boolean tuning) {
        kP.setTuningMode(tuning);
        kI.setTuningMode(tuning);
        kD.setTuningMode(tuning);

        kS.setTuningMode(tuning);
        kG.setTuningMode(tuning);
        kV.setTuningMode(tuning);
        kA.setTuningMode(tuning);

        maxVelocity.setTuningMode(tuning);
        maxAcceleration.setTuningMode(tuning);

        positionSetpoint.setTuningMode(tuning);
        velocitySetpoint.setTuningMode(tuning);
     }
}
