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
    private LoggedNetworkBoolean brakeModeOverride = new LoggedNetworkBoolean("Arm/Brake Mode Override", true);
    private LoggedNetworkBoolean tuningModeOverride = new LoggedNetworkBoolean("Arm/Tuning Mode Override", false);

    private LoggedTunableNumber kP = new LoggedTunableNumber("Arm/Tuning/kP", armHardware.kP());
    private LoggedTunableNumber kI = new LoggedTunableNumber("Arm/Tuning/kI", armHardware.kI());
    private LoggedTunableNumber kD = new LoggedTunableNumber("Arm/Tuning/kD", armHardware.kD());

    private LoggedTunableNumber kS = new LoggedTunableNumber("Arm/Tuning/kS", armHardware.kS());
    private LoggedTunableNumber kG = new LoggedTunableNumber("Arm/Tuning/kG", armHardware.kG());
    private LoggedTunableNumber kV = new LoggedTunableNumber("Arm/Tuning/kV", armHardware.kV());
    private LoggedTunableNumber kA = new LoggedTunableNumber("Arm/Tuning/kA", armHardware.kA());

    private LoggedTunableNumber maxVelocity = new LoggedTunableNumber(
        "Arm/Tuning/Max Velocity",
        ArmConstants.FAST_PROFILE_CONSTRAINTS.maxVelocity
    );

    private LoggedTunableNumber maxAcceleration = new LoggedTunableNumber(
        "Arm/Tuning/Max Acceleration",
        ArmConstants.FAST_PROFILE_CONSTRAINTS.maxAcceleration
    );

    private LoggedTunableNumber positionSetpoint = new LoggedTunableNumber(
        "Arm/Tuning/Position Setpoint", 
        ArmConstants.START_POSITION
    );

    private LoggedTunableNumber velocitySetpoint = new LoggedTunableNumber(
        "Arm/Tuning/Velocity Setpoint", 
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
            double previousVelocity = setpointState.velocity;

            setpointState = profile.calculate(LoggedRobot.defaultPeriodSecs, setpointState, goalState);

            double acceleration = (setpointState.velocity - previousVelocity) / LoggedRobot.defaultPeriodSecs;
            double feedforwardOutput = feedforward.calculate(setpointState.position, setpointState.velocity, acceleration);
            
            armIO.runPosition(setpointState.position, feedforwardOutput);
        }

        // Dashboard Overrides
        if (RobotState.isDisabled() && !brakeModeOverride.get()) {
            if (inBrakeMode) {
                armIO.setBrakeMode(false);
                inBrakeMode = false;
            }
        } else {
            if (!inBrakeMode) {
                armIO.setBrakeMode(true);
                inBrakeMode = true;
            }
        }

        setTuningMode(tuningModeOverride.get());

        // Tuning
        if (kP.hasChanged(kP.hashCode()) || kI.hasChanged(kI.hashCode()) || kD.hasChanged(kD.hashCode())) {
            armIO.setPID(kP.get(), kI.get(), kD.get());
        }

        if (kS.hasChanged(kS.hashCode()) || kG.hasChanged(kG.hashCode())
            || kV.hasChanged(kV.hashCode()) || kA.hasChanged(kA.hashCode())) {
            feedforward = new ArmFeedforward(
                kS.get(),
                kG.get(),
                kV.get(),
                kA.get()
            );
        }

        if (maxVelocity.hasChanged(maxVelocity.hashCode())
            || maxAcceleration.hasChanged(maxAcceleration.hashCode())) {
            profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    maxVelocity.get(),
                    maxAcceleration.get()
                )
            );
        }

        if (positionSetpoint.hasChanged(positionSetpoint.hashCode()) || velocitySetpoint.hasChanged(velocitySetpoint.hashCode())) {
            setArmGoal(positionSetpoint.get(), velocitySetpoint.get());
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
    public double getPosition() {
        return inputs.data.positionRads();
    }

    public double getVelocity() {
        return inputs.data.velocityRadsPerSecond();
    }

    public void setArmGoal(double positionRads) {
        runClosedLoop = true;

        goalState.position = positionRads;
        goalState.velocity = 0;
    }

    public void setArmGoal(double positionRads, double velocityRadsPerSecond) {
        runClosedLoop = true;

        goalState.position = positionRads;
        goalState.velocity = velocityRadsPerSecond;
    }

    public void runPercentOutput(double percentOutput) {
        runClosedLoop = false;
        armIO.runPercentOutput(percentOutput);
    }

    public void runVoltage(double voltage) {
        runClosedLoop = false;
        armIO.runVoltage(voltage);
    }

    public void stop() {
        runClosedLoop = false;
        armIO.stop();
    }

    public boolean atPosition(double targetPositionRads, double toleranceRads) {
        return Math.abs(getPosition() - targetPositionRads) <= toleranceRads;
    }

    public boolean atVelocity(double targetVelocityRadsPerSec, double toleranceRadsPerSec) {
        return Math.abs(getVelocity() - targetVelocityRadsPerSec) <= toleranceRadsPerSec;
    }

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
