package org.frogforce503.robot2025.subsystems.arm;

import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.hardware.subsystem_hardware.ArmHardware;
import org.frogforce503.robot2025.subsystems.arm.io.ArmIO;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.frogforce503.robot2025.subsystems.arm.io.ArmIOInputsAutoLogged;

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
    private boolean inBrakeMode = false;
    private boolean atGoal = false;

    // Logging
    private LoggedNetworkBoolean brakeModeOverride = new LoggedNetworkBoolean("Arm/Brake Mode Override", false);

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

            setpointState = profile.calculate(0.02, setpointState, goalState);

            double acceleration = (setpointState.velocity - previousVelocity) / 0.02;
            double feedforwardOutput = feedforward.calculate(setpointState.position, setpointState.velocity, acceleration);
            
            armIO.runPosition(setpointState.position, feedforwardOutput);
        }

        // Brake Mode
        if (RobotState.isDisabled() && brakeModeOverride.get()) {
            if (!inBrakeMode) {
                armIO.setBrakeMode(true);
                inBrakeMode = true;
            }
        } else {
            if (!inBrakeMode) {
                armIO.setBrakeMode(true);
                inBrakeMode = true;
            }
        }

        // Logging
        Logger.recordOutput("Arm/Running Closed Loop", runClosedLoop);
        Logger.recordOutput("Arm/Closed Loop/Setpoint Position", setpointState.position);
        Logger.recordOutput("Arm/Closed Loop/Setpoint Velocity", setpointState.velocity);
        Logger.recordOutput("Arm/Closed Loop/Goal Position", goalState.position);
        Logger.recordOutput("Arm/Closed Loop/Goal Velocity", goalState.velocity);

        Logger.recordOutput("Arm/At Goal", atGoal);
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
        return Math.abs(getPosition() - targetVelocityRadsPerSec) <= toleranceRadsPerSec;
    }

    public boolean atGoal() {
        return atGoal;
    }
}
