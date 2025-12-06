package org.frogforce503.robot2025.subsystems.arm;

import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.hardware.subsystem_hardware.ArmHardware;
import org.frogforce503.robot2025.subsystems.arm.io.ArmIO;
import org.littletonrobotics.junction.Logger;
import org.frogforce503.robot2025.subsystems.arm.io.ArmIOInputsAutoLogged;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
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

    private State currentState = new State(ArmConstants.START_POSITION, 0);
    private State setpointState = new State();
    private State goalState = new State(ArmConstants.START_POSITION, 0);

    // Logic
    private boolean runClosedLoop = false;

    public Arm(ArmIO armIO) {
        this.armIO = armIO;
    }

    @Override
    public void periodic() {
        armIO.updateInputs(inputs);
        Logger.processInputs("Arm/Inputs", inputs);

        currentState.position = getPosition();
        currentState.velocity = getVelocity();

        if (runClosedLoop) {
            setpointState = profile.calculate(0.02, currentState, goalState);
            double feedforwardOutput = feedforward.calculate(getPosition(), getVelocity());

            armIO.runPosition(setpointState.position, feedforwardOutput);
        }
    }

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
}
