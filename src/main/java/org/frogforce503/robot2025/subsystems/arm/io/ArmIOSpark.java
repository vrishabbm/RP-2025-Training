package org.frogforce503.robot2025.subsystems.arm.io;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;

import org.frogforce503.robot2025.Robot;

public class ArmIOSpark implements ArmIO {
    SparkMax motor;
    AbsoluteEncoder encoder;

    SparkMaxConfig config;
    SparkMaxConfigAccessor configAccessor;

    SparkClosedLoopController controller;

    public ArmIOSpark() {
        motor = new SparkMax(Robot.bot.armConstants.id(), MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder();
        
        configAccessor = motor.configAccessor;
        config = new SparkMaxConfig();

        controller = motor.getClosedLoopController();
    }

    /****************************************** ARM IO IMPLEMENTED METHODS ******************************************/
    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.data = new ArmIOData(
            motor.getLastError() == REVLibError.kOk,
            encoder.getPosition(),
            encoder.getVelocity(),
            motor.getAppliedOutput() * motor.getBusVoltage(),
            motor.getOutputCurrent(),
            motor.getMotorTemperature()
        );
    }

    @Override
    public void reset() {
        config.absoluteEncoder.zeroOffset(encoder.getPosition());

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void runPercentOutput(double percentOutput) {
        motor.set(percentOutput);
    }

    @Override
    public void runVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void runPosition(double positionDegrees, double feedforward) {
        controller.setReference(positionDegrees, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward);
    }

    @Override
    public void runVelocity(double velocityDegreesPerSecond, double feedforward) {
        controller.setReference(velocityDegreesPerSecond, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        config.closedLoop.p(kP);
        config.closedLoop.i(kI);
        config.closedLoop.d(kD);

        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        config.idleMode(brake ? SparkMaxConfig.IdleMode.kBrake : SparkMaxConfig.IdleMode.kCoast);

        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    
}
