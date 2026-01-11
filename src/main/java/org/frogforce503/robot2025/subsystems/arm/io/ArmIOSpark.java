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

import edu.wpi.first.math.MathUtil;
import lombok.Getter;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.hardware.subsystemhardware.ArmHardware;
import org.frogforce503.robot2025.subsystems.arm.ArmConstants;

public class ArmIOSpark implements ArmIO {
    private ArmHardware armHardware;

    @Getter private SparkMax motor;
    private AbsoluteEncoder encoder;
    private SparkMaxConfig config;

    private SparkClosedLoopController controller;
    private ClosedLoopSlot closedLoopSlot = ClosedLoopSlot.kSlot0;

    public ArmIOSpark() {
        armHardware = Robot.bot.getArmHardware();

        motor = new SparkMax(armHardware.id(), MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder();
        
        config = new SparkMaxConfig();

        controller = motor.getClosedLoopController();

        // Configuration
        config.inverted(armHardware.inverted());
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(ArmConstants.CURRENT_LIMIT);
        config.voltageCompensation(12);

        config.absoluteEncoder
            .zeroOffset(armHardware.zeroOffset())
            .positionConversionFactor(2 * Math.PI) // rotations -> radians
            .velocityConversionFactor(2 * Math.PI / 60); // RPM -> radians per second

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(armHardware.kP())
            .i(armHardware.kI())
            .d(armHardware.kD());
        
        config.signals
            .absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderVelocityAlwaysOn(true)
            .analogPositionAlwaysOn(false) 
            .analogVelocityAlwaysOn(false) 
            .analogVoltageAlwaysOn(false) 
            .externalOrAltEncoderPositionAlwaysOn(true)
            .externalOrAltEncoderVelocityAlwaysOn(true)
            .faultsAlwaysOn(true)
            .faultsPeriodMs(1000) // Updates once a second, faults don't need high update rate
            .iAccumulationAlwaysOn(true)
            .limitsPeriodMs(250) // Updates 4 times a second, PLEASE uncomment if there are limit switches attached to motor controller
            .motorTemperaturePeriodMs(1000) // Updates once a second, temperature doesn't need high update rate
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderVelocityAlwaysOn(true)
            .warningsAlwaysOn(true)
            .warningsPeriodMs(1000); // Updates once a second, warnings don't need high update rate

        motor.clearFaults();
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public AbsoluteEncoder getEncoder() {
        return encoder;
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
    public void reset() {}

    @Override
    public void runPercentOutput(double percentOutput) {
        motor.set(percentOutput);
    }

    @Override
    public void runVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void runPosition(double positionRads, double feedforward) {
        controller.setReference(positionRads, ControlType.kPosition, closedLoopSlot, feedforward);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        config.closedLoop.p(kP, closedLoopSlot);
        config.closedLoop.i(kI, closedLoopSlot);
        config.closedLoop.d(kD, closedLoopSlot);

        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setPIDSlot(int slot) {
        closedLoopSlot = switch (MathUtil.clamp(slot, 0, 3)) {
            case 0 -> ClosedLoopSlot.kSlot0;
            case 1 -> ClosedLoopSlot.kSlot1;
            case 2 -> ClosedLoopSlot.kSlot2;
            case 3 -> ClosedLoopSlot.kSlot3;
            default -> ClosedLoopSlot.kSlot0;
        };
    }

    @Override
    public void setBrakeMode(boolean brake) {
        config.idleMode(brake ? SparkMaxConfig.IdleMode.kBrake : SparkMaxConfig.IdleMode.kCoast);

        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
