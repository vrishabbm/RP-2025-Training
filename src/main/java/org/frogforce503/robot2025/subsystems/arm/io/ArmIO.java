package org.frogforce503.robot2025.subsystems.arm.io;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs {
        public ArmIOData data = new ArmIOData(
            false,
            0, 
            0, 
            0, 
            0, 
            0
        );
    }

    record ArmIOData(
        boolean connected,
        double positionInputs,
        double velocityInputsPerSecond,
        double appliedVolts,
        double currentAmps,
        double tempCelsius
    ) {}

    public void updateInputs(ArmIOInputs inputs);

    public void reset();

    public void runPercentOutput(double percentOutput);

    public void runVoltage(double voltage);

    public void runPosition(double positionRadians, double feedforward);

    public void stop();

    public void setPID(double kP, double kI, double kD);

    public void setPIDSlot(int slot);

    public void setBrakeMode(boolean brake);
}
