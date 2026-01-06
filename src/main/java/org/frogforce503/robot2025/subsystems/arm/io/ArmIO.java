package org.frogforce503.robot2025.subsystems.arm.io;

import org.littletonrobotics.junction.AutoLog;

/**
 * ArmIO is an interface that defines the methods for using the physical electronics on the arm.
 */
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

    /**
     * ArmIOData is a record that holds the inputs from the arm subsystem.
     * Inputs are data given to us by the hardware on the arm that we need in our code.
     * 
     * @param connected whether the arm motor is connected with no faults/errors
     * @param positionRads the position of the arm in radians
     * @param velocityRadsPerSecond the velocity of the arm in radians per second
     * @param appliedVolts the voltage applied to the arm motor
     * @param currentAmps the current draw of the arm motor in amps
     * @param tempCelsius the temperature of the arm motor in celsius
     * 
     */
    record ArmIOData(
        boolean connected,
        double positionRads,
        double velocityRadsPerSecond,
        double appliedVolts,
        double currentAmps,
        double tempCelsius
    ) {}

    public void updateInputs(ArmIOInputs inputs);

    public void reset();

    public void runPercentOutput(double percentOutput);

    public void runVoltage(double voltage);

    public void runPosition(double positionRads, double feedforward);

    public void stop();

    public void setPID(double kP, double kI, double kD);

    public void setPIDSlot(int slot);

    public void setBrakeMode(boolean brake);
}
