package org.frogforce503.robot2025.subsystems.arm;

import org.frogforce503.robot2025.subsystems.arm.io.ArmIO;
import org.littletonrobotics.junction.Logger;
import org.frogforce503.robot2025.subsystems.arm.io.ArmIOInputsAutoLogged;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    ArmIO armIO;
    ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public Arm(ArmIO armIO) {
        this.armIO = armIO;
    }

    @Override
    public void periodic() {
        armIO.updateInputs(inputs);
        Logger.processInputs("Arm/Inputs", inputs);
    }
}
