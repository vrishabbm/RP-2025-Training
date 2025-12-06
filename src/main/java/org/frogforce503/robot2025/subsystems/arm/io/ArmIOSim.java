package org.frogforce503.robot2025.subsystems.arm.io;

import org.frogforce503.robot2025.hardware.subsystem_hardware.ArmHardware;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.subsystems.arm.ArmConstants;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim extends ArmIOSpark{
    private SparkMaxSim motorSim;
    private SingleJointedArmSim armSim;
    private ArmHardware armHardware = Robot.bot.armHardware;

    private final DCMotor motorModel = DCMotor.getNEO(1);
    private final double momentOfInertia = 0.95; // kg * m^2
    private final double length = Units.inchesToMeters(19.0); 

    public ArmIOSim() {
        super();
        motorSim = new SparkMaxSim(getMotor(), motorModel);

        armSim = new SingleJointedArmSim(
            motorModel,
            armHardware.mechanismRatio(),
            momentOfInertia,
            length,
            ArmConstants.MIN_POSITION,
            ArmConstants.MAX_POSITION,
            true,
            ArmConstants.START_POSITION
        );
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        double appliedVolts = motorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        
        // Apply physics
        armSim.setInputVoltage(appliedVolts);
        armSim.update(0.02);

        // Update motor simulation
        motorSim.iterate(armSim.getVelocityRadPerSec(), RobotController.getBatteryVoltage(), 0.02);

        inputs.data =
            new ArmIOData(
                true,
                armSim.getAngleRads(),
                armSim.getVelocityRadPerSec(),
                appliedVolts,
                armSim.getCurrentDrawAmps(),
                24.0);
    }
}
