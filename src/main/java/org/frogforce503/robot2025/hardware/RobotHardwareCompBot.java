package org.frogforce503.robot2025.hardware;

import org.frogforce503.robot2025.hardware.subsystem_hardware.ArmHardware;

public class RobotHardwareCompBot extends RobotHardware {
    public RobotHardwareCompBot() {
        armHardware = new ArmHardware(
            10,
            100.0 / 12.0,

            false, 10.0,
            0.1, 0.0, 0.0,
            0.2, 0.0, 0.0, 0.1,
            
            new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
                90.0,
                180.0
            )
        );
    }
}
