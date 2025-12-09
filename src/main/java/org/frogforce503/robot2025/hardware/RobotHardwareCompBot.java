package org.frogforce503.robot2025.hardware;

import org.frogforce503.robot2025.hardware.subsystem.ArmHardware;

public class RobotHardwareCompBot extends RobotHardware {
    public RobotHardwareCompBot() {
        armHardware = new ArmHardware(
            10,
            150,

            false, 0.0,
            2.5, 0.0, 0.0,
            0.2, 1.0, 0.0, 0.0,
            
            new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
                90.0,
                180.0
            )
        );
    }
}
