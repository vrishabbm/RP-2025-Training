package org.frogforce503.robot2025;

import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Visualizer {
    LoggedMechanismLigament2d arm;

    public Visualizer () {
        arm = new LoggedMechanismLigament2d("Visualizer/Arm", Units.inchesToMeters(19), 0);
    }
    
    public void update(double armAngleRadians) {
        arm.setAngle(Rotation2d.fromRadians(armAngleRadians));
    }
}
