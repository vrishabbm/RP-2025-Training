package org.frogforce503.robot2025.visualizers;

import org.frogforce503.robot2025.subsystems.arm.ArmConstants;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Visualizer2d {
    // Measurements
    private final Translation2d elevatorOrigin = new Translation2d(Units.inchesToMeters(15), Units.inchesToMeters(15)); // Base of Elevator
    private final double elevatorInitialHeight = Units.inchesToMeters(28);

    private final double armLength = Units.inchesToMeters(19);
    private final double armAngleOffset = Units.degreesToRadians(-85);

    // Mechanism/Ligaments
    private final LoggedMechanism2d superstructureMechanism = new LoggedMechanism2d(
        Units.inchesToMeters(50),
        Units.inchesToMeters(80));

    private final LoggedMechanismLigament2d elevatorLigament;
    private final LoggedMechanismLigament2d armLigament;

    public Visualizer2d () {
        LoggedMechanismRoot2d mainRoot =
            superstructureMechanism.getRoot(
                "Main Root", elevatorOrigin.getX(), elevatorOrigin.getY());
            
        elevatorLigament = mainRoot.append(
            new LoggedMechanismLigament2d(
                "Elevator", 
                elevatorInitialHeight, 
                armAngleOffset,
                4.0,
                new Color8Bit(Color.kFirstBlue)
            )
        );
                    
        armLigament = elevatorLigament.append(
            new LoggedMechanismLigament2d(
                "Arm",
                armLength,
                Units.radiansToDegrees(ArmConstants.START_POSITION), // Initial relative angle (set in update)
                4.0,
                new Color8Bit(Color.kFirstRed))
        );
    }
    
    public void update(double armAngleRadians) {
        armLigament.setAngle(Rotation2d.fromRadians(armAngleRadians));
    }
}
