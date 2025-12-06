package org.frogforce503.robot2025.subsystems.arm;

import edu.wpi.first.math.util.Units;

public class ArmConstants {
    public static int CURRENT_LIMIT = 35; // Amps

    public static double TOLERANCE = Units.degreesToRadians(0.5);

    // Arm position is 0 degrees when parallel with the ground
    public static double START_POSITION = Units.degreesToRadians(-90);
    public static double MAX_POSITION = Units.degreesToRadians(135);
    public static double MIN_POSITION = Units.degreesToRadians(-90);


}
