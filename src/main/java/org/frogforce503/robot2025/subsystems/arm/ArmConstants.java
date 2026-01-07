package org.frogforce503.robot2025.subsystems.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class ArmConstants {
    public static int CURRENT_LIMIT = 35; // Amps

    public static double POSITION_TOLERANCE = Units.degreesToRadians(0.5); // Radians
    public static double VELOCITY_TOLERANCE = Units.degreesToRadians(2); // Radians/second

    // Arm position is 0 degrees when parallel with the ground
    public static double START_POSITION = Units.degreesToRadians(-90); // Radians
    public static double MAX_POSITION = Units.degreesToRadians(135); // Radians
    public static double MIN_POSITION = Units.degreesToRadians(-90); // Radians

    public static Constraints FAST_PROFILE_CONSTRAINTS = new Constraints(
        Units.degreesToRadians(480), // Velocity Radians/second
        Units.degreesToRadians(960) // Acceleration Radians/second^2
    );
}
