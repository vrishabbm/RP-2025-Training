package org.frogforce503.robot2025.hardware.subsystem;

public record ArmHardware (
    int id,
    double mechanismRatio,

    boolean inverted, double zeroOffset,

    double kP, double kI, double kD,
    double kS, double kG, double kV, double kA
) {}
