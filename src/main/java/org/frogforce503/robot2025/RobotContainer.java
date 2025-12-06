package org.frogforce503.robot2025;

import org.frogforce503.robot2025.auto.AutoChooser;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer{
  // Controllers
  public final CommandXboxController driver = new CommandXboxController(0);
  public final CommandXboxController operator = new CommandXboxController(1);

  // Needs
  public final AutoChooser autoChooser = new AutoChooser();
  
  // Other Hardware
  // public final PowerDistribution powerDistribution = new PowerDistribution();

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {

  }
}