package org.frogforce503.robot2025;

import org.frogforce503.robot2025.auto.AutoChooser;
import org.frogforce503.robot2025.subsystems.arm.Arm;
import org.frogforce503.robot2025.subsystems.arm.io.ArmIOSim;
import org.frogforce503.robot2025.subsystems.arm.io.ArmIOSpark;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.util.Units;

public class RobotContainer{
  // Controllers
  public final CommandXboxController driver = new CommandXboxController(0);
  public final CommandXboxController operator = new CommandXboxController(1);

  // Subsystems
  public final Arm arm;

  // Miscellaneous
  public final AutoChooser autoChooser = new AutoChooser();
  public final Visualizer visualizer = new Visualizer();

  public RobotContainer() {
    switch (RobotStatus.getInstance().getCurrentRobot()) {
      case CompBot:
        arm = new Arm(
          new ArmIOSpark()
        );

        break;
      
      case SimBot:
        arm = new Arm(
          new ArmIOSim()
        );

        break;

      default:
        arm = new Arm(
          new ArmIOSpark()
        );
    }

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    Trigger leftStickPushedUp = new Trigger(() -> (driver.getLeftY() > 0.2));
    Trigger leftStickPushedDown = new Trigger(() -> (driver.getLeftY() < -0.2));

    leftStickPushedUp.onTrue(
      Commands.runOnce(() -> arm.setArmGoal(Units.degreesToRadians(50)), arm)
    );

    leftStickPushedDown.onTrue(
      Commands.runOnce(() -> arm.setArmGoal(Units.degreesToRadians(-50)), arm)
    );
  }

  public void periodic() {
    visualizer.update(arm.getPosition());
  }
}