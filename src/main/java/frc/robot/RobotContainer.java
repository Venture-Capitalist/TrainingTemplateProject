// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  private final CommandXboxController m_driverController =
      new CommandXboxController(0);
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via  named factories in the {@link
   * CommandXboxController Xbox} class.
   */
  private void configureBindings() {
    //YOUR BINDINGS HERE

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_drivetrain.exampleMethodCommand());
    m_driverController.a().onTrue(m_DriveSubsystem.moveDaBot());
    m_driverController.b().onTrue(m_DriveSubsystem.moveDaBot());
    m_driverController.x().onTrue(m_DriveSubsystem.moveDaBot());
    m_driverController.y().onTrue(m_DriveSubsystem.moveDaBot());
  }
}
