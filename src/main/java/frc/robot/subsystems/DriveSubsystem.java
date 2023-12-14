// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveSubsystem extends SubsystemBase {
  private CommandXboxController controller;

  private CANSparkMax motor1 = new CANSparkMax(11, MotorType.kBrushless);
  private CANSparkMax motor2 = new CANSparkMax(12, MotorType.kBrushless);
  private CANSparkMax motor3 = new CANSparkMax(13, MotorType.kBrushless);
  private CANSparkMax motor4 = new CANSparkMax(14, MotorType.kBrushless);

  private MotorControllerGroup leftMotorGroup = new MotorControllerGroup(motor1, motor2);
  private MotorControllerGroup rightMotorGroup = new MotorControllerGroup(motor3, motor4);

  private DifferentialDrive difDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

  public DriveSubsystem(CommandXboxController controller) {
    this.controller = controller;
  }

  @Override
  public void periodic() {
    leftMotorGroup.setInverted(true);
  }

  public InstantCommand probeJoysticks(double leftX, double leftY) {
    return new InstantCommand(() -> {
      difDrive.arcadeDrive(leftX, leftY);
    },this);
  }
}
