// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

public class DriveSubsystem extends SubsystemBase {
  private CommandXboxController controller;
  
  private PIDController pidController = new PIDController(0, 0, 0);

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
    difDrive.feedWatchdog();
  }

  public InstantCommand probeJoysticks(CommandXboxController controller) {
    return new InstantCommand(() -> {
      double goalRotation = controller.getRightX();
      double goalVelocity = controller.getLeftY();

      RelativeEncoder motor1Encoder = motor1.getEncoder(Type.kHallSensor, 42);
      RelativeEncoder motor2Encoder = motor2.getEncoder(Type.kHallSensor, 42);
      RelativeEncoder motor3Encoder = motor3.getEncoder(Type.kHallSensor, 42);
      RelativeEncoder motor4Encoder = motor4.getEncoder(Type.kHallSensor, 42);

      double motor1Velocity = motor1Encoder.getVelocity();
      double motor2Velocity = motor2Encoder.getVelocity();
      double motor3Velocity = motor3Encoder.getVelocity();
      double motor4Velocity = motor4Encoder.getVelocity();

      difDrive.arcadeDrive(leftY/1.25, leftX/1.25);
    },this);
  }
}
