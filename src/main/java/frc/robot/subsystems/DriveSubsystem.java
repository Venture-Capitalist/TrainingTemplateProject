// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

public class DriveSubsystem extends SubsystemBase {
  private final double kS = 0.4; 

  private CommandXboxController controller;
  
  private PIDController pidController = new PIDController(0.1, 0, 0, 0.02);
  private double ksValueL;
  private double ksValueR;
  private boolean administerVoltageL = true;
  private boolean administerVoltageR = true;

  private CANSparkMax motor1 = new CANSparkMax(11, MotorType.kBrushless);
  private CANSparkMax motor2 = new CANSparkMax(12, MotorType.kBrushless);
  private CANSparkMax motor3 = new CANSparkMax(13, MotorType.kBrushless);
  private CANSparkMax motor4 = new CANSparkMax(14, MotorType.kBrushless);

  private RelativeEncoder leftEncoder = motor1.getEncoder(Type.kHallSensor, 42);
  private RelativeEncoder rightEncoder = motor3.getEncoder(Type.kHallSensor, 42);

  private MotorControllerGroup leftMotorGroup = new MotorControllerGroup(motor1, motor2);
  private MotorControllerGroup rightMotorGroup = new MotorControllerGroup(motor3, motor4);

  private double leftSpeed = 0;
  private double rightSpeed = 0;

  //private DifferentialDrive difDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

  public DriveSubsystem(CommandXboxController controller) {
    this.controller = controller;
    leftMotorGroup.setInverted(true);

    motor1.setIdleMode(IdleMode.kCoast);
    motor2.setIdleMode(IdleMode.kCoast);
    motor3.setIdleMode(IdleMode.kCoast);
    motor4.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    double leftVelocity = leftEncoder.getVelocity();
    double rightVelocity = rightEncoder.getVelocity();
    
    SmartDashboard.putNumber("leftVelocity", leftVelocity);
    SmartDashboard.putNumber("rightVelocity", rightVelocity);
    SmartDashboard.putNumber("Left Voltage", ksValueL);
    SmartDashboard.putNumber("Right Voltage", ksValueR);

    if (administerVoltageL) {
      motor1.setVoltage(leftSpeed);
      motor2.setVoltage(leftSpeed);
    }
    else {
      motor1.setVoltage(0);
      motor2.setVoltage(0);
    }
    if (administerVoltageR) {
      motor3.setVoltage(rightSpeed);
      motor4.setVoltage(rightSpeed);
    }
    else {
      motor3.setVoltage(0);
      motor4.setVoltage(0);
    }

    //difDrive.feedWatchdog();
  }

  public InstantCommand probeJoysticks(CommandXboxController controller) {
    return new InstantCommand(() -> {
      double goalRotation = controller.getRightX();
      double goalVelocity = controller.getLeftY();

      WheelSpeeds spedz =  DifferentialDrive.arcadeDriveIK(goalRotation, goalVelocity, false);
      
      if (!(spedz.left >= -0.1 && spedz.left <= 0.1) ) {
        if (goalVelocity < 0) {
          leftSpeed = spedz.left + kS;
        } 
        else {
          leftSpeed = spedz.left - kS;
        }
      }
      else {
        leftSpeed = 0;
      }
      if (!(spedz.right >= -0.1 && spedz.right <= 0.1)) {
        if (goalVelocity < 0) {
          rightSpeed = spedz.right - kS;
        } 
        else {
          rightSpeed = spedz.right + kS;
        }
      }
      else {
        rightSpeed = 0;
      }
    }, this);
  }

  public InstantCommand increaseVoltageL() {
    return new InstantCommand(() -> {
      ksValueL += 0.1;
    }, this);
  }

  public InstantCommand increaseVoltageR() {
    return new InstantCommand(() -> {
      System.out.println("WEEEEEEE");
      ksValueR += 0.1;
    }, this);
  }
  
  public InstantCommand toggleLeftVoltage() {
    return new InstantCommand(() -> {
      administerVoltageL = !administerVoltageL;
    });
  }

  public InstantCommand toggleRightVoltage() {
    return new InstantCommand(() -> {
      administerVoltageR = !administerVoltageR;
    });
  }
}