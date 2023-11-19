// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;

public class DriveSubsystem extends SubsystemBase {
  private boolean automated = false;
  private String auto_forwards = "down";

  private Encoder m_leftEncoder = new Encoder(0, 1);
  private Encoder m_rightEncoder = new Encoder(2, 3);
  private EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);

  private Field2d m_field = new Field2d();
  
  private AnalogGyro m_gyro = new AnalogGyro(1);
  private AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

  private DifferentialDrivetrainSim m_driveSim = DifferentialDrivetrainSim.createKitbotSim(
    KitbotMotor.kDualCIMPerSide, // 2 CIMs per side.
    KitbotGearing.k10p71,        // 10.71:1
    KitbotWheelSize.kSixInch,    // 6" diameter wheels.
    null                         // No measurement noise.
  );

  private PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  private PWMSparkMax m_rightMotor = new PWMSparkMax(1);

  private DifferentialDrive difDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  private Float speedMod = 0f;

  public String speedVal = "0";
  public String rotationVal = "0";

  private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
    m_gyro.getRotation2d(), 
    m_leftEncoder.getDistance(), 
    m_rightEncoder.getDistance()
  );

  public DriveSubsystem() {
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * 3 / 4096);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * 3 / 4096);

    m_leftMotor.setInverted(true);

    SmartDashboard.putData(m_field);
    SmartDashboard.putNumber("SPEED MODIFIER (PRESS BUTTON 6 TO BEGIN MOVING):", speedMod);
    SmartDashboard.putString("CURRENT STATE:", "TELEOPERATED");
  }

  @Override
  public void simulationPeriodic() {
    // Set the inputs to the system. Note that we need to convert
    // the [-1, 1] PWM signal to voltage by multiplying it by the
    // robot controller voltage.
    if (automated && auto_forwards != "disabled") {
      if (auto_forwards == "forwards") {
        speedVal = "1";
        m_driveSim.setInputs(
          m_leftMotor.get() * speedMod,
          m_rightMotor.get() * speedMod
        );
      }
      else if (auto_forwards == "backwards") {
        speedVal = "-1";
        m_driveSim.setInputs(
          m_leftMotor.get() * speedMod,
          m_rightMotor.get() * speedMod
        );
      }
    }
    else {
      m_driveSim.setInputs(
          m_leftMotor.get() * speedMod,
          m_rightMotor.get() * speedMod
      );
    }

    if (auto_forwards == "disabled") {
      speedVal = "0";
    }
  
    difDrive.arcadeDrive(Integer.valueOf(speedVal) * speedMod, Integer.valueOf(rotationVal));

    // Advance the model by 20 ms. Note that if you are running this
    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.
    m_driveSim.update(0.02);

    // Update all of our sensors.
    m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());

    m_odometry.update(m_gyro.getRotation2d(),
      m_leftEncoderSim.getDistance(),
      m_rightEncoderSim.getDistance()
    );

    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void periodic() {
    
  }

  public InstantCommand moveDaBot(String speed, String rotation) {
    return new InstantCommand(() -> {  
      if (!automated) {
        if (Integer.valueOf(speed) != 0) {
          speedVal = speed;
        }
        else {
          rotationVal = rotation;
        }
      }
    });
  }

  public InstantCommand shiftSpeed(String modifier) {
    return new InstantCommand(() -> {  
      if (!automated) {
        speedMod += Float.valueOf(modifier);

        if (speedMod > 12f) {
          speedMod = 12f;
        }
        else if (speedMod < 0f) {
          speedMod = 0f;
        }
        
        SmartDashboard.putNumber("SPEED MODIFIER (PRESS BUTTON 6 TO BEGIN MOVING):", speedMod);
        System.out.println(Float.valueOf(speedMod));
      }
    });
  }

  public InstantCommand release(String key) {
    return new InstantCommand(() -> {
      if (!automated) {
        switch (key) { 
          case "a":
            rotationVal = "0";
            break;
            
          case "b":
            speedVal = "0";
            break;
          
          case "x":
            speedVal = "0";
            break;
            
          case "y":
            rotationVal = "0";
            break;
        }
      }
    });
  }

  public InstantCommand automatedDrive(String forwards) {
    return new InstantCommand(() -> {
      System.out.println(forwards);
      automated = true;
      auto_forwards = forwards;
      SmartDashboard.putString("CURRENT STATE:", "AUTOMATED, " + forwards);
    });
  }

  public InstantCommand disableAutomatedDrive() {
    return new InstantCommand(() -> {
      automated = false; 
      auto_forwards = "down";
      speedMod = 2f;
      speedVal = "0";
      SmartDashboard.putNumber("SPEED MODIFIER (PRESS BUTTON 6 TO BEGIN MOVING):", speedMod);
      SmartDashboard.putString("CURRENT STATE:", "TELEOPERATED");
    });
  }
}
