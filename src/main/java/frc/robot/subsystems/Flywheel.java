// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.Locale.LanguageRange;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.encoder.DetachedEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class Flywheel extends SubsystemBase {
  /** Creates a new Flywheel. */
  // FIELDS
  private static Flywheel instance;
  private SparkMax flywheelMotor;
  private RelativeEncoder flywheelEncoder; // Flywheel speed sensor (in sparkmax)

  /** Creates a new Turret. */
  public static Flywheel getInstance(){
    if (instance == null)
      instance = new Flywheel();
      return instance;
  }

  private Flywheel() {
    flywheelMotor = new SparkMax(Ports.FLYWHEEL_MOTOR_PORT, MotorType.kBrushless);
    flywheelEncoder = flywheelMotor.getEncoder();  //Built in encoder
  }

  public void revFlywheel(){
    flywheelMotor.set(1.0);
  }

  public void stopFlywheel(){
    flywheelMotor.set(0.0);
  }

  public Command revFlywheelCommand() {
    return run(
      () -> {
        revFlywheel();
      });
  }

  public Command stopFlywheelCommand() {
    return run(
      () -> {
        stopFlywheel();
      });
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
