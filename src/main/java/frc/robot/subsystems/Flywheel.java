// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.function.DoubleSupplier;
// import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Flywheel extends SubsystemBase {
  
  // FLYWHEEL FIELDS
  private static Flywheel instance;
  private SparkMax flywheelMotor;
  private RelativeEncoder flywheelEncoder; // Flywheel speed sensor (in sparkmax)
  public double setSpeed;

  // FLYWHEEL SINGLETON
  public static Flywheel getInstance(){
    if (instance == null)
      instance = new Flywheel();
      return instance;
  }

  // FLYWHEEL CONSTRUCTOR
  private Flywheel() {
    flywheelMotor = new SparkMax(Ports.FLYWHEEL_MOTOR_PORT, MotorType.kBrushless);
    flywheelEncoder = flywheelMotor.getEncoder();  //Built in encoder
  }

  // BASIC FLYWHEEL METHODS
  public void revFlywheel(){
    flywheelMotor.set(setSpeed);
  }

  public void stopFlywheel(){
    flywheelMotor.set(0.0);
  }

  public double getLauncherSpeed(){
    //  StatusSignal<AngularVelocity> velocitySignal = launcherMotor.getVelocity();
    // return velocitySignal.getValueAsDouble();
    return flywheelEncoder.getVelocity();
  }

  public void speedUp(double direction){
    flywheelMotor.set(direction);
  }


  // BASIC FLYWHEEL COMMANDS

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
  
  public Command flyWheelCommand(DoubleSupplier speed){
    return run(
      () -> {
        speedUp(speed.getAsDouble());
      
      });
    }

  
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("flyWheelSpeed", getLauncherSpeed());
    SmartDashboard.putNumber("Flywheel setSpeed", setSpeed);
    
  }
}
