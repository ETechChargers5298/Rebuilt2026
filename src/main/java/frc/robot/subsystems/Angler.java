// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Angler extends SubsystemBase {

  // ANGLER FIELDS
  private static Angler instance;
  private SparkMax angleMotor;
  private RelativeEncoder anglerEncoder; // Angle sensor UP/DOWN () == Relative position 
  public double angleAngler = 0;


  // ANGLER SINGLETON
  public static Angler getInstance(){
  if (instance == null)
    instance = new Angler();
    return instance;
  }

  // ANGLER CONSTRUCTOR
  public Angler() {
    angleMotor = new SparkMax(Ports.ANGLE_MOTOR_PORT, MotorType.kBrushless);
    anglerEncoder = angleMotor.getAlternateEncoder();   //REV throughbore connected to Angler Sparkmax
  }


  // BASIC ANGLER METHODS

  public void aimup(){
    angleMotor.set(1);
  }
  public void aimdown(){
    angleMotor.set(-1);
  }

  public void aimAngler(double direction){
    angleMotor.set(direction);
  }

  public double getAnglerAngle(){
    return anglerEncoder.getPosition(); 
  }


  // BASIC ANGLER COMMANDS

  // In-line Command to move the hood down, producing a higher launch angle (closer shots)
  public Command angleUpCommand() {
    return run(
      () -> {
        aimup();
      });
  }

  // In-line Command to rotate the pitch angle of the hood based on provided speed
  public Command aimAnglerCommand(DoubleSupplier speedSupplier) {
    return run(
      () -> {
        aimAngler(speedSupplier.getAsDouble());
      });
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("anglerAngle", getAnglerAngle());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
