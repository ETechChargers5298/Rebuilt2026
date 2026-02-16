// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.Locale.LanguageRange;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
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

public class Turret extends SubsystemBase {

  // TURRET FIELDS
  private static Turret instance;
  private TalonFX turretMotor;
  // private RelativeEncoder turretEncoder; // turret sensor LEFT/RIGHT() == relative position with throughbore

  
  public double distanceFromHub = 0;
  public double angleToHub = 0;
  public double turretMotorSpeed = 0;
  public double turretAngle = 0;
  public double angleAngler = 0;

  // TURRET CONSTRUCTOR
  private Turret() {
    //turretMotor = new SparkMax(Ports.TURRET_MOTOR_PORT, MotorType.kBrushless);
    turretMotor = new TalonFX(Ports.TURRET_MOTOR_PORT);
    // turretEncoder = turretMotor.getAlternateEncoder();    //REV throughbore connected to Turret Sparkmax
    
  }

  // TURRET SINGLETON
  public static Turret getInstance(){
    if (instance == null)
      instance = new Turret();
      return instance;
  }
    
  // BASIC TURRET METHODS
  public void aimRight(){
    turretMotor.set(1.0);
  }

  public void aimLeft(){
    turretMotor.set(-1.0);
  }

  public void aimTurret(double direction){
    turretMotor.set(direction);
  }

  //  public double getTurretAngle(){ 
  //   return turretEncoder.getPosition();
  // }

  // BASIC TURRET COMMANDS

  // In-line Command to rotate the turret based on provided speed
  public Command aimTurretCommand(DoubleSupplier speedSupplier) {
    return run(
      () -> {
        aimTurret(speedSupplier.getAsDouble()/4);
      });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("turretAngle", getTurretAngle());
  }
}
