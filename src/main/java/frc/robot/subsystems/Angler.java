// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Angler extends SubsystemBase {

  // ANGLER FIELDS

  private SparkMax angleMotor;
  private RelativeEncoder anglerEncoder; // Angle sensor UP/DOWN () == Relative position 
  public double anglerAngle = 0;
  private DigitalInput limitSwitch;


  // ANGLER CONSTRUCTOR
  public Angler(String side, int motorPort, int limitPort) {
    angleMotor = new SparkMax(motorPort, MotorType.kBrushless);
    anglerEncoder = angleMotor.getEncoder();
    limitSwitch = new DigitalInput(limitPort);    //REV throughbore connected to Angler Sparkmax
  }


  // BASIC ANGLER METHODS
  public void aimup(){
    angleMotor.set(1);
  }
  public void aimdown(){
    angleMotor.set(-1);
  }

  public void aimAngler(double direction){
    angleMotor.set(-direction);
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

  // In-line Command to angle the angler up to a specific angle - in degrees
  public Command aimAnglerToSetPointCommand( Supplier<Double> anglerAngle) {


    return null;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("anglerAngle", getAnglerAngle());
    if(!limitSwitch.get()){
      anglerEncoder.setPosition(0);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
