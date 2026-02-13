// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;


public class Loader extends SubsystemBase {

  // LOADER FIELDS
  private static Loader instance;
  private SparkMax loaderMotor;

  // LOADER CONSTRUCTOR
  private Loader() {
    loaderMotor = new SparkMax(Ports.LOADER_MOTOR_PORT, MotorType.kBrushless);
  }

  // LOADER SINGLETON
  public static Loader getInstance() {
    if(instance == null){
      instance = new Loader();
    }
    return instance;
  }

  // BASIC LOADER METHODS

  public void loadFuel()
  {
    loaderMotor.set(1);
  }

  public void unloadFuel()
  {
    loaderMotor.set(-1);
  }

  public void generalLoading(double speed)
  {
    loaderMotor.set(speed);
  }

  public void stopLoading()
  {
    loaderMotor.set(0);
  }


  // BASIC LOADER COMMANDS

  // In-line Command to load the ball into the launcher to shoot
  public Command loadInCommand() {
  return run(
      () -> {
        loadFuel();
      });
  }

  // In-line Command to UNLOAD a ball stuck in the loader
  public Command unloadCommand() {
    return run(
        () -> {
          unloadFuel();
        });
  }

  // In-line Command to stop the loader from moving
  public Command stopLoadCommand() {
    return run(
        () -> {
          stopLoading();
        });
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}