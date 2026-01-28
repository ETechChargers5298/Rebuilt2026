// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Hopper extends SubsystemBase {

  //Fields?
  private static Hopper instance;
  private SparkMax conveyorMotor;
  


  //Hopper Constructer

  private Hopper() 
  {
    conveyorMotor = new SparkMax(Ports.CONVEYOR_MOTOR_PORT, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
  }

  //Hopper Singleton

  public static Hopper getInstance()
  {
    if(instance == null)
    {
      instance = new Hopper();
    }
    return instance;
  }

  //Methods

  public void fowardConvey()
  {
    conveyorMotor.set(1.0);
  }

  public void backConvey()
  {
    conveyorMotor.set(-1.0);
  }

  public void stopConvey()
  {
    conveyorMotor.set(0);
  }

  public void generalMoveConvey(double speed)
  {
    conveyorMotor.set(speed);
  }



  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
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
