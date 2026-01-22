// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkBase.SoftLimitDirection;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.SparkAbsoluteEncoder.Type;

public class Intake extends SubsystemBase {

// Fields
private static Intake instance;
private SparkMax eatMotor;

private SparkMax extendMotor; // All extendMotor related things are currntly placeholder
private AbsoluteEncoder extendEncoder;


  /** Creates a new ExampleSubsystem. */
  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }  
    return instance;
  }

  private Intake() {
    eatMotor = new SparkMax(Ports.EAT_MOTOR_PORT,MotorType.kBrushless);
    extendMotor = new SparkMax(Ports.EXTEND_MOTOR_PORT,MotorType.kBrushless);
    extendEncoder = extendMotor.getAbsoluteEncoder();
  }

  // Other Methods
  public void eat(){
    eatMotor.set(1.0);
  }
  
  public void spit()
  {
    eatMotor.set(-1.0);
  }

  public void stopEating()
  {
    eatMotor.set(0);
  }

  public void generalExtend(double speed)
  {
    extendMotor.set(speed);
  }

  public void extend()
  {
    extendMotor.set(1.0);
  }

  public void extract()
  {
    extendMotor.set(-1.0);
  }

  public double getExtendAngle()
  {
    return extendEncoder.getPosition();
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
