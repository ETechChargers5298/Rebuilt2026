package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class Intake extends SubsystemBase {

  // INTAKE FIELDS
  private static Intake instance;
  private SparkMax eatMotor;
  private SparkMax extendMotorRight; // All extendMotor related things are currntly placeholder
  private SparkMax extendMotorLeft; // All extendMotor related things are currntly placeholder
  private RelativeEncoder extendEncoder;


  // INTAKE CONSTRUCTOR
  private Intake() {
    eatMotor = new SparkMax(Ports.EAT_MOTOR_PORT,MotorType.kBrushless);
    extendMotorRight = new SparkMax(Ports.EXTEND_MOTOR_RIGHT_PORT,MotorType.kBrushless);
    extendMotorLeft = new SparkMax(Ports.EXTEND_MOTOR_LEFT_PORT,MotorType.kBrushless);
    extendEncoder = extendMotorRight.getEncoder();
  }

  // INTAKE SINGLETON - ensures only 1 instance of Intake is constructed
  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }  
    return instance;
  }

  // BASIC INTAKE METHODS

  public void eat() {
    eatMotor.set(-1.0);
  }
  
  public void spit()
  {
    eatMotor.set(1.0);
  }

  public void stopEating()
  {
    eatMotor.set(0);
  }

  public void generalExtend(double speed)
  {
    extendMotorRight.set(speed);
    extendMotorLeft.set(-speed);
  }

  public void extend()
  {
    extendMotorRight.set(1.0);
    extendMotorLeft.set(-1.0);
  }

  public void retract()
  {
    extendMotorRight.set(-1.0);
    extendMotorLeft.set(1.0);
  }

  public void stopExtending(){
    
    extendMotorRight.set(0);
    extendMotorLeft.set(0);

  }

  public double getExtendAngle()
  {
    return extendEncoder.getPosition();
  }

  // Check if the fuel is jammed
  public boolean isFuelJam(){
    return false;
  }


  // BASIC INTAKE COMMANDS

  // In-line Command to eat fuel off the ground into the hopper
  public Command eatFuelCommand() {
    return run(
        () -> {
          eat();
        });
  }

  // In-line Command to spit fuel from the hopper back onto the ground
  public Command spitFuelCommand() {
    return run(
        () -> {
          spit();
        });
  }

  // In-line Command to stop moving the intake rollers
  public Command stopEatingCommand(){
    return run(
      () -> {
        stopEating();
      });
  }

  // In-line Command to eat fuel off the ground into the hopper
  public Command extendCommand() {
    return run(
        () -> {
          extend();
        }).finallyDo(
          () -> {
              stopExtending();
          }
        );
  }

  // In-line Command to spit fuel from the hopper back onto the ground
  public Command retractCommand() {
    return run(
        () -> {
          retract();
        }).finallyDo(
          () -> {
              stopExtending();
          }
        );
  }

  // In-line Command to stop moving the intake rollers
  public Command stopExtendingCommand(){
    return run(
      () -> {
        stopExtending();
      });
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Extend Angle", getExtendAngle());
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
