package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;


public class IntakeRollers extends SubsystemBase {

  // INTAKE FIELDS
  private static IntakeRollers instance;
  private SparkMax eatMotor;
  private double setSpeed;

  // INTAKE CONSTRUCTOR
  private IntakeRollers() {
    eatMotor = new SparkMax(Ports.EAT_MOTOR_PORT,MotorType.kBrushless);
    // new SparkMaxConfig().smartCurrentLimit(60);
    // eatMotor.configure( new SparkMaxConfig().smartCurrentLimit(60));
  }

  // INTAKE SINGLETON - ensures only 1 instance of Intake is constructed
  public static IntakeRollers getInstance() {
    if (instance == null) {
      instance = new IntakeRollers();
    }  
    return instance;
  }

  // BASIC INTAKE METHODS

  public void eat() {
    setSpeed = -IntakeConstants.EAT_SPEED;
    eatMotor.set(setSpeed);
  }
  
  public void spit()
  {
    setSpeed = IntakeConstants.EAT_SPEED;
    eatMotor.set(setSpeed);
  }

  public void stopEating()
  {
    setSpeed = 0;
    eatMotor.set(setSpeed);
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


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Intake Roller Speed", setSpeed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
