package frc.robot.subsystems;

import frc.robot.Ports;


public class FlywheelLeft extends Flywheel{

  // FLYWHEELLEFT FIELDS
  private static FlywheelLeft instance;

  // FLYWHEELLEFT CONSTRUCTOR
  private FlywheelLeft(){
    super("LEFT", Ports.FLYWHEEL_LEFT_MOTOR_PORT);
  }

  // FLYWHEEL SINGLETON
  public static FlywheelLeft getInstance(){
    if (instance == null){
      instance = new FlywheelLeft();
    }
    return instance;
  }



}
