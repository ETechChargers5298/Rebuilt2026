package frc.robot.subsystems;

import frc.robot.Ports;


public class FlywheelRight extends Flywheel{

  // FLYWHEELLEFT FIELDS
  private static FlywheelRight instance;

  // FLYWHEELLEFT CONSTRUCTOR
  private FlywheelRight(){
    super("RIGHT", Ports.FLYWHEEL_RIGHT_MOTOR_PORT);
  }

  // FLYWHEEL SINGLETON
  public static FlywheelRight getInstance(){
    if (instance == null){
      instance = new FlywheelRight();
    }
    return instance;
  }



}
