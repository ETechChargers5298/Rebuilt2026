package frc.robot.subsystems;

import frc.robot.Ports;


public class AnglerRight extends Angler {

  // ANGLERRIGHT FIELDS
  private static AnglerRight instance;

  // ANGLERRIGHT CONSTRUCTOR
  private AnglerRight(){
    super("RIGHT",
      Ports.ANGLER_RIGHT_MOTOR_PORT,
      Ports.ANGLER_RIGHT_LIMIT_SWITCH_PORT
    );
  }

  // ANGLERRIGHT SINGLETON
  public static AnglerRight getInstance(){
    if (instance == null){
      instance = new AnglerRight();
    }
    return instance;
  }


}
