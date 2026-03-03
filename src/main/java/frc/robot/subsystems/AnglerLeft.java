package frc.robot.subsystems;

import frc.robot.Ports;


public class AnglerLeft extends Angler {

  // ANGLERLEFT FIELDS
  private static AnglerLeft instance;

  // ANGLERLEFT CONSTRUCTOR
  private AnglerLeft(){
    super("LEFT",
      Ports.ANGLER_LEFT_MOTOR_PORT,
      Ports.ANGLER_LEFT_LIMIT_SWITCH_PORT
    );
  }

  // AnglerLeft SINGLETON
  public static AnglerLeft getInstance(){
    if (instance == null){
      instance = new AnglerLeft();
    }
    return instance;
  }


}
