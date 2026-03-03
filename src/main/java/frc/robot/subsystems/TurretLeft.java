package frc.robot.subsystems;

import frc.robot.Ports;


public class TurretLeft extends Turret{

  // TURRETLEFT FIELDS
  private static TurretLeft instance;

  // TURRETLEFT CONSTRUCTOR
  private TurretLeft (){
      super("LEFT", Ports.TURRET_LEFT_MOTOR_PORT);
  }

  //TURRETLEFT SINGLETON
  public static TurretLeft getInstance(){
    if (instance == null){
      instance = new TurretLeft();
    }
    return instance;
  }



}
