package frc.robot.subsystems;

import frc.robot.Ports;


public class TurretRight extends Turret {

  // TURRETRIGHT FIELDS
  private static TurretRight instance;

  // TURRETRIGHT CONSTRUCTOR
  private TurretRight(){
      super("RIGHT", Ports.TURRET_RIGHT_MOTOR_PORT);
  }

  //TURRETRIGHT SINGLETON
  public static TurretRight getInstance(){
    if (instance == null){
      instance = new TurretRight();
    }
    return instance;
  }


}
