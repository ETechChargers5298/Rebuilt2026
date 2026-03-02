// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.MechConstants;


public class TurretLeft extends Turret{

    private static TurretLeft instance;

    private TurretLeft (){
        super("LEFT", MechConstants.LEFT_TURRET_X_OFFSET, MechConstants.LEFT_TURRET_Y_OFFSET);

    }


    
    //TurretLeft SINGLETON
  public static TurretLeft getInstance(){
    if (instance == null)
      instance = new TurretLeft();
      return instance;
  }








}
