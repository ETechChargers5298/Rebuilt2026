// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.MechConstants;

/** Add your docs here. */
public class TurretRight extends Turret {

    private static TurretRight instance;



    private TurretRight(){
        super("RIGHT", MechConstants.RIGHT_TURRET_X_OFFSET, MechConstants.RIGHT_TURRET_Y_OFFSET);
    }

    //TurretRight SINGLETON
  public static TurretRight getInstance(){
    if (instance == null)
      instance = new TurretRight();
      return instance;
  }


}
