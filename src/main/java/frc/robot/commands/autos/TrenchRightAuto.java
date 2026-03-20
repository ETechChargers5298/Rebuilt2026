// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrenchRightAuto extends SequentialCommandGroup {
  /** Creates a new TrenchRightAuto. */
  public TrenchRightAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(


    // 1. Bring intake down
    new WaitCommand(1).deadlineFor(IntakePivot.getInstance().extendCommand()),
    
    // 2. Start intake
    new PathPlannerAuto("TrenchRightAuto").deadlineFor(IntakeRollers.getInstance().eatFuelCommand())
    
    // 3. Trip to middle

    //


    );
  }
}
