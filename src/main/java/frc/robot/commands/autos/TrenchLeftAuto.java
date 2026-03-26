// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.FlywheelLeft;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.ScorerLeft;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrenchLeftAuto extends SequentialCommandGroup {
  /** Creates a new TrenchLeftAuto. */
  public TrenchLeftAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(


    // 1. Bring intake down
      new WaitCommand(1).deadlineFor(IntakePivot.getInstance().extendCommand()),
      
    
      // 2. Start intake
      new PathPlannerAuto("TrenchLeftAuto").deadlineFor(IntakeRollers.getInstance().eatFuelCommand()),
      
      // 3. wait 1 second
      // new WaitCommand(1),

      // 4. Aim turret and angler to Hub
      new ParallelDeadlineGroup(
        new WaitCommand(3),
        ScorerLeft.getInstance().AimToTarget()
      ),
        
      // 5. Begin revving flywheel
      // new WaitCommand(4).deadlineFor(FlywheelLeft.getInstance().revFlywheelCommand()),
      
      // 6. Load while flywheel keeps revving for 8 seconds
      new ParallelDeadlineGroup(
        new WaitCommand(8), 
        Loader.getInstance().loadInCommand(),
        // FlywheelLeft.getInstance().revFlywheelCommand()
        ScorerLeft.getInstance().AimToTarget()
      )

    );  // end addcommands
  }
}
