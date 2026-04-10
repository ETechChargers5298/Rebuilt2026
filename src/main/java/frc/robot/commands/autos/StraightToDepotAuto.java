// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AnglerLeft;
import frc.robot.subsystems.FlywheelLeft;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.ScorerLeft;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StraightToDepotAuto extends SequentialCommandGroup {
  /** Creates a new StraightToDepot. */
  public StraightToDepotAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // 0. Disable vision
      Vision.getInstance().disable(),
      // 1. Bring intake down
      new WaitCommand(1.2).deadlineFor(IntakePivot.getInstance().extendCommand()),
      
      // 2. Start intake
      new PathPlannerAuto("StraightToDepot Auto 1").deadlineFor(IntakeRollers.getInstance().eatFuelCommand()),
      
      // 3. Re-enable Vision
      Vision.getInstance().enable(),

      // 4. Aim turret and angler to Hub
      new ParallelDeadlineGroup(
        new WaitCommand(2),
        ScorerLeft.getInstance().AimToTarget()
      ),
        
      // 5. Begin revving flywheel
      // new WaitCommand(3).deadlineFor(FlywheelLeft.getInstance().revFlywheelCommand()),
      
      // 6. Load while flywheel keeps revving for 9 seconds
      
      new ParallelDeadlineGroup(
        
        new WaitCommand(4), 
        Loader.getInstance().loadInCommand(),
        IntakePivot.getInstance().sixSevenCommand(),
        // FlywheelLeft.getInstance().revFlywheelCommand()
        ScorerLeft.getInstance().AimToTarget()
      ),

      new PathPlannerAuto("StraightToDepot Auto 2").deadlineFor(
        IntakeRollers.getInstance().eatFuelCommand(),
        IntakePivot.getInstance().sixSevenCommand(),
        ScorerLeft.getInstance().AimToTarget(),
        Loader.getInstance().loadInCommand()
      ),


      new ParallelDeadlineGroup(
        
        new WaitCommand(6), 
        Loader.getInstance().loadInCommand(),
        IntakePivot.getInstance().sixSevenCommand(),
        // FlywheelLeft.getInstance().revFlywheelCommand()
        ScorerLeft.getInstance().AimToTarget()
      )

    );  // end addcommands
  }
}
