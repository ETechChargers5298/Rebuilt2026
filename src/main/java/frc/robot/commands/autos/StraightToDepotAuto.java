package frc.robot.commands.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.ScorerLeft;
import frc.robot.subsystems.Vision;


public class StraightToDepotAuto extends SequentialCommandGroup {

  public StraightToDepotAuto() {

    addCommands(

      // 0. Disable vision
      Vision.getInstance().disable(),

      // 1. turn on PivotLimits
      IntakePivot.getInstance().limitOnCommand(),

      // 2. Bring intake down
      new WaitCommand(1.2).deadlineFor(IntakePivot.getInstance().extendCommand()),
      
      // 3. Start Intake Rollers
      new PathPlannerAuto("StraightToDepot Auto 1").deadlineFor(IntakeRollers.getInstance().eatFuelCommand()),
      
      // 4. Re-enable Vision
      Vision.getInstance().enable(),

      // 5. Aim turret and angler to Hub
      new ParallelDeadlineGroup(
        new WaitCommand(2),
        ScorerLeft.getInstance().AimToTarget()
      ),
      
      // 6. Launch fuel with loader with 67 assistance
      new ParallelDeadlineGroup(  
        new WaitCommand(4), 
        Loader.getInstance().loadInCommand(),
        IntakePivot.getInstance().sixSevenCommand(),
        ScorerLeft.getInstance().AimToTarget()
      ),

      // 7. 2nd Path to depot
      new PathPlannerAuto("StraightToDepot Auto 2").deadlineFor(
        IntakeRollers.getInstance().eatFuelCommand(),
        //IntakePivot.getInstance().sixSevenCommand(), // fuel will not go into intake when this is happening
        ScorerLeft.getInstance().AimToTarget()
      ),

      // 8. 3rd Path for closer shot
      new PathPlannerAuto("StraightToDepot Auto 3").deadlineFor(
        IntakeRollers.getInstance().eatFuelCommand(),
        ScorerLeft.getInstance().AimToTarget()
      ),

      //9. Aim + Shoot Fuel for second volley
      new ParallelDeadlineGroup(
        new WaitCommand(6),
        ScorerLeft.getInstance().AimToTarget(),
        Loader.getInstance().loadInCommand(),
        IntakePivot.getInstance().sixSevenCommand()

      )

    );  // end addcommands
  }
}
