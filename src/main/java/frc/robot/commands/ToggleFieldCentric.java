package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

public class ToggleFieldCentric extends Command{
    private Drivetrain drivetrain;

    public ToggleFieldCentric() {
        drivetrain = Drivetrain.getInstance();
    }

    //Toggles fieldCentric on/off
    @Override
    public void initialize(){
        drivetrain.setFieldCentric(!drivetrain.fieldCentric);
    }


}
