package frc.robot.commands.complex;


import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Drivetrain;


public class SwerveDrive extends Command {

  private static Drivetrain drivetrain;
  private Supplier<Double> xSpeed;
  private Supplier<Double> ySpeed;
  private Supplier<Double> rotSpeed;
  private SlewRateLimiter xFilter;
  private SlewRateLimiter yFilter;
  private Supplier<Boolean> fieldReset;

  
  /** Creates a new SwerveDrive. */
  public SwerveDrive(Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotSpeed, Supplier<Boolean> fieldReset) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotSpeed = rotSpeed;
    this.fieldReset = fieldReset;
    xFilter = new SlewRateLimiter(1.2);
    yFilter = new SlewRateLimiter(1.2);


    drivetrain = Drivetrain.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.stopDrive();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.allianceCentric = true;
    
    drivetrain.setXSpeed(MathUtil.applyDeadband(xSpeed.get(), 0.1) * SwerveConstants.TOP_SPEED);
    drivetrain.setYSpeed(MathUtil.applyDeadband(ySpeed.get(), 0.1) * SwerveConstants.TOP_SPEED);
    drivetrain.setRotSpeed(MathUtil.applyDeadband(rotSpeed.get(), 0.1) * SwerveConstants.TOP_ANGULAR_SPEED);

    // drivetrain.move(
    //   MathUtil.applyDeadband(xSpeed.get(), 0.1), 
    //   MathUtil.applyDeadband(ySpeed.get(), 0.1), 
    //   MathUtil.applyDeadband(rotSpeed.get(), 0.1), 
    //   true);

      // xFilter.calculate(MathUtil.applyDeadband(xSpeed.get(), 0.1)), 
      // yFilter.calculate(MathUtil.applyDeadband(ySpeed.get(), 0.1)),
    
      if(fieldReset.get()) {
        drivetrain.zeroRobotHeading();
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      drivetrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
