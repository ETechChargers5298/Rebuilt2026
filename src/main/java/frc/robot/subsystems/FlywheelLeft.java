package frc.robot.subsystems;

public class FlywheelLeft {
    private static FlywheelLeft instance;

    private FlywheelLeft(){
        super();
    }

      // FLYWHEEL SINGLETON
  public static FlywheelLeft getInstance(){
    if (instance == null)
      instance = new FlywheelLeft();
      return instance;
  }

}
