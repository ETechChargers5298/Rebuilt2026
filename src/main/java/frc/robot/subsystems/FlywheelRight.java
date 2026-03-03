package frc.robot.subsystems;

public class FlywheelRight {
    private static FlywheelRight instance;

        private FlywheelRight(){
            super();
        }

    // FLYWHEEL SINGLETON
    public static FlywheelRight getInstance(){
    if (instance == null)
      instance = new FlywheelRight();
      return instance;
  }

}
