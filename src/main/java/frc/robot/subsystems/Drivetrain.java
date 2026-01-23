package frc.robot.subsystems;


public class Drivetrain {

    private static Drivetrain instance;
    
    // Drivetrain Constructor
    private Drivetrain() {

    }
    
    // Drivetrain Singleton - ensures only 1 instance of Drivetrain is constructed
    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;
    }


}
