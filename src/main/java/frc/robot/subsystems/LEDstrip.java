package frc.robot.subsystems;

import frc.robot.LEDColors;
import frc.robot.Ports;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Flywheel.revFlywheel;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDstrip extends SubsystemBase {

    private static LEDstrip instance;
    private static Spark LED = new Spark(Ports.BLINKIN_PORT);
    // ON OR OFF
    public static final double DISABLED = LEDColors.BLACK;
    public static final double ENABLED = LEDColors.BLUE;

    //SCORER
    public static final double IN_SCORER = LEDColors.GREEN;
   public static final double SCORE_READY = LEDColors.RAINBOW_BPM;

    //INTAKE
    public static final double INTAKING = LEDColors.YELLOW;
    public static final double SPITING = LEDColors.GRAY;
    public static final double FULL = LEDColors.AQUA;
    public static final double FUEL_STUCK = LEDColors.RED_HEARTBEAT;

    //HOPPER
    public static final double FUEL_IN = LEDColors.BLUE_HEARTBEAT;
    //public static final double


    //ABOUT THE ROBOT
    public static final double STUCK = LEDColors.RED;

    //IN THE GAME
    public static final double GET_READY = LEDColors.RAINBOW_TWINKLES;

    //VISION
    public static final double SHOOT = LEDColors.GOLD;

    //AUTO
    public static final double AUTO = LEDColors.VIOLET;


    private static int topCurrentPriority = 0;
    private static double[] patternArray = new double[5];

    //LED SINGLETON
    public static LEDstrip getInstance() {
        if(instance == null){
            instance = new LEDstrip()
        }
        return instance;
    }
       // Enum to determine which subsystem the light pattern is for
    public enum SubsystemPriority {
        
        FLYWHEEL(1),
        CORAL(2),
        ALGAE(3),
        ELCORAL(4),
        DEFAULT(0);

        private int priority;

        private SubsystemPriority(int priority){
            this.priority = priority;
        }

        public int get(){
            return priority;
        }
    }

    // primary method to set the lights
    public static void request(SubsystemPriority priority, double lightColor) {
        // update the top priority
        if (priority.get() > topCurrentPriority) {
            topCurrentPriority = priority.get();
        }
        // recording the request in the array
        patternArray[priority.get()] = lightColor;
    }

    public static void disable() {
        LED.stopMotor();
    }


    //---------- PRIVATE METHODS ---------//
    private static void setPattern(double ledPattern) {
        LED.set(ledPattern);
    }

    private static void setStatus() {
        // turn light on for the top priority reqeust
        if (topCurrentPriority < patternArray.length) {
            setPattern(patternArray[topCurrentPriority]);
        }

        topCurrentPriority = SubsystemPriority.DEFAULT.get();

        SmartDashboard.putNumber("Top Priority", topCurrentPriority);
        SmartDashboard.putNumber("LED Value", patternArray[topCurrentPriority]);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        request(SubsystemPriority.DEFAULT, ENABLED);
        setStatus();

        //FLYWHEEL MAX SPEED
        boolean ShootReadyFlywheel = instance.revFlywheel() = 1;
    }


}
