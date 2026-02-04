package frc.robot.subsystems;

import frc.robot.LEDColors;
import frc.robot.Ports;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDstrip {
    // ON OR OFF
    public static final double DISABLED = LEDColors.BLACK;
    public static final double ENABLED = LEDColors.BLUE;

    //SCORER
    public static final double IN_SCORER = LEDColors.GREEN;
   public static final double SCORE_READY = LEDColors.RAINBOW_BPM;

    //INTAKE
    public static final double INTAKING = LEDColors.YELLOW;
    public static final double STUCK = LEDColors.RED;
    public static final double FULL = LEDColors.GOLD;
    public static final double FUEL_STUCK = LEDColors.RED_HEARTBEAT;

    //HOPPER
    //public static final double 


    //
    public static final double AUTO = LEDColors.VIOLET;




}