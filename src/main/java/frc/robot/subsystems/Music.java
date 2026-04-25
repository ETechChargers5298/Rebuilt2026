package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.traits.CommonDevice;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Music extends SubsystemBase {
    // WIP, not implemented yet
    // MUSIC FIELDS
    private static Music instance;
    private Orchestra orchestra;
    private CommonDevice motor1;

    private final AudioConfigs audioConfig = new AudioConfigs();
  
    //Music Constructor
    private Music(){
        
        orchestra = new Orchestra();
        orchestra.addInstrument(motor1, 1);
        loadMusic();
        audioConfig.withAllowMusicDurDisable(true);
        SmartDashboard.putData("Play Music", playMusicCommand().ignoringDisable(true));
        SmartDashboard.putData("Stop Music", stopMusicCommand().ignoringDisable(true));
        SmartDashboard.putData("Load Music", loadMusicCommand().ignoringDisable(true));
    }

    //Music Singleton
    public static Music getInstance(){
        if(instance == null){
            instance = new Music();
        }
        return instance;
    }

    //Basic music methods

    public void loadMusic(){
        orchestra.loadMusic("underTheSea.chrp");
    }

    public void playMusic(){
        orchestra.play();
    }

    public void stopMusic(){
        orchestra.stop();
    }

    //Music Commands
    public Command playMusicCommand(){
        return runOnce(
        () -> 
        {
            playMusic();
        
        });
    }

    public Command stopMusicCommand(){
        return runOnce(
        () -> 
        {
            stopMusic();
        
        });
    }
    public Command loadMusicCommand(){
        return runOnce(
        () -> 
        {
            loadMusic();
        
        });
    }


    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Music playing", orchestra.isPlaying());
        
    }
    
}
