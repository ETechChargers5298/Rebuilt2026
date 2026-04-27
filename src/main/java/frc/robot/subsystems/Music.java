package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.traits.CommonDevice;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Music extends SubsystemBase {
    // MUSIC FIELDS
    private static Music instance;
    private static Orchestra orchestra;
      
        //Music Constructor
        
        private Music(){
            orchestra = new Orchestra();

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
    
        public void addInstrument(CommonDevice motor, int track){
            orchestra.addInstrument(motor, track);
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
        System.out.println("Im working");
        return runOnce(
        () -> 
        {
            loadMusic();
            
        });
    }


    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Music playing", orchestra.isPlaying());
        SmartDashboard.putNumber("Music: Time in track", orchestra.getCurrentTime());
        
    }
    
}
