package frc.robot.subsystems;

public class AnglerLeft extends Angler {

    private static AnglerLeft instance;

    private AnglerLeft(){
       super(); 
    }

     // AnglerLeft SINGLETON
  public static AnglerLeft getInstance(){
  if (instance == null)
    instance = new AnglerLeft();
    return instance;
  }


}
