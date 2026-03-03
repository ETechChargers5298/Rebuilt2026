package frc.robot.subsystems;

public class AnglerRight extends Angler{

    private static AnglerRight instance;

    private AnglerRight(){
        super();
    }

     // AnglerRight SINGLETON
  public static AnglerRight getInstance(){
  if (instance == null)
    instance = new AnglerRight();
    return instance;
  }


}
