package frc.robot.subsystems;

import frc.robot.Constants.MechConstants;


public class ScorerLeft extends Scorer{

    // SCORERLEFT FIELDS
    private static ScorerLeft instance;

    // SCORERLEFT CONSTRUCTOR
    private ScorerLeft(){
        super("LEFT",
            MechConstants.LEFT_SCORER_X_OFFSET, 
            MechConstants.LEFT_SCORER_Y_OFFSET    
        );
    }

    // SCORERLEFT SINGLETON
    public static ScorerLeft getInstance(){
        if (instance == null){
            instance = new ScorerLeft();
        }
        return instance;
    }




}
