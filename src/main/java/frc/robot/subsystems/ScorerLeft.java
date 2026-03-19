package frc.robot.subsystems;

import frc.robot.Constants.*;


public class ScorerLeft extends Scorer{

    // SCORERLEFT FIELDS
    private static ScorerLeft instance;

    // SCORERLEFT CONSTRUCTOR
    private ScorerLeft(){
        super("LEFT",
            ScorerConstants.LEFT_SCORER_X_OFFSET, 
            ScorerConstants.LEFT_SCORER_Y_OFFSET    
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
