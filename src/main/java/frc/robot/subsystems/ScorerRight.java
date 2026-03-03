package frc.robot.subsystems;

import frc.robot.Constants.MechConstants;


public class ScorerRight extends Scorer{

    // SCORERLEFT FIELDS
    private static ScorerRight instance;

    // SCORERLEFT CONSTRUCTOR
    private ScorerRight(){
        super("RIGHT",
            MechConstants.RIGHT_SCORER_X_OFFSET, 
            MechConstants.RIGHT_SCORER_Y_OFFSET    
        );
    }

    // SCORERLEFT SINGLETON
    public static ScorerRight getInstance(){
        if (instance == null){
            instance = new ScorerRight();
        }
        return instance;
    }


}
