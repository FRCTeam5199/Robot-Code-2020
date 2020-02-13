package frc.shooter;

import frc.controllers.*;
import frc.shooter.*;

public class BallHandler{
    private Shooter shooter;
    private Hopper hopper;
    private Intake intake;
    private JoystickController joy;

    public void init(){
        joy = new JoystickController(1);
        shooter = new Shooter();
        hopper = new Hopper();
        intake = new Intake();
        shooter.init();
        hopper.init();
        intake.init();
    }
    /**
     * Update all the mechanisms being handled by the BallHandler.
     */
    public void update(){
        if(joy.getHat()==180){
            intake.setIntake(1);
        }
        else if(joy.getHat()==0){
            intake.setIntake(-1);
        }
        else{
            intake.setIntake(0);
        }

        if(joy.getButton(1)){
            shooter.toggle(true);
            hopper.setAgitator(true);
            hopper.setIndexer(shooter.spunUp()&&shooter.validTarget());
        }
        else{
            shooter.toggle(false);
            hopper.setAgitator(false);
            hopper.setIndexer(false);
        }
        if(joy.getButton(11)){
            shooter.toggle(joy.getButton(8));
            hopper.setAgitator(joy.getButton(10));
            hopper.setIndexer(joy.getButton(12));
        }
        updateMechanisms();
    }

    private void updateMechanisms(){
        shooter.update();
        intake.update();
        hopper.update();
    }

    public void closeLoggers(){
        shooter.closeLogger();
    }
}