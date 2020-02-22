package frc.shooter;

import frc.controllers.*;
import frc.shooter.*;

public class BallHandler{
    private Shooter shooter;
    public Hopper hopper;
    private Intake intake;
    private JoystickController joy;
    public boolean shooting;

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
            //deploy intake
            intake.setDeploy(true);
        }
        else if(joy.getHat()==0){
            intake.setIntake(-1);
            //deploy intake
            intake.setDeploy(true);
        }
        else{
            intake.setIntake(0);
            //deployn't intake
            intake.setDeploy(false);
        }
        boolean visOverride = hopper.visionOverride.getBoolean(false);
        boolean spinOverride = hopper.spinupOverride.getBoolean(false);
        boolean runDisable = hopper.disableOverride.getBoolean(false);
        if(joy.getButton(1)){
            fireHighAccuracy();
            shooting = true;
        }
        else{
            shooter.toggle(false);
            hopper.setAgitator(false);
            hopper.setIndexer(false);
            shooting = false;
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

    public void fireHighSpeed(){
        boolean visOverride = hopper.visionOverride.getBoolean(false);
        boolean spinOverride = hopper.spinupOverride.getBoolean(false);
        boolean runDisable = hopper.disableOverride.getBoolean(false);
        shooter.toggle(true);
        hopper.setAgitator((shooter.spunUp()||shooter.recovering()||spinOverride)&&(shooter.validTarget()||visOverride)&&!runDisable);
        hopper.setIndexer((shooter.spunUp()||shooter.recovering()||spinOverride)&&(shooter.validTarget()||visOverride)&&!runDisable);
    }

    public void fireHighAccuracy(){
        boolean visOverride = hopper.visionOverride.getBoolean(false);
        boolean spinOverride = hopper.spinupOverride.getBoolean(false);
        boolean runDisable = hopper.disableOverride.getBoolean(false);
        shooter.toggle(true);
        hopper.setAgitator((shooter.spunUp()||spinOverride)&&(shooter.validTarget()||visOverride)&&!runDisable);
        hopper.setIndexer((shooter.spunUp()||spinOverride)&&(shooter.validTarget()||visOverride)&&!runDisable);
    }
}