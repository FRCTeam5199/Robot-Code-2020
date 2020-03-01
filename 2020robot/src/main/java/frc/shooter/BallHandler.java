package frc.shooter;

import edu.wpi.first.wpilibj.Timer;
import frc.controllers.*;
import frc.shooter.*;
import frc.leds.ShooterLEDs;

public class BallHandler{
    //private ShooterLEDs leds;
    public Shooter shooter;
    public Hopper hopper;
    public Intake intake;
    private JoystickController joy;
    public boolean shooting;
    public boolean indexing;

    //private XBoxController xbox;

    public void init(){
        joy = new JoystickController(1);
        //xbox = new XBoxController(0);
        shooter = new Shooter();
        hopper = new Hopper();
        intake = new Intake();
        shooter.init();
        hopper.init();
        intake.init();
        indexing = false;
        //leds = new ShooterLEDs();
        //leds.init();
    }
    /**
     * Update all the mechanisms being handled by the BallHandler.
     */
    public void update(){
        // if(xbox.getButtonDown(1)){
        //     leds.startShootCycle();
        // }
        if(joy.getButtonDown(6)){
            intake.setDeploy(true);
        }
        if(joy.getButtonDown(4)){
            intake.setDeploy(false);
        }
        if(joy.getHat()==180){
            intake.setIntake(1);
            //deploy intake
            //intake.setDeploy(true);
        }
        else if(joy.getHat()==0){
            intake.setIntake(-1);
            //deploy intake
            //intake.setDeploy(true);
        }
        else{
            intake.setIntake(0);
            //deployn't intake
            //intake.setDeploy(false);
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
        indexing = (shooter.spunUp()||spinOverride)&&(shooter.validTarget()||visOverride)&&!runDisable;
        updateMechanisms();
    }

    public void updateMechanisms(){
        shooter.update();
        intake.update();
        hopper.update();
        //leds.update();
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
        boolean runDisable = false;//hopper.disableOverride.getBoolean(false);
        shooter.toggle(true);
        hopper.setAgitator((shooter.spunUp()||spinOverride)&&(shooter.validTarget()||visOverride)&&!runDisable);
        hopper.setIndexer((shooter.spunUp()||spinOverride)&&(shooter.validTarget()||visOverride)&&!runDisable);
    }

    public void stopFiring(){
        shooter.toggle(false);
        hopper.setAgitator(false);
        hopper.setIndexer(false);
        shooting = false;
    }

    private Timer shooterTimer;
    public void setupShooterTimer(){
        shooterTimer = new Timer();
        shooterTimer.stop();
        shooterTimer.reset();
        stopFiring();
    }

    public boolean allBallsFired = false;
    private boolean timerFlag = false;
    public void fireThreeBalls(){
        fireHighAccuracy();
        shooting = true;
        allBallsFired = false;
        //return true if speed has been at target speed for a certain amount of time
        // if(shooter.atSpeed&&shooterTimer.get()>2){
        //     shooterTimer.stop();   //stop the timerasw
        //     //shooterTimer.reset();  //set the timer to zero
        //     stopFiring();          //stop firing
        //     allBallsFired = true;
        // }
        if((shooter.actualRPM>shooter.speed-50)){
            if(!timerFlag){
                shooterTimer.start();
                timerFlag = true;
                System.out.println("Starting Timer");
            }
        }
        if(!(shooter.actualRPM>shooter.speed-50)){
            timerFlag = false;
            shooterTimer.stop();
            System.out.println("Stopping Timer");
            //shooterTimer.reset();
        }
        if((shooter.actualRPM>shooter.speed-50)&&shooterTimer.get()>1.8){
            stopFiring();
            shooterTimer.stop();
            allBallsFired = true;
            System.out.println("STOPPING THINGS!!!!!!");
        }

        System.out.println(shooterTimer.get()+" "+(shooter.actualRPM>shooter.speed-50));
        updateMechanisms();
    }

    public boolean setIntakeState(boolean intakeState){
        int intakeRun = 0;
        if(intakeState){intakeRun = 1;}
        intake.setIntake(intakeRun);
        updateMechanisms();
        return true;
    }
}