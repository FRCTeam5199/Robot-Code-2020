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
    private ButtonPanel panel;
    public boolean shooting;
    public boolean indexing;

    //private XBoxController xbox;

    public void init(){
        joy = new JoystickController(1);
        panel = new ButtonPanel(2);
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
        if(panel.getButtonDown(13)){
            shooter.toggle(true);
        }

        if(panel.getButtonDown(9)){
            intake.setDeploy(true);
        }
        if(panel.getButtonDown(8)){
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
        if(panel.getButton(11)){
            hopper.setReverse(true);
        }
        else{
            hopper.setReverse(false);
        }
        indexing = (shooter.spunUp()||spinOverride)&&(shooter.validTarget()||visOverride)&&!runDisable;
        // if(joy.getButton(3)){
        //     shooter.toggle(true);
        // }
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
        hopper.setAgitator((shooter.atSpeed()||spinOverride)&&(shooter.validTarget()||visOverride)&&!runDisable);
        hopper.setIndexer((shooter.atSpeed()||spinOverride)&&(shooter.validTarget()||visOverride)&&!runDisable);
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
        timerFlag = false;
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

        //if the shooter is at speed, reset and start the timer

        if(shooter.atSpeed()){
            if(!timerFlag){
                shooterTimer.reset();
                shooterTimer.start();
                timerFlag = true;
                System.out.println("Starting Timer");
            }
        }
        if(!shooter.atSpeed()){
            timerFlag = false;
            shooterTimer.stop();
            System.out.println("Stopping Timer");
            //shooterTimer.reset();
        }
        if((shooter.atSpeed())&&shooterTimer.get()>0.4){
            stopFiring();
            shooterTimer.stop();
            shooterTimer.reset();
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