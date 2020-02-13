package frc.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.*;
import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.controllers.XBoxController;
import frc.robot.RobotMap;
import frc.robot.RobotNumbers;
import frc.robot.RobotToggles;
import frc.util.Logger;
//import frc.util.Permalogger;
import frc.vision.GoalChameleon;

public class Shooter{
    private final CANSparkMax leader, follower;
    private CANPIDController speedo;
    private CANEncoder encoder;
    private XBoxController xbox;
    private boolean enabled = true;

    private Timer timer = new Timer();
    private Logger logger = new Logger("shooter");
    //private Permalogger permalogger = new Permalogger("shooter");

    private double pulleyRatio = RobotNumbers.motorPulleySize/RobotNumbers.driverPulleySize;

    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    private NetworkTableEntry shooterSpeed = tab.add("Shooter Speed", 0).getEntry();
    private NetworkTableEntry shooterToggle = tab.add("Shooter Toggle", false).getEntry();
    private NetworkTableEntry rampRate = tab.add("Ramp Rate", 40).getEntry();

    public String[] data = {
        "match time", "init time", 
        "speed", "target speed", 
        "motor temperature", "motor current", 
        "powered", 
        "P", "I", "D", 
        "rP", "rI", "rD",
        "distance"
    };
    public String[] units = {"seconds", "seconds", "rpm", "rpm", "C", "A", "T/F", "num", "num", "num", "num", "num", "num", "meters"};

    private double targetRPM;
    private double speed;
    private int ballsShot = 0;
    private boolean poweredState;
    private boolean atSpeed = false;
    private boolean spunUp = false;
    private boolean recoveryPID = false;

    private NetworkTableEntry shooterP = tab.add("P", 0.00032).getEntry();
    private NetworkTableEntry shooterI = tab.add("I", 0).getEntry();
    private NetworkTableEntry shooterD = tab.add("D", 0).getEntry();
    private NetworkTableEntry recP = tab.add("recP", 3e-4).getEntry();
    private NetworkTableEntry recI = tab.add("recI", 1e-7).getEntry();
    private NetworkTableEntry recD = tab.add("recD", 0.07).getEntry();
    private NetworkTableEntry shooterF = tab.add("F", 0.000185).getEntry();

    private double P, I, D, F, recoveryP, recoveryI, recoveryD;
    private double actualRPM;

    private GoalChameleon chameleon;

    public Shooter(){
        leader = new CANSparkMax(RobotMap.shooterLeader, MotorType.kBrushless);
        follower = new CANSparkMax(RobotMap.shooterFollower, MotorType.kBrushless);
        if(RobotToggles.shooterPID){
            speedo = leader.getPIDController();
        }
        leader.setInverted(true);
        follower.follow(leader, true);
        poweredState = false;
        chameleon = new GoalChameleon();
    }

    /**
     * Update the Shooter object.
     */
    public void update(){
        actualRPM = leader.getEncoder().getVelocity();
        checkState();
        speed = shooterSpeed.getDouble(0);
        double rate = rampRate.getDouble(40);
        //boolean toggle = shooterToggle.getBoolean(false);

        // if(leader.getOpenLoopRampRate()!=rate){
        //     leader.setOpenLoopRampRate(rate);
        //     System.out.println("Ramp Rate Set to "+rate+", now "+leader.getOpenLoopRampRate());
        // }

        //3.00E-04	3.50E-07	0.02
        P = RobotNumbers.shooterSpinUpP; //shooterP.getDouble(0);
        I = RobotNumbers.shooterSpinUpI; //shooterI.getDouble(0);
        D = RobotNumbers.shooterSpinUpD; //shooterD.getDouble(0);
        P = shooterP.getDouble(0.00032);
        I = shooterI.getDouble(0);
        D = shooterD.getDouble(0);
        F = shooterF.getDouble(0.000185);
        

        //3.00E-04	1.00E-07	0.07 (tentative values, not perfect yet)
        recoveryP = recP.getDouble(3e-4);
        recoveryI = recI.getDouble(1e-7);
        recoveryD = recD.getDouble(0.07);

        //setPID(P,I,D);

        // if(P!=Pold || I!=Iold || D!=Dold){
        //     setPID(P,I,D);
        //     System.out.println("PID reset");
        // }
        //if(enabled){
            //leader.set(0.05);
        //toggle(toggle);
        if(enabled){
            //poweredState = true;
            if(RobotToggles.shooterPID){
                setSpeed(speed);
            }
            else{
                leader.set(speed);
            }
        }
        else{
            //poweredState = false;
            if(RobotToggles.shooterPID){
                //do nothing because the voltage being set to 0 *should* coast it?
                //to past me: it does
                leader.set(0);
            }
            else{
                leader.set(0);
            }
        }

        SmartDashboard.putNumber("RPM", actualRPM);
        SmartDashboard.putNumber("Target RPM", speed);
        SmartDashboard.putNumber("Drive Wheel RPM", actualRPM*pulleyRatio);
        SmartDashboard.putNumber("Drive Wheel IPS", actualRPM*pulleyRatio*RobotNumbers.driverWheelDiameter*Math.PI);
        SmartDashboard.putNumber("Motor Current", leader.getOutputCurrent());
        SmartDashboard.putNumber("Motor Temp", leader.getMotorTemperature());
        SmartDashboard.putNumber("I accumulator", speedo.getIAccum());
        SmartDashboard.putBoolean("RecMode", recoveryPID);

        
        
        // if(poweredState == true){
        //     leader.setVoltage(12);
        //     follower.setVoltage(12);
        // }
        // else{
        //     leader.setVoltage(0);
        //     follower.setVoltage(0);
        // }
        
        if(RobotToggles.logData){writeData();}
        //System.out.println(leader.getEncoder().getVelocity());
        SmartDashboard.putBoolean("atSpeed", atSpeed);
        SmartDashboard.putNumber("ballsShot", ballsShot);
        SmartDashboard.putBoolean("shooter enable", enabled);
    }

    /**
     * Get motor speed based 
     * @param distance
     * @return
     */
    private double getSpeedBasedOnDistance(double distance){
        //speed of motor = wheel speed/1.5
        //wheel speed = ips/4pi
        //therefore speed of motor = (ips/4pi)/1.5
        //velocity(m/s) = sqrt((49)/(sin(theta))^2)
        
        return 0;
    }

    public boolean validTarget(){
        return chameleon.validTarget();
    }

    private void checkState(){
        if(actualRPM >= speed-50){
            atSpeed = true;
            spunUp = true;
        }
        if(atSpeed && actualRPM < speed-70){
            ballsShot++;
        }
        if(actualRPM < speed-70){
            atSpeed = false;
        }

        if(spunUp && actualRPM<speed-80){
            recoveryPID = true;
        }
        if(actualRPM<10){
            recoveryPID = false;
            spunUp = false;
        }

        if(recoveryPID){
            //setPID(recoveryP, recoveryI, recoveryD, 0);
            setPID(P,I,D, F);
        }
        else{
            setPID(P,I,D, F);
        }
    }

    public boolean spunUp(){
        return spunUp;
    }

    /**
     * Set drive wheel RPM
     * @param rpm
     */
    public void setSpeed(double rpm){
        //System.out.println("setSpeed1");
        speedo.setReference(rpm, ControlType.kVelocity);
        //System.out.println("setSpeed2");
    }

    /**
     * Enable or disable the shooter being spun up.
     * @param toggle - spun up true or false
     */
    public void toggle(boolean toggle){
        enabled = toggle;
    }

    /**
     * Initialize the Shooter object.
     */
    public void init(){
        shooterP.getDouble(0);
        shooterI.getDouble(0);
        shooterD.getDouble(0);

        leader.setSmartCurrentLimit(80);
        follower.setSmartCurrentLimit(80);
        leader.setIdleMode(IdleMode.kCoast);
        follower.setIdleMode(IdleMode.kCoast);

        leader.getEncoder().setPosition(0);
        leader.setOpenLoopRampRate(40);
        
        ballsShot = 0;

        //leader.setInverted(false);
        // follower.setInverted(true);

        speedo = leader.getPIDController();
        encoder = leader.getEncoder();
        //setPID(4e-5, 0, 0);
        speedo.setOutputRange(-1, 1);
        //setPID(1,0,0);

        speedo.setOutputRange(-1, 1);

        chameleon.init();
    }

    /**
     * Set the P, I, and D values for the shooter.
     * @param P - P value
     * @param I - I value
     * @param D - D value
     */
    private void setPID(double P, double I, double D, double F){
        speedo.setP(P);
        speedo.setI(I);
        speedo.setD(D);
        speedo.setFF(F);
    }

    /**
     * Initialize the Shooter logger, run during autonomousInit.
     */
    public void initLogger(){
        System.out.println("attempting to initialize logger - Shooter");
        logger.init(data, units);
        timer.start();
        //permalogger.init();
    }
    /**
     * Close the Shooter logger, call during disabledInit().
     */
    public void closeLogger(){
        //permalogger.writeData(ballsShot);
        //permalogger.close();
        logger.close();
    }
    /**
     * Write shooter data to the log file.
     */
    private void writeData(){
        double powered;
        if(enabled){powered = 1;}else{powered = 0;}
        double[] data = {
            Timer.getMatchTime(), 
            timer.get(), 
            leader.getEncoder().getVelocity(), 
            speed, leader.getMotorTemperature(), 
            leader.getOutputCurrent(), 
            powered, 
            P, I, D, 
            recoveryP, recoveryI, recoveryD,
            chameleon.getGoalDistance()
        };
        logger.writeData(data);
    }
}
