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

import frc.controllers.XBoxController;
import frc.robot.RobotMap;
import frc.robot.RobotNumbers;
import frc.robot.RobotToggles;
import frc.util.Logger;

import edu.wpi.first.wpilibj.Timer;

public class Shooter{
    private final CANSparkMax leader, follower;
    private CANPIDController speedo;
    private CANEncoder encoder;
    private XBoxController xbox;
    private boolean enabled = true;

    private Timer timer = new Timer();
    private Logger logger = new Logger("shooter");

    private double pulleyRatio = RobotNumbers.motorPulleySize/RobotNumbers.driverPulleySize;

    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    private NetworkTableEntry shooterSpeed = tab.add("Shooter Speed", 0).getEntry();
    private NetworkTableEntry shooterToggle = tab.add("Shooter Toggle", false).getEntry();
    private NetworkTableEntry rampRate = tab.add("Ramp Rate", 40).getEntry();

    public String[] data = {"match time", "init time", "speed", "target speed", "motor temperature", "motor current"};
    public String[] units = {"seconds", "seconds", "rpm", "rpm", "C", "A"};

    private double targetRPM;
    private double speed;
    private int ballsShot;

    // private NetworkTableEntry shooterP = tab.add("P", 0).getEntry();
    // private NetworkTableEntry shooterI = tab.add("I", 0).getEntry();
    // private NetworkTableEntry shooterD = tab.add("D", 0).getEntry();

    public Shooter(){
        leader = new CANSparkMax(RobotMap.shooterLeader, MotorType.kBrushless);
        follower = new CANSparkMax(RobotMap.shooterFollower, MotorType.kBrushless);
        //speedo = leader.getPIDController();
        follower.follow(leader, true);
    }

    /**
     * Update the Shooter object.
     */
    public void update(){
        speed = shooterSpeed.getDouble(0);
        double rate = rampRate.getDouble(40);
        boolean toggle = shooterToggle.getBoolean(false);
        
        boolean atSpeed;

        if(leader.getOpenLoopRampRate()!=rate){
            leader.setOpenLoopRampRate(rate);
            System.out.println("Ramp Rate Set to "+rate+", now "+leader.getOpenLoopRampRate());
        }

        // double Pold = speedo.getP();
        // double Iold = speedo.getI();
        // double Dold = speedo.getD();
        // double P = shooterP.getDouble(0);
        // double I = shooterI.getDouble(0);
        // double D = shooterD.getDouble(0);
        // if(P!=Pold || I!=Iold || D!=Dold){
        //     setPID(P,I,D);
        //     System.out.println("PID reset");
        // }
        //if(enabled){
            //leader.set(0.05);
        toggle(toggle);
        if(enabled){
            leader.set(speed);
            //setSpeed(speed);
        }
        else{
            leader.set(0);
            //setSpeed(0);
        }

        
        // }
        // else{
        //     setSpeed(0);
        // }
        double actualRPM = leader.getEncoder().getVelocity();
        SmartDashboard.putNumber("RPM", actualRPM);
        SmartDashboard.putNumber("Target RPM", speed);
        SmartDashboard.putNumber("Drive Wheel RPM", actualRPM*pulleyRatio);
        SmartDashboard.putNumber("Drive Wheel IPS", actualRPM*pulleyRatio*RobotNumbers.driverWheelDiameter*Math.PI);
        SmartDashboard.putNumber("Motor Current", leader.getOutputCurrent());
        SmartDashboard.putNumber("Motor Temp", leader.getMotorTemperature());
        
        if(RobotToggles.logData){writeData();}
        //System.out.println(leader.getEncoder().getVelocity());
    }

    // public void setSpeed(double rpm){
    //     //System.out.println("setSpeed1");
    //     speedo.setReference(rpm, ControlType.kVelocity);
    //     //System.out.println("setSpeed2");
    // }

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
        // shooterP.getDouble(0);
        // shooterI.getDouble(0);
        // shooterD.getDouble(0);

        leader.setSmartCurrentLimit(40);
        follower.setSmartCurrentLimit(40);
        leader.setIdleMode(IdleMode.kCoast);
        follower.setIdleMode(IdleMode.kCoast);

        leader.getEncoder().setPosition(0);
        leader.setOpenLoopRampRate(40);     
        
        ballsShot = 0;

        //leader.setInverted(false);
        // follower.setInverted(true);

        //speedo = leader.getPIDController();
        //encoder = leader.getEncoder();
        //setPID(4e-5, 0, 0);
        //speedo.setOutputRange(-1, 1);
        //setPID(1,0,0);

        //speedo.setOutputRange(-1, 1);
    }

    /**
     * Set the P, I, and D values for the shooter.
     * @param P - P value
     * @param I - I value
     * @param D - D value
     */
    private void setPID(double P, double I, double D){
        // speedo.setP(P);
        // speedo.setI(I);
        // speedo.setD(D);
    }

    /**
     * Initialize the Shooter logger, run during autonomousInit.
     */
    public void initLogger(){
        System.out.println("attempting to initialize logger - Shooter");
        logger.init(data, units);
        timer.start();
    }
    /**
     * Close the Shooter logger, run during disabledInit().
     */
    public void closeLogger(){
        logger.close();
    }
    /**
     * Write shooter data to the log file.
     */
    private void writeData(){
        double[] data = {Timer.getMatchTime(), timer.get(), leader.getEncoder().getVelocity(), speed, leader.getMotorTemperature(), leader.getOutputCurrent()};
        logger.writeData(data);
    }
}
