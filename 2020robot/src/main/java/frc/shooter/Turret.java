package frc.shooter;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotNumbers;
import frc.vision.GoalChameleon;
import frc.controllers.XBoxController;

import java.io.IOException;

import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Turret{
    private double sprocketRatio = 1; //replace 1 with ratio between motor and turret sprocket(turret/motor)
    private double gearingRatio = 1; //replace with whatever number

    private CANSparkMax motor;
    private CANEncoder encoder;
    private CANPIDController controller;
    private PIDController positionControl;
    private PigeonIMU pigeon;
    //private PIDController control;
    private double driveOmega;
    private double turretOmega;
    private double robotYaw;
    private double startYaw;
    private double[] ypr;
    private double[] startypr;

    private double fMultiplier;
    private double targetPosition;
    private ShuffleboardTab tab = Shuffleboard.getTab("Turret");
    private NetworkTableEntry fMult = tab.add("F Multiplier", 0).getEntry();
    private NetworkTableEntry pos = tab.add("Position", 0).getEntry();
    private NetworkTableEntry p = tab.add("P", 0).getEntry();
    private NetworkTableEntry i = tab.add("I", 0).getEntry();
    private NetworkTableEntry d = tab.add("D", 0).getEntry();

    private GoalChameleon chameleon;

    public Turret(){
        motor = new CANSparkMax(5, MotorType.kBrushless);
        encoder = motor.getEncoder();
        pigeon = new PigeonIMU(RobotMap.pigeon);
        chameleon = new GoalChameleon();
    }

    public void update(){
        fMultiplier = fMult.getDouble(0);
        targetPosition = pos.getDouble(0);
        setMotorPID(p.getDouble(0), i.getDouble(0), d.getDouble(0));
        //turretOmega = -driveOmega*RobotNumbers.turretRotationSpeedMultiplier;
        //double motorOmega = turretOmega*sprocketRatio;    
        
        //!!!!! THE TURRET ZERO IS THE MECHANICAL STOP CLOSEST TO THE GOAL

        /*things to do:
        check if there is a valid target, if not, face north based on gyro
        if there is a valid target, point at it
        if 270>position>0 then offset WHATEVER speed it is turning at by -driveOmega to counterrotate
        */
        double omegaSetpoint;
        if(270>turretDegrees() && turretDegrees()>0){
            omegaSetpoint = -driveOmega;
        }
        else{
            omegaSetpoint = 0;
        }

        if(!chameleon.validTarget()){//no target
            //face north
            omegaSetpoint += positionControl.calculate(turretDegrees(), limitAngle(35+yawWrap()));
        }
        else{//target good
            omegaSetpoint += positionControl.calculate(chameleon.getGoalAngle(), 0);
        }

        boolean safe = turretDegrees()<270 && turretDegrees()>0;
        if(safe){
            rotateTurret(omegaSetpoint);
        }
        else{
            motor.set(0); //this shouldn't happen but if it does, stop the motor
        }

        //setF(1);
        SmartDashboard.putNumber("Shooter Omega", turretOmega);
        SmartDashboard.putNumber("Turret Degrees", turretDegrees());
        SmartDashboard.putNumber("Turret Speed", encoder.getVelocity());
        SmartDashboard.putNumber("Turret FF", controller.getFF());
    }

    public void init(){
        fMultiplier = 0;
        //control = new PIDController(0, 0, 0);
        //control.setPID(RobotNumbers.turretP, RobotNumbers.turretI, RobotNumbers.turretD);
        //                                                        v   ANGERY   v
        //double motorEncoderCounts = 1; NOTE TO FUTURE PEOPLE: ENCODER SPITS OUT ROTATIONS(not counts)BY DEFAULT
        double versaRatio = RobotNumbers.turretGearRatio;
        double turretSprocketSize = RobotNumbers.turretSprocketSize;
        double motorSprocketSize = RobotNumbers.motorSprocketSize;
        double degreesPerRotation = 360; 
        //set the motor encoder to return the position of the turret in degrees using the power of MATH
        encoder.setPositionConversionFactor(((turretSprocketSize/motorSprocketSize)*versaRatio*degreesPerRotation));
        controller = motor.getPIDController();
        positionControl = new PIDController(0, 0, 0);
        encoder.setPosition(270);
        //controller.setReference(0, ControlType.kPosition);
        setMotorPID(0.01, 0, 0);
        setPosPID(0.001, 0, 0);
        motor.setIdleMode(IdleMode.kBrake);
    }

    public void resetEncoderAndGyro(){
        encoder.setPosition(0);
        resetPigeon();
    }

    /**
     * don't use
     * @param degrees
     */
    private void setTurretTarget(double degrees){
        rotateTurret(positionControl.calculate(turretDegrees(), limitAngle(degrees)));
    }

    private double turretDegrees(){
        return encoder.getPosition();
    }

    public void setDriveOmega(double omega){
        driveOmega = omega;
    }

    private void setMotorPID(double P, double I, double D){
        controller.setP(P);
        controller.setI(I);
        controller.setD(D);
    }
    private void setPosPID(double P, double I, double D){
        positionControl.setP(P);
        positionControl.setI(I);
        positionControl.setD(D);
    }

    private double limitAngle(double angle){
        if(angle>RobotNumbers.turretMaxPos){
            angle = RobotNumbers.turretMaxPos;
        }
        if(angle<RobotNumbers.turretMinPos){
            angle = RobotNumbers.turretMinPos;
        }
        return angle;
    }

    // private void setF(double F){
    //     controller.setFF(F*fMultiplier);
    // }

    /**
     * Rotate the turret at a certain rad/sec
     * @param speed - rad/sec to rotate the turret at
     */
    private void rotateTurret(double speed){
        //1 Radians Per Second to Revolutions Per Minute = 9.5493 RPM
        double turretRPM = speed*9.5493;
        double motorRPM = turretRPM * (RobotNumbers.turretSprocketSize / RobotNumbers.motorSprocketSize) * RobotNumbers.turretGearRatio;
        controller.setReference(motorRPM, ControlType.kVelocity);
    }

    private void pointNorth(){
        //set position of turret to whatever angle is "north"(generally towards goal)
    }


    //pigeon ------------------------------------------------------------------------------------------------------------------------
    public void updatePigeon(){
        pigeon.getYawPitchRoll(ypr);
    }
    public void resetPigeon(){
        updatePigeon();
        startypr = ypr;
        startYaw = yawAbs();
    }
    //absolute ypr -----------------------------------------------------------------------------------------------------------------
    public double yawAbs(){ //return absolute yaw of pigeon
        updatePigeon();
        return ypr[0];
    }
    //relative ypr ----------------------------------------------------------------------------------------------------------------
    public double yawRel(){ //return relative(to start) yaw of pigeon
        updatePigeon();
        return (ypr[0]-startYaw);
    }
    public double yawWrap(){
        double yaw = yawRel();
        while(yaw>360){
            yaw -= 360;
        }
        while(yaw<0){
            yaw += 360;
        }
        return yaw;
    }
}