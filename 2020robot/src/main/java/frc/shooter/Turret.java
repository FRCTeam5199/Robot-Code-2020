package frc.shooter;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotNumbers;

import frc.controllers.XBoxController;

import java.io.IOException;

import com.revrobotics.CANSparkMax;
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
    //private PIDController control;
    private double driveOmega;
    private double turretOmega;

    private double fMultiplier;
    private double targetPosition;
    private ShuffleboardTab tab = Shuffleboard.getTab("Turret");
    private NetworkTableEntry fMult = tab.add("F Multiplier", 0).getEntry();
    private NetworkTableEntry pos = tab.add("Position", 0).getEntry();
    private NetworkTableEntry p = tab.add("P", 0).getEntry();
    private NetworkTableEntry i = tab.add("I", 0).getEntry();
    private NetworkTableEntry d = tab.add("D", 0).getEntry();

    public Turret(){
        motor = new CANSparkMax(5, MotorType.kBrushless);
        encoder = motor.getEncoder();
    }

    public void update(){
        fMultiplier = fMult.getDouble(0);
        targetPosition = pos.getDouble(0);
        setMotorPID(p.getDouble(0), i.getDouble(0), d.getDouble(0));
        //turretOmega = -driveOmega*RobotNumbers.turretRotationSpeedMultiplier;
        //double motorOmega = turretOmega*sprocketRatio;    
        
        setTurretTarget(targetPosition);
        //setF(1);
        SmartDashboard.putNumber("Shooter Omega", turretOmega);
        SmartDashboard.putNumber("Turret Degrees", turretDegrees());
        SmartDashboard.putNumber("Turret Speed", encoder.getVelocity());
        SmartDashboard.putNumber("Turret FF", controller.getFF());
    }

    public void init(){
        fMultiplier = 0;
        encoder.setPosition(0);
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
        //controller.setReference(0, ControlType.kPosition);
        setMotorPID(0.01, 0, 0);
        setPosPID(0.001, 0, 0);
    }

    public void resetEncoder(){
        encoder.setPosition(0);
    }

    private void setTurretTarget(double degrees){
        rotateTurret(positionControl.calculate(turretDegrees(), degrees)+driveOmega);
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

    private void setF(double F){
        controller.setFF(F*fMultiplier);
    }

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
}