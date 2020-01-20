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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret{
    private double sprocketRatio = 1; //replace 1 with ratio between motor and turret sprocket(turret/motor)
    private double gearingRatio = 1; //replace with whatever number

    private CANSparkMax motor;
    private CANEncoder encoder;
    //private CANPIDController controller;
    private PIDController control;
    private double driveOmega;
    private double turretOmega;

    public Turret(){
        motor = new CANSparkMax(5, MotorType.kBrushless);
        encoder = motor.getEncoder();
    }

    public void update(){
        turretOmega = -driveOmega*RobotNumbers.turretRotationSpeedMultiplier;
        double motorOmega = turretOmega*sprocketRatio;    
        
        SmartDashboard.putNumber("Shooter Omega", turretOmega);
        SmartDashboard.putNumber("Turret Degrees", turretDegrees());
    }

    public void init(){
        encoder.setPosition(0);
        control = new PIDController(0, 0, 0);
        control.setPID(RobotNumbers.turretP, RobotNumbers.turretI, RobotNumbers.turretD);
        //                                                        v   ANGERY   v
        //double motorEncoderCounts = 1; NOTE TO FUTURE PEOPLE: ENCODER SPITS OUT ROTATIONS(not counts)BY DEFAULT
        double versaRatio = 1;
        double turretSprocketSize = 1;
        double motorSprocketSize = 1;
        double degreesPerRotation = 360; 
        //set the motor encoder to return the position of the turret in degrees using the power of MATH
        encoder.setPositionConversionFactor(
            ((turretSprocketSize/motorSprocketSize)*versaRatio*degreesPerRotation)
        ); 
        //cursed formatting
        //controller = motor.getPIDController();
        //controller.setReference(0, ControlType.kPosition);
    }

    public void resetEncoder(){
        encoder.setPosition(0);
    }

    private void setTurretDegrees(double degrees){
        //controller.setReference(degrees*gearingRatio, ControlType.kPosition);

    }

    private double turretDegrees(){
        return encoder.getPosition();
    }

    public void setDriveOmega(double omega){
        driveOmega = omega;
    }
}