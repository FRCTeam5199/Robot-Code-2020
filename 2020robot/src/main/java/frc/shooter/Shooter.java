package frc.shooter;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotNumbers;

import frc.controllers.XBoxController;

import java.io.IOException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter{
    private double driveOmega;
    private double omega;
    public Shooter(){

    }

    public void update(){
        omega = -driveOmega;
        SmartDashboard.putNumber("Shooter Omega",omega);
    }

    public void init(){

    }

    public void setDriveOmega(double omega){
        driveOmega = omega;
    }

}