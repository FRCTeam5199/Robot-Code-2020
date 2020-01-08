package frc.drive;

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

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class Driver{
    //wheelbase 27"
    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.415));
    DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(getGyroHeading(), new Pose2d(5.0, 13.5, new Rotation2d()));

    private final XBoxController controller;
    private CANSparkMax leaderL;
    private CANSparkMax followerL1;
    private CANSparkMax leaderR;
    private CANSparkMax followerR1;

    private PIDController headControl;

    private double targetHeading;

    public Driver(){
        controller = new XBoxController(0);
        leaderL = new CANSparkMax(RobotMap.driveLeaderL, MotorType.kBrushless);
        leaderR = new CANSparkMax(RobotMap.driveLeaderR, MotorType.kBrushless);
        followerL1 = new CANSparkMax(RobotMap.driveFollowerL, MotorType.kBrushless);
        followerR1 = new CANSparkMax(RobotMap.driveFollowerR, MotorType.kBrushless);
        
        //headControl = new PIDController(Kp, Ki, Kd);
    }

    public void init(){
        followerL1.follow(leaderL);
        followerR1.follow(leaderR);
        leaderL.setInverted(true);
    }

    public void update(){

    }

    
}