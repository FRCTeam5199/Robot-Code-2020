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

import java.lang.Math;

public class Driver{
    private final PigeonIMU pigeon = new PigeonIMU(RobotMap.pigeon);
    //wheelbase 27"
    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.415));
    //DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(yawAbs()), new Pose2d(0, 0, new Rotation2d()));

    private final XBoxController controller;
    private CANSparkMax leaderL;
    private CANSparkMax followerL1;
    private CANSparkMax leaderR;
    private CANSparkMax followerR1;

    private CANPIDController leftPID;
    private CANPIDController rightPID;

    private double targetHeading;

    public double[] ypr = new double[3];
    private double[] startypr = new double[3];

    public double currentOmega;

    public Driver(){
        controller = new XBoxController(0);
        leaderL = new CANSparkMax(RobotMap.driveLeaderL, MotorType.kBrushless);
        leaderR = new CANSparkMax(RobotMap.driveLeaderR, MotorType.kBrushless);
        followerL1 = new CANSparkMax(RobotMap.driveFollowerL, MotorType.kBrushless);
        followerR1 = new CANSparkMax(RobotMap.driveFollowerR, MotorType.kBrushless);

        leftPID = leaderL.getPIDController();
        rightPID = leaderR.getPIDController();
        
        //headControl = new PIDController(Kp, Ki, Kd);
    }

    public void init(){
        followerL1.follow(leaderL);
        followerR1.follow(leaderR);
        leaderL.setInverted(false);
        leaderR.setInverted(true);
        //resetPigeon();
        //updatePigeon();
        setPID(RobotNumbers.drivebaseP, RobotNumbers.drivebaseI, RobotNumbers.drivebaseD);
    }

    public void update(){
        //drive(0.5,1);
        double turnSpeed = controller.getStickRX() * -0.7;
        //drivePID((controller.getStickLY()*(1)) + turnSpeed, (controller.getStickLY()*(1)) - turnSpeed);
        drive(controller.getStickLY(), controller.getStickRX());
        //drivePure(adjustedDrive(controller.getStickLY()), adjustedRotation(controller.getStickRX()));
    }

    //drive with inputs -1 to 1
    private void drive(double forward, double rotation){ 
        drivePure(adjustedDrive(forward), adjustedRotation(rotation));
    }

    public void drivePID(double left, double right){ //set target speeds for PID controlled drive
        leftPID.setReference(left*5000, ControlType.kVelocity);
        rightPID.setReference(left*5000, ControlType.kVelocity);
    }

    private void drivePure(double FPS, double omega){
        currentOmega = -omega;
        var chassisSpeeds = new ChassisSpeeds(Units.feetToMeters(FPS), 0, -omega);
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        double leftVelocity = Units.metersToFeet(wheelSpeeds.leftMetersPerSecond);
        double rightVelocity = Units.metersToFeet(wheelSpeeds.rightMetersPerSecond);
        //System.out.println("FPS: "+leftVelocity+"  "+rightVelocity+" RPM: "+convertFPStoRPM(leftVelocity)+" "+convertFPStoRPM(rightVelocity));
        leftPID.setReference(convertFPStoRPM(leftVelocity)*3.8, ControlType.kVelocity);
        rightPID.setReference(convertFPStoRPM(rightVelocity)*3.8, ControlType.kVelocity);
        System.out.println(leaderL.getEncoder().getVelocity()+" "+leaderR.getEncoder().getVelocity());
    }

    private void setPID(double P, double I, double D){
        leftPID.setP(P);
        leftPID.setI(I);
        leftPID.setD(D);
        rightPID.setP(P);
        rightPID.setI(I);
        rightPID.setD(D);

        leftPID.setOutputRange(-1, 1);
        rightPID.setOutputRange(-1, 1);
    }

    private double adjustedDrive(double input){
        return input*RobotNumbers.maxSpeed;
    }

    private double adjustedRotation(double input){
        return input*RobotNumbers.maxRotation;
    }


    private double convertFPStoRPM(double FPS){
        return FPS*(RobotNumbers.maxMotorSpeed/RobotNumbers.maxSpeed);
    }

    public double omega(){
        return currentOmega;
    }
    
    /*//pigeon code ------------------------------------------------------------------------------------------------------------------
    public void updatePigeon(){
        pigeon.getYawPitchRoll(ypr);
    }
    public void resetPigeon(){
        updatePigeon();
        startypr = ypr;
    }
    //absolute ypr -----------------------------------------------------------------------------------------------------------------
    public double yawAbs(){ //return absolute yaw of pigeon
        updatePigeon();
        return ypr[0];
    }
    public double pitchAbs(){ //return absolute pitch of pigeon
        updatePigeon();
        return ypr[1];
    }
    public double rollAbs(){ //return absolute roll of pigeon
        updatePigeon();
        return ypr[2];
    }
    //relative ypr ----------------------------------------------------------------------------------------------------------------
    public double yawRel(){ //return relative(to start) yaw of pigeon
        updatePigeon();
        return ypr[0]-startypr[0];
    }
    public double pitchRel(){ //return relative pitch of pigeon
        updatePigeon();
        return ypr[1]-startypr[1];
    }
    public double rollRel(){ //return relative roll of pigeon
        updatePigeon();
        return ypr[2]-startypr[2];
    }*/

}