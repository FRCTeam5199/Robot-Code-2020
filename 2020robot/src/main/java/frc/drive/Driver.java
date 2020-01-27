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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import java.io.IOException;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

import frc.vision.BallChameleon;

import java.lang.Math;

public class Driver{
    private final PigeonIMU pigeon = new PigeonIMU(RobotMap.pigeon);
    //wheelbase 27"
    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.415));
    //DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(yawAbs()), new Pose2d(0, 0, new Rotation2d()));
    private BallChameleon chameleon = new BallChameleon();

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

    private boolean chaseBall;
    private boolean pointBall;

    private boolean invert;

    public int autoStage = 0;
    public boolean autoComplete = false;
    private double relLeft;
    private double relRight;

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

    /**
     * Initialize the Driver object.
     */
    public void init(){
        chameleon.init();
        followerL1.follow(leaderL);
        followerR1.follow(leaderR);
        leaderL.setInverted(true);
        leaderR.setInverted(false);
        resetPigeon();
        updatePigeon();
        setPID(RobotNumbers.drivebaseP, RobotNumbers.drivebaseI, RobotNumbers.drivebaseD);
        autoStage = 0;
        autoComplete = false;
        //setupPathfinderAuto();
    }

    /**
     * Update the Driver object.
     */
    public void update(){
        invert = false;//controller.getButton(6);
        SmartDashboard.putBoolean("invert", invert);
        //drive(0.5,1);
        double turn = -controller.getStickRX();
        double drive;
        if(invert){
            drive = -controller.getStickLY();
        }
        else{
            drive = controller.getStickLY();
        }
        pointBall = controller.getButton(6); //DISABLED BECAUSE WE YOINKED THE LL
        chaseBall = false;//controller.getRTriggerPressed(); //ALSO DISABLED BECAUSE WE YOINKED THE LL

        //!!!!!
        //if statement for ball tracking should add an omega offset proportional to the ball's left/rightness in the limelight
        if(pointBall){
            double omegaOffset = -chameleon.getBallAngle();
            double driveOffset = chameleon.getBallSize();
            //System.out.println("attempting to aim");
            if(Math.abs(omegaOffset)>RobotNumbers.llTolerance){
                //System.out.println("attempting to drive");
                turn += omegaOffset/70; //pulled number out of nowhere, bigger value makes the limelight have a smaller effect
            }
            if(chaseBall){
                drive += 70/driveOffset;
            }
            //System.out.println("turn: "+turn);
        }
        
        //drivePID((controller.getStickLY()*(1)) + turnSpeed, (controller.getStickLY()*(1)) - turnSpeed);
        drive(drive, turn);
        //drivePure(adjustedDrive(controller.getStickLY()), adjustedRotation(controller.getStickRX()));
        
    }

    

    /**
     * Drive each side based on inputs -1 to 1.
     */
    private void drive(double forward, double rotation){ 
        drivePure(adjustedDrive(forward), adjustedRotation(rotation));
    }

    /**
     * Drive each side based on a -1 to 1 scale but with PID
     */
    public void drivePID(double left, double right){ 
        leftPID.setReference(left*RobotNumbers.maxMotorSpeed, ControlType.kVelocity);
        rightPID.setReference(right*RobotNumbers.maxMotorSpeed, ControlType.kVelocity);
    }

    /**
     * Drive based on FPS and omega(speed of rotation in rad/sec)
     */
    private void drivePure(double FPS, double omega){
        currentOmega = omega;
        var chassisSpeeds = new ChassisSpeeds(Units.feetToMeters(FPS), 0, -omega);
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        double leftVelocity = Units.metersToFeet(wheelSpeeds.leftMetersPerSecond);
        double rightVelocity = Units.metersToFeet(wheelSpeeds.rightMetersPerSecond);
        //System.out.println("FPS: "+leftVelocity+"  "+rightVelocity+" RPM: "+convertFPStoRPM(leftVelocity)+" "+convertFPStoRPM(rightVelocity));
        leftPID.setReference(convertFPStoRPM(leftVelocity)*3.8, ControlType.kVelocity);
        rightPID.setReference(convertFPStoRPM(rightVelocity)*3.8, ControlType.kVelocity);
        //System.out.println(leaderL.getEncoder().getVelocity()+" "+leaderR.getEncoder().getVelocity());
    }

    /**
     * Set P, I, and D values for the drivetrain.
     */
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

    /**
     * Get the current Omega value.
     * @return speed of rotation in rad/sec
     */
    public double omega(){
        return currentOmega;
    }
    
    //pigeon code ------------------------------------------------------------------------------------------------------------------
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
    }

    //auto ----------------------------------------------------------------------------------------------------------------------
    public boolean driveSidesToPos(double leftFeet, double rightFeet){
        double leftSpeed, rightSpeed;
        double reverser = RobotNumbers.autoSpeedMultiplier;
        double leftPos = (leaderL.getEncoder().getPosition()-relLeft)/6.8*(RobotNumbers.wheelDiameter*Math.PI)/12; //motor rots > feet: encoder/(geardown)*(diameter*pi)/12
        double rightPos = (leaderR.getEncoder().getPosition()-relRight)/6.8*(RobotNumbers.wheelDiameter*Math.PI)/12;
        SmartDashboard.putNumber("leftPos", leftPos);
        SmartDashboard.putNumber("rightPos", rightPos);
        if(leftFeet<0 || rightFeet<0){
            reverser = -1*RobotNumbers.autoSpeedMultiplier;
        }

        if(leftFeet>rightFeet){
            leftSpeed = reverser;
            rightSpeed = rightFeet/leftFeet*reverser;
        }
        else if(rightFeet>leftFeet){
            rightSpeed = reverser;
            leftSpeed = leftFeet/rightFeet*reverser;
        }
        else{ //distances are equal
            rightSpeed = reverser;
            leftSpeed = reverser;
        }
        SmartDashboard.putNumber("lspeed", rightSpeed);
        SmartDashboard.putNumber("rspeed", leftSpeed);
        if(reverser>0){ //if not driving in reverse
            if(leftPos<leftFeet||rightPos<rightFeet){
                drivePID(leftSpeed, rightSpeed);
            }
            else{
                drivePID(0, 0);
                return true;
            }
        }
        else if(reverser<0){ //if driving in reverse
            if(leftPos>leftFeet||rightPos>rightFeet){
                drivePID(leftSpeed, rightSpeed);
            }
            else{
                drivePID(0, 0);
                return true;
            }
        }
        return false;
    }

    private void setRelativePositions(){
        relLeft = leaderL.getEncoder().getPosition();
        relRight = leaderR.getEncoder().getPosition();
    }

    public void resetAuton(){
        autoStage = 0;
        autoComplete = false;
    }

    public void updateAuto1(){
        switch(autoStage){
            case(0):
                System.out.println("Stage 0");
                if(driveSidesToPos(4, 8)){
                    setRelativePositions();
                    autoStage++;
                }
                break;
            case(1):
                System.out.println("Stage 1");
                if(driveSidesToPos(8, 4)){
                    setRelativePositions();
                    autoStage++;
                }
                break;
            case(2):
                System.out.println("Stage 2");
                if(driveSidesToPos(4, 10)){
                    setRelativePositions();
                    autoStage++;
                }
                break;
            default:
                autoComplete = true;
                break;    
        }
    }


    private SpeedController m_left_motor;
    private SpeedController m_right_motor;

    private Encoder m_left_encoder;
    private Encoder m_right_encoder;

    private AnalogGyro m_gyro;

    private EncoderFollower m_left_follower;
    private EncoderFollower m_right_follower;

    private Notifier m_follower_notifier;

    private static final int k_ticks_per_rev = 2048;
    private static final double k_wheel_diameter = 6.0 / 12.0;
    private static final double k_max_velocity = 8;

    private static final int k_left_channel = 0;
    private static final int k_right_channel = 1;

    private static final int k_left_encoder_port_a = 0;
    private static final int k_left_encoder_port_b = 1;
    private static final int k_right_encoder_port_a = 2;
    private static final int k_right_encoder_port_b = 3;

    private static final int k_gyro_port = 0;

    private static final String k_path_name = "RunTowardsTrench";

    /**
     * run during robot init
     */
    public void setupPathfinderAuto(){
        m_left_encoder = new Encoder(k_left_encoder_port_a, k_left_encoder_port_b);
        m_right_encoder = new Encoder(k_right_encoder_port_a, k_right_encoder_port_b);
    }
    /**
     * run during auton init
     */
    public void initPathfinderAuto(){
        try {
            Trajectory left_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".left");
            Trajectory right_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".right");
        
            m_left_follower = new EncoderFollower(left_trajectory);
            m_right_follower = new EncoderFollower(right_trajectory);
        
            m_left_follower.configureEncoder(m_left_encoder.get(), k_ticks_per_rev, k_wheel_diameter);
            // You must tune the PID values on the following line!
            m_left_follower.configurePIDVA(8.0, 0.0, 0.0, 1 / k_max_velocity, 0);
        
            m_right_follower.configureEncoder(m_right_encoder.get(), k_ticks_per_rev, k_wheel_diameter);
            // You must tune the PID values on the following line!
            m_right_follower.configurePIDVA(8.0, 0.0, 0.0, 1 / k_max_velocity, 0);
        
            m_follower_notifier = new Notifier(this::followPath);
            m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);
          } catch (IOException e) {
            e.printStackTrace();
          }
    }

    private void followPath() {
        if (m_left_follower.isFinished() || m_right_follower.isFinished()) {
          m_follower_notifier.stop();
        } else {
          double left_speed = m_left_follower.calculate(m_left_encoder.get());
          double right_speed = m_right_follower.calculate(m_right_encoder.get());
          double heading = yawRel();
          double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
          double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
          double turn =  0.8 * (-1.0/80.0) * heading_difference;
          leaderL.set(left_speed + turn);
          leaderR.set(right_speed - turn);
        }
    }

    /**
     * stop, and deactivate ~~robots~~ motors
     */
    public void stopMotors(){
        m_follower_notifier.stop();
        leaderL.set(0);
        leaderR.set(0);
    }
}