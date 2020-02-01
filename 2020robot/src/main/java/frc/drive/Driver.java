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
import edu.wpi.first.wpilibj.geometry.Translation2d;

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
import frc.util.Logger;

import frc.vision.BallChameleon;

import java.lang.Math;

public class Driver{
    private PigeonIMU pigeon = new PigeonIMU(RobotMap.pigeon);
    private Logger logger = new Logger("drive");
    private Logger posLogger = new Logger("positions");
    //wheelbase 27"
    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.2));
    DifferentialDriveOdometry odometer;
    private BallChameleon chameleon = new BallChameleon();

    private XBoxController controller;
    private CANSparkMax leaderL;
    private CANSparkMax followerL1;
    private CANSparkMax leaderR;
    private CANSparkMax followerR1;

    private CANPIDController leftPID;
    private CANPIDController rightPID;

    private double targetHeading;

    public double[] ypr = new double[3];
    public double[] startypr = new double[3];

    public double currentOmega;

    private boolean chaseBall;
    private boolean pointBall;

    private boolean invert;

    public int autoStage = 0;
    public boolean autoComplete = false;
    private double relLeft;
    private double relRight;

    public Pose2d robotPose;
    public Translation2d robotTranslation;
    public Rotation2d robotRotation;

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
     * Meant to be run during all periodic modes except robotPeriodic().
     */
    public void updateGeneric(){
        robotPose = odometer.update(new Rotation2d(Units.degreesToRadians(yawAbs())), getMetersLeft(), getMetersRight());
        robotTranslation = robotPose.getTranslation();
        robotRotation = robotPose.getRotation();
        double[] dataElements = {robotTranslation.getX(), robotTranslation.getY(), Logger.boolToDouble(controller.getButtonDown(5))};
        logger.writeData(dataElements);
    }

    /**
     * Update the Driver object(for teleop mode).
     */
    public void updateTeleop(){
        updateGeneric();
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
            double angleOffset = -chameleon.getBallAngle();
            double driveOffset = chameleon.getBallSize();
            //System.out.println("attempting to aim");
            if(Math.abs(angleOffset)>RobotNumbers.llTolerance){
                //System.out.println("attempting to drive");
                turn += angleOffset/70; //pulled number out of nowhere, bigger value makes the limelight have a smaller effect
            }
            // if(chaseBall){
            //     drive += 70/driveOffset;
            // }
            //System.out.println("turn: "+turn);
        }
        
        //drivePID((controller.getStickLY()*(1)) + turnSpeed, (controller.getStickLY()*(1)) - turnSpeed);
        drive(drive, turn);
        //drivePure(adjustedDrive(controller.getStickLY()), adjustedRotation(controller.getStickRX()));
        
    }

    public void updateTest(){
        updateGeneric();
        //log position on left bumper(?) presses(useful for getting auton points)
        double[] dataElements = {robotTranslation.getX(), robotTranslation.getY(), Logger.boolToDouble(controller.getButtonDown(5))};
        if(controller.getButtonDown(5)){posLogger.writeData(dataElements);}

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
            double angleOffset = -chameleon.getBallAngle();
            double driveOffset = chameleon.getBallSize();
            //System.out.println("attempting to aim");
            if(Math.abs(angleOffset)>RobotNumbers.llTolerance){
                //System.out.println("attempting to drive");
                turn += angleOffset/70; //pulled number out of nowhere, bigger value makes the limelight have a smaller effect
            }
            // if(chaseBall){
            //     drive += 70/driveOffset;
            // }
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
    private double startYaw;
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
        return -(ypr[0]-startYaw);
    }
    public double pitchRel(){ //return relative pitch of pigeon
        updatePigeon();
        return ypr[1]-startypr[1];
    }
    public double rollRel(){ //return relative roll of pigeon
        updatePigeon();
        return ypr[2]-startypr[2];
    }
    public double adjustedYaw(){
        return 90-yawRel();
    }
    public double yawWraparound(){
        double yaw = adjustedYaw();
        while(!(360>=yaw || yaw<0)){
            if(yaw>=360){
                yaw -= 360;
            }
            else if(yaw<0){
                yaw += 360;
            }
        }
        return yaw;
    }

    //position conversion -------------------------------------------------------------------------------------------------------
    private double wheelCircumference(){
        return RobotNumbers.wheelDiameter*Math.PI;
    }

    //getRotations - get wheel rotations on encoder
    public double getRotationsLeft(){
        return (leaderL.getEncoder().getPosition())/6.8;
    }
    public double getRotationsRight(){
        return (leaderR.getEncoder().getPosition())/6.8;
    }

    //getRPM - get wheel RPM from encoder
    public double getRPMLeft(){
        return (leaderL.getEncoder().getVelocity())/6.8;
    }
    public double getRPMRight(){
        return (leaderR.getEncoder().getVelocity())/6.8;
    }

    //getIPS - get wheel IPS from encoder
    public double getIPSLeft(){
        return (getRPMLeft()*wheelCircumference())/60;
    }
    public double getIPSRight(){
        return (getRPMRight()*wheelCircumference())/60;
    }

    //getFPS - get wheel FPS from encoder
    public double getFPSLeft(){
        return getIPSLeft()/12;
    }
    public double getFPSRight(){
        return getIPSRight()/12;
    }

    //getInches - get wheel inches traveled
    public double getInchesLeft(){
        return (getRotationsLeft()*wheelCircumference());
    }
    public double getInchesRight(){
        return (getRotationsRight()*wheelCircumference());
    }

    //getFeet - get wheel feet traveled
    public double getFeetLeft(){
        return (getRotationsLeft()*wheelCircumference()/12);
    }
    public double getFeetRight(){
        return (getRotationsRight()*wheelCircumference()/12);
    }

    //getMeters - get wheel meters traveled
    public double getMetersLeft(){
        return Units.feetToMeters(getFeetLeft());
    }
    public double getMetersRight(){
        return Units.feetToMeters(getFeetRight());
    }

    //auto ----------------------------------------------------------------------------------------------------------------------    
    private PIDController headingPID;
    //private int arrayAutoStage;
    /**
     * set stuff up for auto
     */
    public void setupAuto(){
        headingPID = new PIDController(RobotNumbers.headingP, RobotNumbers.headingI, RobotNumbers.headingD);
        odometer = new DifferentialDriveOdometry(Rotation2d.fromDegrees(yawAbs()), new Pose2d(0, 0, new Rotation2d()));
        String[] dataFields = {"X", "Y", "Flag"};
        String[] units = {"Meters", "Meters", ""};
        logger.init(dataFields, units);
        resetPigeon();
        leaderL.getEncoder().setPosition(0);
        leaderR.getEncoder().setPosition(0);
        headingPID.enableContinuousInput(0, 360);
        //arrayAutoStage = 0;
    }

    /**
     * "Attack"(drive towards) a point on the field. Units are in meters and its scary.
     * @param targetX - x position of the waypoint in meters
     * @param targetY - y position of the waypoint in meters
     * @return Boolean representing whether the robot is within tolerance of the waypoint or not.
     */
    public boolean attackPoint(double targetX, double targetY, double speed){
        double xDiff = targetX+robotTranslation.getY();
        double yDiff = targetY-robotTranslation.getX();
        double angleTarget = Math.toDegrees(Math.atan2(yDiff, xDiff));
        //logic: use PID to drive in such a way that the robot's heading is adjusted towards the target as it moves forward
        //wait is this just pure pursuit made by an idiot?
        double rotationOffset = -headingPID.calculate(yawWraparound(), angleTarget);
        boolean xInTolerance = Math.abs(xDiff) < RobotNumbers.autoTolerance;
        boolean yInTolerance = Math.abs(yDiff) < RobotNumbers.autoTolerance;
        boolean inTolerance = yInTolerance && xInTolerance;
        if(!inTolerance){
            //drive(RobotNumbers.autoSpeed*speed, rotationOffset*RobotNumbers.autoRotationMultiplier);
        }
        else{
            //drive(0,0);
        }
        SmartDashboard.putNumber("xDiff", xDiff);
        SmartDashboard.putNumber("yDiff", yDiff);
        SmartDashboard.putNumber("angleTarget", angleTarget);
        SmartDashboard.putNumber("heading", yawWraparound());
        SmartDashboard.putNumber("abs", yawAbs());
        SmartDashboard.putNumber("rotationOffset", rotationOffset); //number being fed into drive()
        SmartDashboard.putNumber("rotationDifference", -(angleTarget-yawWraparound()));
        SmartDashboard.putBoolean("inTolerance", inTolerance);
        SmartDashboard.putNumber("left", getMetersLeft());
        SmartDashboard.putNumber("right", getMetersRight());
        return inTolerance;
    }

    /**
     * arc follower code(bad, don't use)
     */
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

    /**
     * only used for the garbage drive arc code
     */
    private void setRelativePositions(){
        relLeft = leaderL.getEncoder().getPosition();
        relRight = leaderR.getEncoder().getPosition();
    }

    /**
     * used literally nowhere, resets auto stage to 0 and completion to false but 
     * like, auto can just kinda chill at the end with my new array stuff?
     */
    public void resetAuton(){
        autoStage = 0;
        autoComplete = false;
    }
    /**
     * shouldn't be needed anymore
     */
    public void updateAuto1(){
        robotPose = odometer.update(new Rotation2d(Units.degreesToRadians(yawAbs())), getMetersLeft(), getMetersRight());
        robotTranslation = robotPose.getTranslation();
        robotRotation = robotPose.getRotation();
        double[] dataElements = {robotTranslation.getX(), robotTranslation.getY(), 0};
        logger.writeData(dataElements);
        switch(autoStage){
            case(0):
                System.out.println("Stage 0");
                if(attackPoint(1, 2, 1)){
                    setRelativePositions();
                    autoStage++;
                }
                break;
            case(1):
                System.out.println("Stage 1");
                if(attackPoint(0, 4, 1)){
                    setRelativePositions();
                    autoStage++;
                }
                break;
            case(2):
                System.out.println("Stage 2");
                if(attackPoint(0 ,5, 1)){
                    setRelativePositions();
                    autoStage++;
                }
                break;
            default:
                autoComplete = true;
                break;    
        }
    }

    // private SpeedController m_left_motor;
    // private SpeedController m_right_motor;

    // private Encoder m_left_encoder;
    // private Encoder m_right_encoder;

    // private AnalogGyro m_gyro;

    // private EncoderFollower m_left_follower;
    // private EncoderFollower m_right_follower;

    // private Notifier m_follower_notifier;

    // private static int k_ticks_per_rev = 2048;
    // private static double k_wheel_diameter = 6.0 / 12.0;
    // private static double k_max_velocity = 8;

    // private static int k_left_channel = 0;
    // private static int k_right_channel = 1;

    // private static int k_left_encoder_port_a = 0;
    // private static int k_left_encoder_port_b = 1;
    // private static int k_right_encoder_port_a = 2;
    // private static int k_right_encoder_port_b = 3;

    // private static int k_gyro_port = 0;

    // private static String k_path_name = "RunTowardsTrench";

    // /**
    //  * run during robot init
    //  */
    // public void setupPathfinderAuto(){
    //     m_left_encoder = new Encoder(k_left_encoder_port_a, k_left_encoder_port_b);
    //     m_right_encoder = new Encoder(k_right_encoder_port_a, k_right_encoder_port_b);
    // }
    // /**
    //  * run during auton init
    //  */
    // public void initPathfinderAuto(){
    //     try {
    //         Trajectory left_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".left");
    //         Trajectory right_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".right");
        
    //         m_left_follower = new EncoderFollower(left_trajectory);
    //         m_right_follower = new EncoderFollower(right_trajectory);
        
    //         m_left_follower.configureEncoder(m_left_encoder.get(), k_ticks_per_rev, k_wheel_diameter);
    //         // You must tune the PID values on the following line!
    //         m_left_follower.configurePIDVA(8.0, 0.0, 0.0, 1 / k_max_velocity, 0);
        
    //         m_right_follower.configureEncoder(m_right_encoder.get(), k_ticks_per_rev, k_wheel_diameter);
    //         // You must tune the PID values on the following line!
    //         m_right_follower.configurePIDVA(8.0, 0.0, 0.0, 1 / k_max_velocity, 0);
        
    //         m_follower_notifier = new Notifier(this::followPath);
    //         m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);
    //       } catch (IOException e) {
    //         e.printStackTrace();
    //       }
    // }

    // private void followPath() {
    //     if (m_left_follower.isFinished() || m_right_follower.isFinished()) {
    //       m_follower_notifier.stop();
    //     } else {
    //       double left_speed = m_left_follower.calculate(m_left_encoder.get());
    //       double right_speed = m_right_follower.calculate(m_right_encoder.get());
    //       double heading = yawRel();
    //       double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
    //       double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
    //       double turn =  0.8 * (-1.0/80.0) * heading_difference;
    //       leaderL.set(left_speed + turn);
    //       leaderR.set(right_speed - turn);
    //     }
    // }

    // /**
    //  * stop, and deactivate ~~robots~~ motors
    //  */
    // public void stopMotors(){
    //     m_follower_notifier.stop();
    //     leaderL.set(0);
    //     leaderR.set(0);
    // }

    /**
     * initialize special logger for logging position when the left bumper is pressed in test mode
     */
    public void initPoseLogger(){
        String[] dataFields = {"X", "Y", "Flag"};
        String[] units = {"Meters", "Meters", ""};
        posLogger.init(dataFields, units);
    }
    
    /**
     * close all loggers
     */
    public void closeLogger(){
        logger.close();
        posLogger.close();
    }
}