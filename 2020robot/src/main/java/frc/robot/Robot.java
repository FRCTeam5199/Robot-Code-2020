/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.drive.*;
import frc.spinner.*;
import frc.shooter.*;
import frc.power.*;
import frc.climber.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  Driver driver;
  Spinner spinner;
  Shooter shooter;
  PDP pdp;
  Climber climber;
  Turret turret;
  Hopper hopper;
  Intake intake;
  BallHandler baller;
  int autoStage;
  

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    driver = new Driver();
    driver.init();

    spinner = new Spinner();
    spinner.init();

    // shooter = new Shooter();
    // shooter.init();

    pdp = new PDP();
    pdp.init();

    climber = new Climber();
    climber.init();

    turret = new Turret();
    turret.init();

    // intake = new Intake();
    // intake.init();

    // hopper = new Hopper();
    // hopper.init();

    baller = new BallHandler();
    baller.init();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    //driver.resetAuton();
    //driver.initPathfinderAuto();
    //shooter.initLogger();
    //pdp.initLogger();
    //hopper.setupSensor();
    // driver.setupAuto();
    autoStage = 0;
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    //driver.updateAuto1();
    //updateAuto(Autos.auto1);
  }

  @Override
  public void teleopInit() {
    driver.setupAuto();
    //driver.stopMotors();
    //shooter.initLogger();
    //pdp.initLogger();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    driver.updateTeleop();
    // pdp.update();
    // //shooter.setDriveOmega(driver.omega());
    // shooter.update();
    climber.update();
  }

  @Override
  public void testInit() { 
    //turret.resetEncoder();
    //hopper.setupSensor();
    // shooter.initLogger();
    // pdp.initLogger();
    driver.setupAuto();
    // driver.initPoseLogger();
    turret.resetEncoderAndGyro();
  }
  
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    //driver.updateTest();
    turret.update();
    driver.updateTeleop(); //USE
    //turret.updateSimple(); 
    //hopper.updateSimple();
    //intake.updateSimple();
    baller.update(); //USE
    

    //shooter.update();
    // pdp.update();
  }

  @Override
  public void disabledInit() {
    baller.closeLoggers();
    pdp.closeLogger();
    driver.closeLogger();
  }

  public void updateAuto(double[][] auto){
    driver.updateGeneric();
    if(auto[autoStage][3]==-1){
      if(driver.attackPoint(auto[autoStage][0], auto[autoStage][1], auto[autoStage][2])){autoStage++;}
    }
    else if(auto[autoStage][3]==-2){
      //do nothing and don't advance the auton stage, as -2 signifies the end of the auton.
    }
    else{
      if(performSpecialAction(auto[autoStage][3])){autoStage++;}
    }
  }

  public boolean performSpecialAction(double actionToPerform){
    boolean complete = false;
    int action = (int)actionToPerform; //done because im lazy
    switch(action){
      case(0):
        complete = specialAction0();
        break;
    }
    return complete;
  }

  private boolean specialAction0(){
    System.out.println("SPECIAL ACTION 0");
    return true;
  }
}
