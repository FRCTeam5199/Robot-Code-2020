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

    shooter = new Shooter();
    shooter.init();

    pdp = new PDP();
    pdp.init();

    climber = new Climber();
    climber.init();

    turret = new Turret();
    turret.init();
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
    shooter.initLogger();
    pdp.initLogger();
    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    
  }


  @Override
  public void teleopInit() {
    shooter.initLogger();
    pdp.initLogger();
  }
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    driver.update();
    pdp.update();
    //shooter.setDriveOmega(driver.omega());
    shooter.update();
    climber.update();
  }

  @Override
  public void testInit() {
    turret.resetEncoder();
    //shooter.initLogger();
    //pdp.initLogger();
  }
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    turret.update();
    //shooter.update();
    //pdp.update();
  }

  @Override
  public void disabledInit() {
    shooter.closeLogger();
    pdp.closeLogger();
  }
}
