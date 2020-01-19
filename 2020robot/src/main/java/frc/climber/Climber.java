package frc.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.*;
import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.controllers.XBoxController;
import frc.robot.RobotMap;
import frc.robot.RobotNumbers;
import frc.robot.RobotToggles;
import frc.util.Logger;

public class Climber{
    private final VictorSPX motorA, motorB;
    private final XBoxController xbox;

    public String[] data = {"match time", "init time", "speed", "motor current"};
    public String[] units = {"seconds", "seconds", "rpm", "A"};

    public Climber(){
        motorA = new VictorSPX(RobotMap.climberA);
        motorB = new VictorSPX(RobotMap.climberB);
        xbox = new XBoxController(0);
    }

    /**
     * Update the Climber object(run every tick)
     */
    public void update(){
        double push = xbox.getRTrigger();
        double pull = xbox.getLTrigger();
        // if(xbox.getButton(6)){
        //     drive(-0.5);
        // }
        // else if(xbox.getButton(5)){
        //     drive(0.5);
        // }
        // else{
        //     drive(0);
        // }
        drive(push-pull);
    }

    /**
     * Drive both motors on a -1 to 1 scale
     * @param speed - motor speed on a -1 to 1 scale
     */
    private void drive(double speed){
        motorA.set(ControlMode.PercentOutput, speed);
        motorB.set(ControlMode.PercentOutput, -speed);
    }

    /**
     * Initialize the Climber object(run during robotInit())
     */
    public void init(){
        motorA.setNeutralMode(NeutralMode.Brake);
        motorB.setNeutralMode(NeutralMode.Brake);
    }
}