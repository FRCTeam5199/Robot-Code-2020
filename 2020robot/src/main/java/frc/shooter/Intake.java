package frc.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.*;
import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.controllers.*;
import frc.robot.RobotMap;
import frc.robot.RobotNumbers;
import frc.robot.RobotToggles;
import frc.util.Logger;
//import frc.util.Permalogger;
import frc.vision.GoalChameleon;

public class Intake{ 
    private VictorSPX victor;
    private ButtonPanel panel;
    private ShuffleboardTab tab = Shuffleboard.getTab("balls");
    private NetworkTableEntry speedEntry = tab.add("Intake Speed", 0).getEntry();

    public void init(){
        victor = new VictorSPX(RobotMap.intakeMotor);
        panel = new ButtonPanel(3);
    }

    public void updateSimple(){
        if(panel.getButton(1)){
            victor.set(ControlMode.PercentOutput, speedEntry.getDouble(0.6));
        }
        else if(panel.getButton(2)){
            victor.set(ControlMode.PercentOutput, -speedEntry.getDouble(0.6));
        }
        else{
            victor.set(ControlMode.PercentOutput, 0);
        }
    }
}