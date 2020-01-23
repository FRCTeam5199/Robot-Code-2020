package frc.robot;
import com.revrobotics.CANSparkMax.FaultID;

//like RobotToggles but for numbers that I would prefer to keep all in one place(may add shuffleboard stuff later)
public class RobotNumbers{
    //public static final double [NAME] = [VALUE];
    //public static final double maxHeight = 40; <-- EXAMPLE

    public static final double maxSpeed = 10; //max speed in fps - REAL IS 10(for 4in wheels)
    public static final double maxRotation = 11.2; //max rotational speed in radians per second - REAL IS 11.2(for 4in wheels)
    public static final double llTolerance = 3;

    public static final double drivebaseP = 3e-5;
    public static final double drivebaseI = 0;
    public static final double drivebaseD = 7e-4;

    public static final double autoSpeedMultiplier = 0.5;

    public static final double wheelDiameter = 4; //wheel diameter, used literally nowhere
    public static final double maxMotorSpeed = 5000; //theoretical max motor speed

    public static final double motorPulleySize = 24;
    public static final double driverPulleySize = 18;
    public static final double driverWheelDiameter = 4;

    public static final double turretRotationSpeedMultiplier = 1; //multiplied to drive omega to calibrate the compensating rotation speed offset of the turret
    public static final double turretP = 0.00000000001;
    public static final double turretI = 0;
    public static final double turretD = 0;

    public static final double triggerSensitivity = 0.25;

    //junk for motor debug code, change names if needed
    public static final String[] sparkErrors = {"Brownout", "Overcurrent", "IWDTReset", "MotorFault", "SensorFault", "Stall", "EEPROMCRC", "CANTX", "CANRX", "HasReset", "DRVFault", "OtherFault"};
    public static final FaultID[] sparkErrorIDs = {FaultID.kBrownout, FaultID.kOvercurrent, FaultID.kIWDTReset, FaultID.kMotorFault, FaultID.kSensorFault, FaultID.kStall, FaultID.kEEPROMCRC, FaultID.kCANTX, FaultID.kCANRX, FaultID.kHasReset, FaultID.kDRVFault, FaultID.kOtherFault};
}