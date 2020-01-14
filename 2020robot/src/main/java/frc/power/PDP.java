package frc.power;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import frc.util.Logger;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;
import frc.robot.RobotNumbers;
import frc.robot.RobotToggles;

//basically just used for logging
public class PDP{
    private PowerDistributionPanel pdp = new PowerDistributionPanel();
    private Timer timer = new Timer();
    private Logger logger = new Logger("power");

    public String[] data = {"match time", "init time", "Battery Voltage", "PDP temperature", "total current"};
    public String[] units = {"seconds", "seconds", "v", "C", "A"};

    /**
     * Initialize the logger for the PDP, call during autonomousInit.
     */
    public void initLogger(){
        System.out.println("attempting to initialize logger - PDP");
        logger.init(data, units);
        timer.start();
    }

    /**
     * Initialize the PDP object.
     */
    public void init(){
        //empty lol
    }

    /**
     * Update the PDP object.
     */
    public void update(){
        double[] data = {Timer.getMatchTime(), timer.get(), pdp.getVoltage(), pdp.getTemperature(), pdp.getTotalCurrent()};
        logger.writeData(data);
    }

    /**
     * Close the PDP logger, call during disabledInit.
     */
    public void closeLogger(){
        logger.close();
    }
}