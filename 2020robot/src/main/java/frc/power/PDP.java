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

    public void initLogger(){
        System.out.println("attempting to initialize logger - PDP");
        logger.init(data, units);
        timer.start();
    }

    public void init(){
        //empty lol
    }

    public void update(){
        double[] data = {Timer.getMatchTime(), timer.get(), pdp.getVoltage(), pdp.getTemperature(), pdp.getTotalCurrent()};
        logger.writeData(data);
    }

    public void closeLogger(){
        logger.close();
    }
}