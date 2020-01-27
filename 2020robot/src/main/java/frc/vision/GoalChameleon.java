package frc.vision;

import edu.wpi.first.networktables.*;

public class GoalChameleon{
    public NetworkTableEntry yaw;
    public NetworkTableEntry size;
    public NetworkTableEntry isValid;
    public NetworkTableEntry pitch;

    public void init(){
        NetworkTableInstance table = NetworkTableInstance.getDefault();
        NetworkTable cameraTable = table.getTable("chameleon-vision").getSubTable("GoalChameleon");
        yaw = cameraTable.getEntry("targetYaw");
        size = cameraTable.getEntry("targetFittedWidth");
        isValid = cameraTable.getEntry("isValid");
        pitch = cameraTable.getEntry("targetPitch");
    }

    public void update(){
        
    }

    /**
     * Get angle between crosshair and goal left/right.
     * @return angle between crosshair and ball, left negative, 29.8 degrees in both directions.
     */
    public double getGoalAngle(){ 
        double angle = yaw.getDouble(0);
        if(isValid.getBoolean(false)){
            return angle;
        }
        return 0;
    }

    /**
     * Get angle between crosshair and goal up/down.
     * @return angle between crosshair and ball, down negative, 22 degrees in both directions.
     */
    public double getGoalPitch(){ 
        double angle = pitch.getDouble(0);
        if(isValid.getBoolean(false)){
            return angle;
        }
        return 0;
    }

    /**
     * Get the size of the goal onscreen.
     * @return size of the ball in % of the screen, 0-100.
     */
    public double getGoalSize(){
        double goalSize = size.getDouble(0);
        if(isValid.getBoolean(false)){
            return goalSize;
        }
        return 0;
    }
}