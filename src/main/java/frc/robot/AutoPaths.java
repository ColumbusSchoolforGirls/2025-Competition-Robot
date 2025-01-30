package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AutoPaths { //obviously need to add more

  //the drop down menu to choose a path on the dashboard
  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  AutoStep[] autoActions = {};
  HashMap<String,AutoStep[]> autoPaths = new HashMap<>();

    public void setAutoPaths() {
        // Add the auto paths to the shuffleboard
        autoPaths.put("Left Main", AutoPaths.autoLeftMain);
        autoPaths.put("Right Main", AutoPaths.autoRightMain);
        autoPaths.put("Middle Main", AutoPaths.autoMiddleMain);

        for(String autoPathName: autoPaths.keySet()){
            autoChooser.addOption(autoPathName, autoPathName);
        }

        SmartDashboard.putData("Auto choices", autoChooser); //actually puts them on the dashboard after they are added to m_chooser
    }

    public static AutoStep[] autoLeftMain = {


    };  
    
    public static AutoStep[] autoMiddleMain = {


    };  

    public static AutoStep[] autoRightMain = {


    };  
}
