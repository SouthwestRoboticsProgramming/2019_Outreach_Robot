package frc.lib.Controller;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.lib.Controller.Buttons;

public class ControllerSet {
    
    private List<MappedController> controllerSet = new ArrayList<MappedController>();

    public void addMappedController(MappedController... mappedController) {
        int count = 0;
        while (count <= mappedController.length) {
            controllerSet.add(mappedController[count]);
            count++;
        }
    }

    public int checkForButton(Buttons button) {
        int buttonsFound = 0;
        int count = -1;
        while (true) {
            count++;
            if (controllerSet.get(count).checkForButton(button)) {
               buttonsFound++;
            } else if (count > controllerSet.size()) {
                return buttonsFound;
            }
            
        }
    }
    
    public JoystickButton getButton(Buttons button) {
        int count = -1;
        while (true) {
            count++;
            if (controllerSet.get(count).checkForButton(button)) {
               return controllerSet.get(count).getButton(button);
            }
            
        }
    }

    public int checkForAxis(Axis axis) {
        int axisFound = 0;
        int count = -1;
        while (true) {
            count++;
            if (controllerSet.get(count).checkForAxis(axis)) {
               axisFound++;
            } else if (count > controllerSet.size()) {
                return axisFound;
            }
            
        }
    }
    
    public double getAxis(Axis axis) {
        int count = -1;
        while (true) {
            count++;
            if (controllerSet.get(count).checkForAxis(axis)) {
               return controllerSet.get(count).getAxis(axis);
            }
            
        }
    }
}
