package frc.lib.Controller;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.lib.Controller.Buttons;

public class ControllerSet {

    private boolean enabled = false;
    private boolean lockEnabled = false;
    private List<MappedController> controllerSet = new ArrayList<MappedController>();

    public void addMappedController(MappedController... mappedController) {
        int count = 0;
        while (count <= mappedController.length - 1) {
            controllerSet.add(mappedController[count]);
            count++;
        }
    }

    public void setEnabled(boolean enabled) {
        if (!lockEnabled) {
            this.enabled = enabled;
        }
    }

    public void lockEnabled(boolean lock) {
        this.enabled = lock;
        this.lockEnabled = true;
    }

    public JoystickButton getButton(Buttons button) {
        int count = -1;
        while (true) {
            count++;
            if (count >= controllerSet.size()-1 || !enabled) {
                return new JoystickButton(new Joystick(5), 20);
            } else if (controllerSet.get(count).checkForButton(button)) {
               return controllerSet.get(count).getButton(button);
            }
            
        }
        
    }

    public double getAxis(Axis axis) {
        int count = -1;
        while (true) {
            count++;
            if (count >= controllerSet.size()-1 || !enabled) {
                return 0;
            } else if (controllerSet.get(count).checkForAxis(axis)) {
               return controllerSet.get(count).getAxis(axis);
            }   
        }
    }
}
