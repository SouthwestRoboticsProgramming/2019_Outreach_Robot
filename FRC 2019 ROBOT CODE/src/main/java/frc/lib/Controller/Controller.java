package frc.lib.Controller;

import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class Controller {

    private ControllerSet controllerSet;

    public void setDefaultControllerSet(ControllerSet controllerSet) {
        try {
            this.controllerSet.setEnabled(false);
        } catch (Exception e) {}
        this.controllerSet = controllerSet;
        this.controllerSet.setEnabled(true);
    }

    public ControllerSet getDefaultControllerSet() {
        return controllerSet;
    }

    public JoystickButton getButton(Buttons Button) {
        return controllerSet.getButton(Button);
    }

    public double getAxis(Axis Axis) {
        return controllerSet.getAxis(Axis);
    }

}
