package frc.robot;

import frc.lib.Controller.Axis;
import frc.lib.Controller.Buttons;
import frc.lib.Controller.Controller;
import frc.lib.Controller.ControllerSet;
import frc.lib.Controller.MappedController;
import frc.robot.commands.DeployRamps;
import frc.robot.commands.PresentPositions;
import frc.robot.commands.SetCameraAngle;
import frc.robot.commands.SetVacuum;
import frc.robot.commands.ToggleLiftRobot;
import frc.robot.commands.ToggleNoArm;
import frc.robot.commands.ToggleOutreachMode;

public class OI {
  public boolean isOutreachMode = false;
  public boolean isNoArm = false;

  public Controller controller = new Controller();
    public ControllerSet threeJoy = new ControllerSet();
      private MappedController manipulator = new MappedController(0);
      private MappedController left = new MappedController(1);
      private MappedController right = new MappedController(2);
    public ControllerSet xbox = new ControllerSet();
      private MappedController xBox = new MappedController(0);
  public OI() {
    threeJoy.addMappedController(manipulator, left, right);
      manipulator.mapButton(Buttons.armBypass, 6)
        .mapButton(Buttons.extendArm, 5)
        .mapButton(Buttons.extentionWristBypass, 4)
        .mapButton(Buttons.preset1, 7)
        .mapButton(Buttons.preset2, 8)
        .mapButton(Buttons.preset3, 9)
        .mapButton(Buttons.preset4, 10)
        .mapButton(Buttons.preset5, 11)
        .mapButton(Buttons.preset6, 12)
        .mapButton(Buttons.retractArm, 3)
        .mapButton(Buttons.vacuumOff, 2)
        .mapButton(Buttons.vacuumOn, 1)
        .mapAxis(Axis.armRotate, 0)
        .mapAxis(Axis.wristRotate, 0);
      left.mapButton(Buttons.deployRamps, 2)
        .mapButton(Buttons.liftRobot, 4)
        .mapButton(Buttons.limelightFollow, 1)
        .mapAxis(Axis.leftDrive, 0);
      right.mapButton(Buttons.cameraFaceFront, 3)
        .mapButton(Buttons.cameraFaceBack, 2)
        .mapButton(Buttons.noArm, 0)
        .mapButton(Buttons.outreachMode, 8)
        .mapButton(Buttons.rightControl, 1)
        .mapAxis(Axis.rightDrive, 0);
    xbox.addMappedController(xBox);
      xBox.mapAxis(Axis.centerDrive, 0)
      .mapAxis(Axis.turnDrive, 0)
      .mapAxis(Axis.armRotate, 0)
      .mapAxis(Axis.wristRotate, 0)
      .mapButton(Buttons.extendArm, 0)
      .mapButton(Buttons.retractArm, 0)
      .mapButton(Buttons.quickTurn, 0)
      .mapButton(Buttons.vacuumOn, 0)
      .mapButton(Buttons.vacuumOff, 0);

    controller.getButton(Buttons.preset1).whenPressed(new PresentPositions(1));
    controller.getButton(Buttons.preset2).whenPressed(new PresentPositions(2));
    controller.getButton(Buttons.preset3).whenPressed(new PresentPositions(3));
    controller.getButton(Buttons.preset4).whenPressed(new PresentPositions(4));
    controller.getButton(Buttons.preset5).whenPressed(new PresentPositions(5));
    controller.getButton(Buttons.preset6).whenPressed(new PresentPositions(6));
    controller.getButton(Buttons.preset7).whenPressed(new PresentPositions(7));

    // LIFT / RAMPS
    controller.getButton(Buttons.liftRobot).whenPressed(new ToggleLiftRobot());
    controller.getButton(Buttons.deployRamps).whenPressed(new DeployRamps());

    // OUTREACH MODE / NO ARM
    controller.getButton(Buttons.outreachMode).whenPressed(new ToggleOutreachMode());
    controller.getButton(Buttons.noArm).whenPressed(new ToggleNoArm());

    // CAMERA SERVO
    controller.getButton(Buttons.cameraFaceFront).whenPressed(new SetCameraAngle((int)Robot.ShuffleBoard.cameraFrontAngle.getDouble(RobotMap.cameraFaceFrontAngle)));
    controller.getButton(Buttons.cameraFaceBack).whenPressed(new SetCameraAngle((int)Robot.ShuffleBoard.cameraBackAngle.getDouble(RobotMap.cameraFaceBackAngle)));

    // VACUUM
    controller.getButton(Buttons.vacuumOn).whenPressed(new SetVacuum(Robot.ShuffleBoard.vacuumPumpSpeed.getDouble(RobotMap.defaultVacuumPumpSpeed)));
    controller.getButton(Buttons.vacuumOff).whenPressed(new SetVacuum(0));
  }

  // TWO DRIVE
  public double leftDrive() {
    return controller.getAxis(Axis.leftDrive);
  }

  public double rightDrive() {
    return controller.getAxis(Axis.rightDrive);
  }

  public boolean LimelightControl() {
    return controller.getButton(Buttons.limelightFollow).get();
  }

  public boolean rightControl() {
    return controller.getButton(Buttons.rightControl).get();
  }

  // ONE DRIVE
  public double oneDrive() {
    return controller.getAxis(Axis.centerDrive);
  }

  public double oneTurn() {
    return controller.getAxis(Axis.turnDrive);
  }

  public boolean oneQuickTurn() {
    return controller.getButton(Buttons.quickTurn).get();
  }

  // ARM
  public double armControl() {
    return controller.getAxis(Axis.armRotate);
  }

  public boolean armLimitBypass() {
    return controller.getButton(Buttons.armBypass).get();
  }

  // EXTENTION
  public double extentionControl() {
    double output = 0;
    if (controller.getButton(Buttons.extendArm).get()) {output += 1;}
    if (controller.getButton(Buttons.retractArm).get()) {output -= 1;}
    return output;
  }

  public boolean wristExtentionlimitBypass() {
    return controller.getButton(Buttons.extentionWristBypass).get();
  }

  // WRIST
  public double wristControl() {
    return controller.getAxis(Axis.wristRotate);
  }
}