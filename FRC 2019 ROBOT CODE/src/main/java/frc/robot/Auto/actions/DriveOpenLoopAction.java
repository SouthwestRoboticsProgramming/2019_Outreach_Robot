package frc.robot.Auto.actions;

import frc.lib.Util.DriveSignal;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

public class DriveOpenLoopAction implements Action {

    private double mStartTime;
    private final double mDuration, mLeft, mRight;

    public DriveOpenLoopAction(double left, double right, double duration) {
        mDuration = duration;
        mLeft = left;
        mRight = right;
    }

    @Override
    public void start() {
        Robot.driveSubsystem.setOpenLoop(new DriveSignal(mLeft, mRight));
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime > mDuration;
    }

    @Override
    public void done() {
        Robot.driveSubsystem.setOpenLoop(new DriveSignal(0.0, 0.0));
    }
}