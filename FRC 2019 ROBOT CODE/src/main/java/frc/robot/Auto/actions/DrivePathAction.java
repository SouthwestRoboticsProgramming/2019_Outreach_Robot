package frc.robot.Auto.actions;

import frc.robot.Robot;
import frc.robot.paths.PathContainer;
import frc.lib.Path.Path;
import frc.lib.Util.DriveSignal;

/**
 * Drives the robot along the Path defined in the PathContainer object. The action finishes once the robot reaches the
 * end of the path.
 *
 * @see PathContainer
 * @see Path
 * @see Action
 */
public class DrivePathAction implements Action {

    private PathContainer mPathContainer;
    private Path mPath;
    private boolean mStopWhenDone;

    public DrivePathAction(PathContainer p, boolean stopWhenDone) {
        mPathContainer = p;
        mPath = mPathContainer.buildPath();
        mStopWhenDone = stopWhenDone;
    }

    public DrivePathAction(PathContainer p) {
        this(p, false);
    }

    @Override
    public void start() {
        Robot.driveSubsystem.setWantDrivePath(mPath, mPathContainer.isReversed());
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return Robot.driveSubsystem.isDoneWithPath();
    }

    @Override
    public void done() {
        if (mStopWhenDone) {
            Robot.driveSubsystem.setVelocity(new DriveSignal(0, 0), new DriveSignal(0, 0));
        }
    }
}
