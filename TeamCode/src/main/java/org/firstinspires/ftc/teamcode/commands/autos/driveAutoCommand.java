package org.firstinspires.ftc.teamcode.commands.autos;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.commands.FollowPath;

public class driveAutoCommand extends CommandBase {
    private Follower follower;
    private PathChain pathChain;

    public driveAutoCommand(Follower follower, PathChain pathChain) {
        this.follower = follower;
        this.pathChain = pathChain;
    }

    @Override
    public void initialize() {
        follower.followPath(pathChain);
    }

    @Override
    public void execute() {
        follower.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (follower.isBusy()) {
            follower.breakFollowing();
        }
    }

    public boolean isFinished() {
        return !follower.isBusy();
    }
}