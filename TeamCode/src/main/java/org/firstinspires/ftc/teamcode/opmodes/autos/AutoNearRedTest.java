package org.firstinspires.ftc.teamcode.opmodes.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.FollowerEx;
import org.firstinspires.ftc.teamcode.utils.PathChainList;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Red Near Auto Test", group = "Competition")
public class AutoNearRedTest extends AutoOpModeEx {
    private FollowerEx follower;
    private AutoCommand autoCommand;
    private List<Command> actions;
    private Shooter shooter;
    private Intake intake;
    private Boolean actionRunning;


    private PathChainList pathChainList;

    private final Pose startPose = new Pose(123.850, 21.490, Math.toRadians(-46));

    private final Pose scorePose = new Pose(105.644, 32.950, Math.toRadians(-46));

    private final Pose scoreMidPose = new Pose(76.644, 50.950, Math.toRadians(-60));

    private final Pose prepare1Pose = new Pose(78.949, 40.147, Math.toRadians(-90));
    private final Pose intake1Pose1 = new Pose(78.949, 28.239, Math.toRadians(-90));
    private final Pose intake1Pose2 = new Pose(78.949, 20.239, Math.toRadians(-90));
    private final Pose intake1Pose3 = new Pose(78.949, 11.239, Math.toRadians(-90));
    private final Pose prepareGatePose = new Pose(78.949, 20.639, Math.toRadians(-90));
    private final Pose openGatePose = new Pose(69, 8.239, Math.toRadians(-90));

    private final Pose openGateIntakePose = new Pose(65,4,Math.toRadians(-70));

    private final Pose prepare2Pose = new Pose(53.050, 35.834, Math.toRadians(-88));
    private final Pose intake2Pose1 = new Pose(53.764, 29.378, Math.toRadians(-88));
    private final Pose intake2Pose2 = new Pose(53.764, 20.378, Math.toRadians(-88));
    private final Pose intake2Pose3 = new Pose(53.764, 2, Math.toRadians(-88));
    private final Pose prepare3Pose = new Pose(32.000, 36.730, Math.toRadians(-87));
    private final Pose intake3Pose1 = new Pose(32.000, 29.932, Math.toRadians(-87));
    private final Pose intake3Pose2 = new Pose(32.000, 23.932, Math.toRadians(-87));
    private final Pose intake3Pose3 = new Pose(32.000, 0, Math.toRadians(-87));
    private final Pose intakeLoad1 = new Pose(7, 29, Math.toRadians(-87));
    private final Pose intakeLoad3 = new Pose(7, 2, Math.toRadians(-87));
    private final Pose parkPose = new Pose(70, 20.000, Math.toRadians(-90));

    private final Pose openGatePose2 = new Pose(72.982, 7.239, Math.toRadians(-1));
    private int currentPathId = 0;


    @Override
    public void initialize() {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = new FollowerEx(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        this.pathChainList = new PathChainList();
        this.actions = new ArrayList<>();
        this.autoCommand = new AutoCommand(shooter, intake);
        this.actionRunning = false;

        buildPaths();
        buildActions();

        follower.setMaxPower(1);
    }

    @NonNull
    private Point getCurrentPoint(){
        return new Point(follower.getPose().getX(),follower.getPose().getY());
    }

    private double getCurrentHeading(){
        return follower.getPose().getHeading();
    }

    private void buildPaths() {
        PathChain scorePreload,
                prepare1, intake1_1, intake1_2, intake1_3, after1,
                prepare2, intake2_1, intake2_2, intake2_3, after2,
                prepare3, intake3_1, intake3_2, intake3_3, after3,
                score1, score2, score3, openGate, afterOpenGate,openGate2,intake1,intake2,intake3,
                openGateIntake,score4,intakeLoad,scoreMidLoad,prepareMid;
        PathChain park;

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        prepare1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(prepare1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prepare1Pose.getHeading())
                .build();

        intake1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prepare1Pose), new Point(intake1Pose3)))
                .setLinearHeadingInterpolation(prepare1Pose.getHeading(), intake1Pose3.getHeading())
                .build();

        intake1_1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prepare1Pose), new Point(intake1Pose1)))
                .setLinearHeadingInterpolation(prepare1Pose.getHeading(), intake1Pose1.getHeading())
                .build();
        intake1_2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake1Pose1), new Point(intake1Pose2)))
                .setLinearHeadingInterpolation(intake1Pose1.getHeading(), intake1Pose2.getHeading())
                .build();
        intake1_3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake1Pose2), new Point(intake1Pose3)))
                .setLinearHeadingInterpolation(intake1Pose2.getHeading(), intake1Pose3.getHeading())
                .build();

        after1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake1Pose3), new Point(prepareGatePose)))
                .setLinearHeadingInterpolation(intake1Pose3.getHeading(), prepareGatePose.getHeading())
                .build();

        openGate = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prepareGatePose), new Point(openGatePose)))
                .setLinearHeadingInterpolation(prepareGatePose.getHeading(), openGatePose.getHeading())
                .build();

        openGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(prepareGatePose), new Point(openGatePose2)))
                .setLinearHeadingInterpolation(prepareGatePose.getHeading(), openGatePose2.getHeading())
                .build();

        openGateIntake = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(openGateIntakePose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), openGateIntakePose.getHeading())
                .build();
        score4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(openGateIntakePose), new Point(scorePose)))
                .setLinearHeadingInterpolation(openGateIntakePose.getHeading(), scorePose.getHeading())
                .build();





//        afterOpenGate = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(openGatePose), new Point(prepare1Pose)))
//                .setLinearHeadingInterpolation(openGatePose.getHeading(), prepare1Pose.getHeading())
//                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(openGatePose), new Point(scorePose)))
                .setLinearHeadingInterpolation(openGatePose.getHeading(), scorePose.getHeading())
                .build();

        prepare2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(prepare2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prepare2Pose.getHeading())
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prepare2Pose), new Point(intake2Pose3)))
                .setLinearHeadingInterpolation(prepare2Pose.getHeading(), intake2Pose3.getHeading())
                .build();

        intake2_1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prepare2Pose), new Point(intake2Pose1)))
                .setLinearHeadingInterpolation(prepare2Pose.getHeading(), intake2Pose1.getHeading())
                .build();
        intake2_2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake2Pose1), new Point(intake2Pose2)))
                .setLinearHeadingInterpolation(intake2Pose1.getHeading(), intake2Pose2.getHeading())
                .build();
        intake2_3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake2Pose2), new Point(intake2Pose3)))
                .setLinearHeadingInterpolation(intake2Pose2.getHeading(), intake2Pose3.getHeading())
                .build();

        after2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake2Pose3), new Point(intake2Pose1)))
                .setLinearHeadingInterpolation(intake2Pose3.getHeading(), intake2Pose1.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake2Pose1), new Point(scorePose)))
                .setLinearHeadingInterpolation(intake2Pose1.getHeading(), scorePose.getHeading())
                .build();

        prepare3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(prepare3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prepare3Pose.getHeading())
                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prepare3Pose), new Point(intake3Pose3)))
                .setLinearHeadingInterpolation(prepare3Pose.getHeading(), intake3Pose3.getHeading())
                .build();

        intake3_1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prepare3Pose), new Point(intake3Pose1)))
                .setLinearHeadingInterpolation(prepare3Pose.getHeading(), intake3Pose1.getHeading())
                .build();
        intake3_2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake3Pose1), new Point(intake3Pose2)))
                .setLinearHeadingInterpolation(intake3Pose1.getHeading(), intake3Pose2.getHeading())
                .build();
        intake3_3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake3Pose2), new Point(intake3Pose3)))
                .setLinearHeadingInterpolation(intake3Pose2.getHeading(), intake3Pose3.getHeading())
                .build();

        after3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake3Pose3), new Point(intake3Pose1)))
                .setLinearHeadingInterpolation(intake3Pose3.getHeading(), intake3Pose1.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake3Pose3), new Point(scorePose)))
                .setLinearHeadingInterpolation(intake3Pose3.getHeading(), scorePose.getHeading())
                .build();

        prepareMid = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(intakeLoad1)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intakeLoad1.getHeading())
                .build();

        intakeLoad = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakeLoad1), new Point(intakeLoad3)))
                .setLinearHeadingInterpolation(intakeLoad1.getHeading(), intakeLoad3.getHeading())
                .build();

        scoreMidLoad = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakeLoad3), new Point(scoreMidPose)))
                .setLinearHeadingInterpolation(intakeLoad3.getHeading(), scoreMidPose.getHeading())
                .build();








        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(parkPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();

        pathChainList.addPath(scorePreload, null, null, null,
                prepare1, intake1,
                after1, openGate, null, score1, null,
                prepare2, intake2,
                after2, score2, null,
                prepare3, intake3,
                score3, null,
                prepareMid, intakeLoad,
                null, scoreMidLoad, null, park);
    }

    @NonNull
    private Command actionEnd(){
        return new InstantCommand(()->this.actionRunning = false);
    }

    private void buildActions(){
        Command intakeCommand, accelerateCommand, scoreCommand, openGateCommand, waitCommand,
                scoreMidCommand, accelerateMidCommand;
        scoreCommand = autoCommand.shoot().andThen(actionEnd());
        scoreMidCommand = autoCommand.shootMid().andThen(actionEnd());
        intakeCommand = autoCommand.intake().andThen(actionEnd());
        accelerateCommand = autoCommand.accelerate().andThen(actionEnd());
        accelerateMidCommand = autoCommand.accelerateMid().andThen(actionEnd());
        openGateCommand = new WaitCommand(700).andThen(actionEnd());
        waitCommand = new WaitCommand(450).andThen(actionEnd());

        actions.addAll(Arrays.asList(accelerateCommand, waitCommand, intakeCommand, scoreCommand,
                null, null,
                null, null, openGateCommand, null, scoreCommand,
                null, null,
                null, null, scoreCommand,
                null, null,
                null, scoreCommand,
                null, null,
                accelerateMidCommand, null,scoreMidCommand, new WaitCommand(99999)));
    }

    private void periodic() {
        CommandScheduler.getInstance().run();
        follower.update();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("shooterUpVelocity", shooter.shooterUp.getVelocity());
        telemetry.addData("shooterDownVelocity", shooter.shooterDown.getVelocity());
        telemetry.addData("drive error",follower.driveError);
        telemetry.addData("follower finished",follower.isFinished);
        telemetry.addData("action finished", !this.actionRunning);
        telemetry.addData("current path /id", currentPathId);
        telemetry.addData("Actions size", actions.size());
        telemetry.addData("PathChainList size", pathChainList.size());
        telemetry.update();
    }

    @Override
    public void run() {

        if(actions.size() != pathChainList.size()){
            throw new IllegalStateException(
                    "Actions count (" + actions.size() +
                            ") does not match path count (" + pathChainList.size() + ")"
            );
        }

        Iterator<PathChain> it = pathChainList.iterator();

        int pathCount = 0;

        while (it.hasNext()){
            pathCount+=1;
            if (!opModeIsActive())break;
            periodic();
            if(!follower.isBusy() && !this.actionRunning){
                PathChain path = it.next();
                if(path!=null){
                    follower.followPath(path, 1,true);
                }

                Command currentAction = actions.get(currentPathId);
                if(currentAction!=null){
                    currentAction.schedule();
                    this.actionRunning = true;
                }
                currentPathId++;
            }
        }
    }
}