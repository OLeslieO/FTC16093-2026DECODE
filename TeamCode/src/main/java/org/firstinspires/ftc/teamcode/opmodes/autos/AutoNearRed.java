//package org.firstinspires.ftc.teamcode.opmodes.autos;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.command.Command;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierCurve;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.Subsystems.Intake;
//import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
//import org.firstinspires.ftc.teamcode.utils.FollowerEx;
//import org.firstinspires.ftc.teamcode.utils.PathChainList;
//
//import java.util.ArrayList;
//import java.util.Arrays;
//import java.util.Iterator;
//import java.util.List;
//
//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;
//
//@Autonomous(name = "Auto Near Red", group = "Competition")
//public class AutoNearRed extends AutoOpModeEx {
//    private FollowerEx follower;
//    private AutoCommand autoCommand;
//    private List<Command> actions;
//    private Shooter shooter;
//    private Intake intake;
//    private Boolean actionRunning;
//
//
//
//    private PathChainList pathChainList;
//
//    private final Pose startPose = new Pose(123.850, -21.490, Math.toRadians(45));
//
//    private final Pose scorePose = new Pose(93.644, 0, Math.toRadians(45));
//
//    private final Pose prepare1Pose = new Pose(79.249, -40.147, Math.toRadians(90));
//    private final Pose intake1Pose = new Pose(79.249, -14.239, Math.toRadians(90));
//    private final Pose openGatePose = new Pose(73.482, -12.239, Math.toRadians(90));
//    private final Pose prepare2Pose = new Pose(57.050, -40.834, Math.toRadians(90));
//    private final Pose intake2Pose = new Pose(57.764, -2.378, Math.toRadians(90));
//    private final Pose prepare3Pose = new Pose(34.876, -40.730, Math.toRadians(90));
//    private final Pose intake3Pose = new Pose(34.392, -2.932, Math.toRadians(90));
//    private int currentPathId = 0;
//
//
//    @Override
//    public void initialize() {
//        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        follower = new FollowerEx(hardwareMap, FConstants.class, LConstants.class);
//        follower.setStartingPose(startPose);
//        shooter = new Shooter(hardwareMap);
//        intake = new Intake(hardwareMap);
//        this.pathChainList = new PathChainList();
//        this.actions = new ArrayList<>();
//        this.autoCommand = new AutoCommand(shooter, intake);
//        this.actionRunning = false;
//
//        buildPaths();
//        buildActions();
//
//        follower.setMaxPower(1);
//    }
//
//    @NonNull
//    private Point getCurrentPoint(){
//        return new Point(follower.getPose().getX(),follower.getPose().getY());
//    }
//
//    private double getCurrentHeading(){
//        return follower.getPose().getHeading();
//    }
//
//    private void buildPaths() {
//        PathChain scorePreload, prepare1, intake1, after1,
//                prepare2, intake2, after2,
//                prepare3, intake3, after3,
//                score1, score2, score3;
//        PathChain park;
//
//        scorePreload = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
//                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
//                .build();
//
//        prepare1 = follower.pathBuilder()
//                .addPath(new BezierLine(getCurrentPoint(), new Point(prepare1Pose)))
//                .setLinearHeadingInterpolation(getCurrentHeading(), prepare1Pose.getHeading())
//                .build();
//
//        intake1 = follower.pathBuilder()
//                .addPath(new BezierCurve(getCurrentPoint(), new Point(intake1Pose), new Point(openGatePose)))
//                .setLinearHeadingInterpolation(getCurrentHeading(), openGatePose.getHeading())
//                .build();
//
//        after1 = follower.pathBuilder()
//                .addPath(new BezierLine(getCurrentPoint(), new Point(prepare1Pose)))
//                .setLinearHeadingInterpolation(getCurrentHeading(), prepare1Pose.getHeading())
//                .build();
//
//        score1 = follower.pathBuilder()
//                .addPath(new BezierLine(getCurrentPoint(), new Point(scorePose)))
//                .setLinearHeadingInterpolation(getCurrentHeading(), scorePose.getHeading())
//                .build();
//
//        prepare2 = follower.pathBuilder()
//                .addPath(new BezierLine(getCurrentPoint(), new Point(prepare2Pose)))
//                .setLinearHeadingInterpolation(getCurrentHeading(), prepare2Pose.getHeading())
//                .build();
//
//        intake2 = follower.pathBuilder()
//                .addPath(new BezierLine(getCurrentPoint(), new Point(intake2Pose)))
//                .setLinearHeadingInterpolation(getCurrentHeading(), intake2Pose.getHeading())
//                .build();
//
//        after2 = follower.pathBuilder()
//                .addPath(new BezierLine(getCurrentPoint(), new Point(prepare2Pose)))
//                .setLinearHeadingInterpolation(getCurrentHeading(), prepare2Pose.getHeading())
//                .build();
//
//        score2 = follower.pathBuilder()
//                .addPath(new BezierLine(getCurrentPoint(), new Point(scorePose)))
//                .setLinearHeadingInterpolation(getCurrentHeading(), scorePose.getHeading())
//                .build();
//
//        prepare3 = follower.pathBuilder()
//                .addPath(new BezierLine(getCurrentPoint(), new Point(prepare3Pose)))
//                .setLinearHeadingInterpolation(getCurrentHeading(), prepare3Pose.getHeading())
//                .build();
//
//        intake3 = follower.pathBuilder()
//                .addPath(new BezierLine(getCurrentPoint(), new Point(intake3Pose)))
//                .setLinearHeadingInterpolation(getCurrentHeading(), intake3Pose.getHeading())
//                .build();
//
//        after3 = follower.pathBuilder()
//                .addPath(new BezierLine(getCurrentPoint(), new Point(prepare2Pose)))
//                .setLinearHeadingInterpolation(getCurrentHeading(), prepare3Pose.getHeading())
//                .build();
//
//        score3 = follower.pathBuilder()
//                .addPath(new BezierLine(getCurrentPoint(), new Point(scorePose)))
//                .setLinearHeadingInterpolation(getCurrentHeading(), scorePose.getHeading())
//                .build();
//
//        park = follower.pathBuilder()
//                .addPath(new BezierLine(getCurrentPoint(), new Point(startPose)))
//                .setLinearHeadingInterpolation(getCurrentHeading(), startPose.getHeading())
//                .build();
//
//        pathChainList.addPath(scorePreload, null, null,
//                park,
//                null);
//    }
//
//    @NonNull
//    private Command actionEnd(){
//        return new InstantCommand(()->this.actionRunning = false);
//    }
//
//    private void buildActions(){
//        Command intakeCommand, accelerateCommand, scoreCommand, stopCommand;
//        scoreCommand = autoCommand.shoot().andThen(actionEnd());
//        intakeCommand = autoCommand.intake().andThen(actionEnd());
//        accelerateCommand = autoCommand.accelerate().andThen(actionEnd());
//        stopCommand = autoCommand.stopAll().andThen(actionEnd());
//
//        actions.addAll(Arrays.asList(accelerateCommand, intakeCommand, scoreCommand,
//                stopCommand,
//                new WaitCommand(999999)));
//    }
//
//    private void periodic() {
//        CommandScheduler.getInstance().run();
//        follower.update();
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.addData("drive error",follower.driveError);
//        telemetry.addData("follower finished",follower.isFinished);
//        telemetry.addData("action finished", !this.actionRunning);
//        telemetry.addData("current path /id", currentPathId);
//        telemetry.addData("Actions size", actions.size());
//        telemetry.addData("PathChainList size", pathChainList.size());
//        telemetry.update();
//    }
//
//    @Override
//    public void run() {
//        if(actions.size() != pathChainList.size()){
//            throw new IllegalStateException(
//                    "Actions count (" + actions.size() +
//                            ") does not match path count (" + pathChainList.size() + ")"
//            );
//        }
//        Iterator<PathChain> it = pathChainList.iterator();
//        int pathCount = 0;
//        while (it.hasNext()){
//            pathCount+=1;
//            if (!opModeIsActive())break;
//            periodic();
//            if(!follower.isBusy() && !this.actionRunning){
//                PathChain path = it.next();
//                if(path!=null){
//                    if (pathCount == 5 || pathCount == 10 || pathCount == 15){
//                        follower.setSlowPID();
//                        follower.followPath(path,0.2,false);
//                    }
//                    else {
//                        follower.setFastPID();
//                        follower.followPath(path, 1,true);
//                    }
//                }
//                Command currentAction = actions.get(currentPathId);
//                if(currentAction!=null){
//                    currentAction.schedule();
//                    this.actionRunning = true;
//                }
//                currentPathId++;
//            }
//        }
//    }
//}