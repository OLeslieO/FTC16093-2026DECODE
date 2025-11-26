//package org.firstinspires.ftc.teamcode.opmodes.autos;
//
//import static java.lang.Thread.sleep;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.command.Command;
//
////import com.pedropathing.localization.Pose;
////import com.pedropathing.pathgen.BezierCurve;
////import com.pedropathing.pathgen.BezierLine;
////import com.pedropathing.pathgen.PathChain;
////import com.pedropathing.pathgen.Point;
//import com.pedropathing.geometry.BezierCurve;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//
//import org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand;
//import org.firstinspires.ftc.teamcode.Subsystems.Intake;
//import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
////import org.firstinspires.ftc.teamcode.utils.FollowerEx;
////import org.firstinspires.ftc.teamcode.utils.PathChainList;
//
//
//import java.util.ArrayList;
//import java.util.List;
//
//import pedroPathing.Constants;
//
//
//@Autonomous(name = "Auto New", group = "Competition")
//public class AutoNearBlueNew extends AutoOpModeEx {
//    private Follower follower;
//    private AutoCommand autoCommand;
//    private List<Command> actions;
//    private Shooter shooter;
//    private Intake intake;
//    private Boolean actionRunning;
//    private int pathState;
//
//    private Timer pathTimer, actionTimer, opmodeTimer;
//
//    private final Pose startPose = new Pose(123.850, -21.490, Math.toRadians(45));
//
//    private final Pose scorePose = new Pose(93.644, -43.420, Math.toRadians(47));
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
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//
//        follower = Constants.createFollower(hardwareMap);
//        shooter = new Shooter(hardwareMap);
//        intake = new Intake(hardwareMap);
//        follower.setStartingPose(startPose);
//        buildPaths();
//        this.actions = new ArrayList<>();
//        this.autoCommand = new AutoCommand(shooter, intake);
//        this.actionRunning = false;
//
//
////        buildActions();
//
//        follower.setMaxPower(1);
//    }
//
//    private double getCurrentHeading(){
//        return follower.getPose().getHeading();
//    }
//
//    PathChain scorePreload, prepare1, intake1, after1,
//            prepare2, intake2, after2,
//            prepare3, intake3, after3,
//            score1, score2, score3;
//
//    private void buildPaths() {
//        scorePreload = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, scorePose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
//                .addParametricCallback(0, shooter::accelerate_mid)
//                .addParametricCallback(0.6, intake::intake)
//                .addParametricCallback(1,shooter::shoot)
//                .setBrakingStrength(1)
//                .build();
//
//        prepare1 = follower.pathBuilder()
//                .addPath(new BezierLine(follower.getPose(), prepare1Pose))
//                .setLinearHeadingInterpolation(getCurrentHeading(), prepare1Pose.getHeading())
//                .setBrakingStrength(1)
//                .build();
//
//        intake1 = follower.pathBuilder()
//                .addPath(new BezierCurve(follower.getPose(), intake1Pose, openGatePose))
//                .setLinearHeadingInterpolation(getCurrentHeading(), openGatePose.getHeading())
//                .setBrakingStrength(0.1)
//                .build();
//
//        after1 = follower.pathBuilder()
//                .addPath(new BezierLine(follower.getPose(), prepare1Pose))
//                .setLinearHeadingInterpolation(getCurrentHeading(), prepare1Pose.getHeading())
//                .setBrakingStrength(1)
//                .build();
//
//        score1 = follower.pathBuilder()
//                .addPath(new BezierLine(follower.getPose(), scorePose))
//                .setLinearHeadingInterpolation(getCurrentHeading(), scorePose.getHeading())
//                .addParametricCallback(1,shooter::shoot)
//                .setBrakingStrength(1)
//                .build();
//
//        prepare2 = follower.pathBuilder()
//                .addPath(new BezierLine(follower.getPose(), prepare2Pose))
//                .setLinearHeadingInterpolation(getCurrentHeading(), prepare2Pose.getHeading())
//                .setBrakingStrength(1)
//                .build();
//
//        intake2 = follower.pathBuilder()
//                .addPath(new BezierLine(follower.getPose(), intake2Pose))
//                .setLinearHeadingInterpolation(getCurrentHeading(), intake2Pose.getHeading())
//                .setBrakingStrength(0.1)
//                .build();
//
//        after2 = follower.pathBuilder()
//                .addPath(new BezierLine(follower.getPose(), prepare2Pose))
//                .setLinearHeadingInterpolation(getCurrentHeading(), prepare2Pose.getHeading())
//                .build();
//
//        score2 = follower.pathBuilder()
//                .addPath(new BezierLine(follower.getPose(), scorePose))
//                .setLinearHeadingInterpolation(getCurrentHeading(), scorePose.getHeading())
//                .build();
//
//        prepare3 = follower.pathBuilder()
//                .addPath(new BezierLine(follower.getPose(), prepare3Pose))
//                .setLinearHeadingInterpolation(getCurrentHeading(), prepare3Pose.getHeading())
//                .build();
//
//        intake3 = follower.pathBuilder()
//                .addPath(new BezierLine(follower.getPose(), intake3Pose))
//                .setLinearHeadingInterpolation(getCurrentHeading(), intake3Pose.getHeading())
//                .build();
//
//        after3 = follower.pathBuilder()
//                .addPath(new BezierLine(follower.getPose(), prepare2Pose))
//                .setLinearHeadingInterpolation(getCurrentHeading(), prepare3Pose.getHeading())
//                .build();
//
//        score3 = follower.pathBuilder()
//                .addPath(new BezierLine(follower.getPose(), scorePose))
//                .setLinearHeadingInterpolation(getCurrentHeading(), scorePose.getHeading())
//                .build();
//    }
//
//
//
//    /* You could check for
//    - Follower State: "if(!follower.isBusy()) {}"
//    - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
//    - Robot Position: "if(follower.getPose().getX() > 36) {}"
//    */
//    public void autonomousPathUpdate() throws InterruptedException {
//        switch (pathState) {
//            case 0:
//                follower.followPath(scorePreload);
//                if(pathTimer.getElapsedTime()>4000){
//                    shooter.init();
//                }
//                setPathState(1);
//                break;
//            case 1:
//                if(!follower.isBusy()) {
//                    follower.followPath(prepare1,false);
//                    setPathState(2);
//                }
//                break;
//            case 2:
//                if(!follower.isBusy()) {
//                    follower.followPath(intake1,true);
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                if(!follower.isBusy()) {
//                    follower.followPath(score1,true);
//                    if(!follower.isBusy()){
//                        sleep(2000);
//                    }
//                    shooter.init();
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                if(!follower.isBusy()) {
//                    follower.followPath(prepare2,false);
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                if(!follower.isBusy()) {
//                    follower.followPath(intake2,true);
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                if(!follower.isBusy()) {
//                    follower.followPath(score2, true);
//                    if(!follower.isBusy()){
//                        sleep(2000);
//                    }
//                    shooter.init();
//                    setPathState(7);
//                }
//                break;
//
//            case 7:
//                if(!follower.isBusy()) {
//                    follower.followPath(prepare2,false);
//                    setPathState(8);
//                }
//                break;
//            case 8:
//                if(!follower.isBusy()) {
//                    follower.followPath(intake2,true);
//                    setPathState(9);
//                }
//                break;
//            case 9:
//                if(!follower.isBusy()) {
//                    follower.followPath(score2, true);
//                    if(!follower.isBusy()){
//                        sleep(2000);
//                    }
//                    shooter.init();
//                    setPathState(-1);
//                }
//                break;
//        }
//    }
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//    @Override
//    public void run() {
//        // These loop the movements of the robot, these must be called continuously in order to work
//        follower.update();
//        try {
//            autonomousPathUpdate();
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//
//        // Feedback to Driver Hub for debugging
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.update();
//    }
//}