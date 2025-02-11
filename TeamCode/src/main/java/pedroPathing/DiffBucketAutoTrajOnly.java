package pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Diff Bucket Trajectory")
public class DiffBucketAutoTrajOnly extends OpMode {
    private Follower follower;
    private Pose startPose = new Pose(133, 40, Math.toRadians(180));
    private Pose spacingPose = new Pose(128, 40, Math.toRadians(180));
    private Pose scoringCtrlPose = new Pose(128.037, 18.651);
    private Pose scoringPose = new Pose(131, 13, Math.toRadians(135));
    private Pose firstGrabPose = new Pose(116, 14, Math.toRadians(157.5));
    private Pose secondGrabPose = new Pose(116, 14, Math.toRadians(180));
    private Pose thirdGrabPose = new Pose(116, 14, Math.toRadians(211.875));
    private Pose parkctrl1 = new Pose(98.296, 11.762, /* PLACEHOLDER */ Math.toRadians(0));
    private Pose parkctrl2 = new Pose(76.285, 16.299, /* PLACEHOLDER */ Math.toRadians(0));
    private Pose Park = new Pose(78, 51, 180);
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private PathChain preload_space, preload_score, grabFirstSample, scoreFirstSample, grabSecondSample, scoreSecondSample, grabThirdSample, scoreThirdSample, park;
public void buildPaths() {
    preload_space = follower.pathBuilder()
            .addPath(
                    new BezierLine(
                            new Point(startPose),
                            new Point(spacingPose)
                    )
            )
            .setLinearHeadingInterpolation(startPose.getHeading(), spacingPose.getHeading())
            .build();

    preload_score = follower.pathBuilder()
            .addPath(
                    new BezierCurve(
                            new Point(spacingPose),
                            new Point(scoringCtrlPose),
                            new Point(scoringPose)
                    )
            )
            .setLinearHeadingInterpolation(spacingPose.getHeading(), scoringPose.getHeading())
            .build();

    grabFirstSample = follower.pathBuilder()
            .addPath(
                    new BezierLine(
                            new Point(scoringPose),
                            new Point(firstGrabPose)
                    )
            )
            .setLinearHeadingInterpolation(scoringPose.getHeading(), firstGrabPose.getHeading()) //Gotta manually get heading
            .build();

    scoreFirstSample = follower.pathBuilder()
            .addPath(
                    new BezierCurve(
                            new Point(firstGrabPose),
                            new Point(scoringCtrlPose),  //The idea is that the tangent is always set the same after sample grab. The curves sharing the same ctrl point hopes to emulate.
                            new Point(scoringPose)
                    )
            )
            .setLinearHeadingInterpolation(firstGrabPose.getHeading(), scoringPose.getHeading())
            .build();

    grabSecondSample = follower.pathBuilder()
            .addPath(
                    new BezierLine(
                            new Point(scoringPose),
                            new Point(secondGrabPose)
                    )
            )
            .setLinearHeadingInterpolation(scoringPose.getHeading(), secondGrabPose.getHeading())
            .build();

    scoreSecondSample = follower.pathBuilder()
            .addPath(
                    new BezierCurve(
                            new Point(secondGrabPose),
                            new Point(scoringCtrlPose),
                            new Point(scoringPose)
                    )
            )
            .setLinearHeadingInterpolation(secondGrabPose.getHeading(), scoringPose.getHeading()) //If the interpolated heading is mismatched with the endpoints' headings, I think PID will kick in and turn the bot (similar to turnto() in RR)
            .build();

    grabThirdSample = follower.pathBuilder()
            .addPath(
                    new BezierLine(
                            new Point(scoringPose),
                            new Point(thirdGrabPose)
                    )
            )
            .setLinearHeadingInterpolation(scoringPose.getHeading(), thirdGrabPose.getHeading())
            .build();

    scoreThirdSample = follower.pathBuilder()
            .addPath(
                    new BezierCurve(
                            new Point(thirdGrabPose),
                            new Point(scoringCtrlPose),
                            new Point(scoringPose)
                    )
            )
            .setLinearHeadingInterpolation(thirdGrabPose.getHeading(), scoringPose.getHeading())
            .build();

    park = follower.pathBuilder()
            .addPath(
                    new BezierCurve(
                            new Point(scoringPose),
                            new Point(parkctrl1), //The first control point controls the tangent of the start
                            new Point(parkctrl2), //The second controls the tangent of the end, more are possible and will control tangents of specific points (please use visualizer)
                            new Point(Park)
                    )
            )
            .setLinearHeadingInterpolation(scoringPose.getHeading(), Park.getHeading())
            .build();
}
//ALL FOLLOWING COMMENTS ARE NOT MINE UNLESS SPECIFICALLY SAID.
//THE FOLLOWING CODE COMES FROM THE SAMPLE. I think having a continuously running switch case on a loop in auto is a good idea b/c it allows for deciding next action based on camera input with a time limit, etc.
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(preload_space);
                setPathState(1);
            case 1:
                if (!follower.isBusy()) {
                    //Add scoring stuff -M
                    follower.followPath(preload_score);
                    setPathState(2);
                }
                break;
            case 2:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabFirstSample,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scoreFirstSample,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabSecondSample,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scoreSecondSample,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabThirdSample,true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scoreThirdSample, true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(park,true);
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {  //Minh: Basically just a fat finite state machine, exit condition being non-movement
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;

                //We WILL have PID feedforward control next year (if the subsystems don't vibrate to the point where they generate too much noise) -Minh
                //We can recalculate PID power after the switch case -Minh
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void loop() { //Remind you of something? -M

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }

}

