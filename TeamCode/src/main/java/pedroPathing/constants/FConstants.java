package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.THREE_WHEEL;

        FollowerConstants.leftFrontMotorName = "FrontLeft";
        FollowerConstants.leftRearMotorName = "BackLeft";
        FollowerConstants.rightFrontMotorName = "FrontRight";
        FollowerConstants.rightRearMotorName = "BackRight";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.REVERSE;

        FollowerConstants.mass = 16.5293;

        FollowerConstants.xMovement = 46.94;
        FollowerConstants.yMovement = 31.8154;

        FollowerConstants.forwardZeroPowerAcceleration = -77.0936; //-55.521
        FollowerConstants.lateralZeroPowerAcceleration = -99.9748; //-121.2911

        FollowerConstants.translationalPIDFCoefficients = new CustomPIDFCoefficients(.15,.1,.05,.1);
        //FollowerConstants.translationalPIDFCoefficients = new CustomPIDFCoefficients(.1,.1,.05,.1);
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients = new CustomPIDFCoefficients(.085,.0,0.05,.0);
        //FollowerConstants.secondaryTranslationalPIDFCoefficients = new CustomPIDFCoefficients(.015,.0,0.10,.0);

        FollowerConstants.headingPIDFCoefficients = new CustomPIDFCoefficients(2,0,0.1,0);
        //FollowerConstants.headingPIDFCoefficients = new CustomPIDFCoefficients(2,0,0.1,0);
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients = new CustomPIDFCoefficients(2,0,0.1,0);
        //FollowerConstants.secondaryHeadingPIDFCoefficients = new CustomPIDFCoefficients(2,0,0.1,0);

        FollowerConstants.drivePIDFCoefficients = new CustomFilteredPIDFCoefficients(0.0125,0.0,0.00001,0.2,.0);
        //FollowerConstants.drivePIDFCoefficients = new CustomFilteredPIDFCoefficients(0.0125,0.0,0.00001,0.2,.0);
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients = new CustomFilteredPIDFCoefficients(0.01,0,.00000075,0.6,0.0);
        //FollowerConstants.secondaryDrivePIDFCoefficients = new CustomFilteredPIDFCoefficients(0.00375,0,.00000075,0.6,0.0);

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.0075;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
