package org.firstinspires.ftc.teamcode.BirdNest.lib.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.00052189;
        ThreeWheelConstants.strafeTicksToInches = 0.00052189;
        ThreeWheelConstants.turnTicksToInches = 0.00053717;
        ThreeWheelConstants.leftY = 2.9317;
        ThreeWheelConstants.rightY = -2.9317;
        ThreeWheelConstants.strafeX = 0.57;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "FrontLeft";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "FrontRight";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "BackLeft";
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD; //Encoder.REVERSE; //TODO Check Reverse
        ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




