package frc.lib.util;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.measure.Distance;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.internal.DriverStationBackend;
// import org.wpilib.smartdashboard.SmartDashboard;
import frc.robot.game.FieldLayout;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Mode;

public class AllianceFlipUtil {
    static {
        // SmartDashboard.putBoolean("Sim/OverrideFlip", false);
    }

    private static boolean overrideFlip = true;

    public static Distance flipX(Distance x) {
        return FieldLayout.kFieldLength.minus(x);
    }

    public static Distance flipY(Distance y) {
        return FieldLayout.kFieldWidth.minus(y);
    }

    public static Translation2d flip(Translation2d translation) {
        return new Translation2d(flipX(translation.getMeasureX()), flipY(translation.getMeasureY()));
    }

    public static Rotation2d flip(Rotation2d rotation) {
        return rotation.rotateBy(Rotation2d.PI);
    }

    public static Pose2d flip(Pose2d pose) {
        return new Pose2d(flip(pose.getTranslation()), flip(pose.getRotation()));
    }

    public static Distance applyX(Distance x) {
        return shouldFlip() ? flipX(x) : x;
    }

    public static Distance applyY(Distance y) {
        return shouldFlip() ? flipY(y) : y;
    }

    public static Translation2d apply(Translation2d translation) {
        return shouldFlip()
                ? flip(translation)
                : translation;
    }

    public static Rotation2d apply(Rotation2d rotation) {
        return shouldFlip() ? flip(rotation) : rotation;
    }

    public static Pose2d apply(Pose2d pose) {
        return shouldFlip()
                ? flip(pose)
                : pose;
    }

    /**
     * For SIM true sets to blue alliance
     *
     * @param shouldOverrideFlip
     */
    public static void overrideFlip(boolean shouldOverrideFlip) {
        overrideFlip = shouldOverrideFlip;
    }

    public static boolean shouldFlip() {
        // overrideFlip = SmartDashboard.getBoolean("Sim/OverrideFlip", overrideFlip);

        return DriverStationBackend.getAlliance().isPresent()
                && DriverStationBackend.getAlliance().get() == Alliance.RED
                && (RobotConstants.mode == Mode.SIM ? !overrideFlip : true);
    }
}
