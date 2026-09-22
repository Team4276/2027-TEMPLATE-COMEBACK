package frc.robot.game;

import static org.wpilib.units.Units.Meters;

// import org.wpilib.vision.apriltag.AprilTag;
// import org.wpilib.vision.apriltag.AprilTagFieldLayout;
// import org.wpilib.vision.apriltag.AprilTagFields;
import org.wpilib.units.measure.Distance;

public class FieldLayout {
    // public static final AprilTagFieldLayout kApriltagLayout = AprilTagFieldLayout
    //         .loadField(AprilTagFields.k2026RebuiltWelded);

    public static final Distance kFieldLength = Meters.of(1.0);
    // Meters.of(kApriltagLayout.getFieldLength());
    public static final Distance kFieldWidth = Meters.of(1.0);
    // Meters.of(kApriltagLayout.getFieldWidth());

    public static class LinesVertical {
        public static final Distance kCenter = kFieldLength.div(2.0);
    }

    public static class LinesHorizontal {
        public static final Distance kCenter = kFieldWidth.div(2.0);
    }

    // public static AprilTag[] getAprilTagArrayFromIDs(int ids[]) {
    //     AprilTag buffer[] = new AprilTag[ids.length];
    //     int offset = 0;
    //     for (AprilTag tag : kApriltagLayout.getTags()) {
    //         for (int id : ids)
    //             if (id == tag.ID) {
    //                 buffer[offset++] = tag;
    //                 break;
    //             }
    //     }
    //     return buffer;
    // }

    // public static Integer[] getIDArrayFromAprilTagArray(AprilTag tags[]) {
    //     Integer buffer[] = new Integer[tags.length];
    //     for (int i = 0; i < tags.length; i++)
    //         buffer[i] = tags[i].ID;
    //     return buffer;
    // }
}
