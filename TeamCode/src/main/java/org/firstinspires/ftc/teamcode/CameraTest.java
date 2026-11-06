package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

/*
 * BioBuzz 38196 - Camera Test: reads the BIOBUZZ AprilTags with any webcam in the robot configuration
 * (Logitech C270 now). Nothing on the robot moves.
 *
 * The tags (IDs 30-45, 3.25 in.) are stuck on the BOTTOM of the four HIVE CELLS, facing down at the floor,
 * about 44 in. up, so the camera has to look UP to see them. The SDK already knows every BIOBUZZ tag.
 * To see what the camera sees: on the INIT screen, Driver Hub ⋮ menu → Camera Stream.
 */
@TeleOp(name = "Camera Test", group = "BioBuzz")
public class CameraTest extends LinearOpMode {

    private static final String GOOD = "#69f0ae", WARN = "#ffb74d", DIM = "#9e9e9e";

    @Override
    public void runOpMode() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        List<WebcamName> cams = hardwareMap.getAll(WebcamName.class);
        if (cams.isEmpty()) {
            telemetry.addLine("<big><b>Camera Test</b></big>");
            telemetry.addLine("<font color='" + WARN + "'>No webcam in the robot configuration.</font>");
            telemetry.addLine("Plug the webcam into the Control Hub's blue USB port, then on the Driver Hub: "
                    + "⋮ → Configure Robot → Edit BioBuzz → Scan. Name the new webcam <b>Webcam 1</b>, Done, Save.");
            telemetry.update();
            waitForStart();
            return;
        }

        AprilTagProcessor tags = new AprilTagProcessor.Builder()   // current game = every BIOBUZZ tag
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .build();
        VisionPortal camera = new VisionPortal.Builder()
                .setCamera(cams.get(0))
                .setCameraResolution(new Size(640, 480))           // the C270's calibrated size: distances are right
                .addProcessor(tags)
                .build();

        while (opModeInInit() || opModeIsActive()) {
            telemetry.addLine("<big><b>Camera Test</b></big>  <small>(" + cams.get(0).getDeviceName() + ")</small>");
            VisionPortal.CameraState state = camera.getCameraState();
            if (state != VisionPortal.CameraState.STREAMING) {
                telemetry.addLine("<font color='" + WARN + "'>Camera starting... (" + state + ")</font>");
            } else {
                List<AprilTagDetection> seen = tags.getDetections();
                if (seen.isEmpty()) {
                    telemetry.addLine("No tags in view. Point the camera UP at the bottom of a HIVE CELL.");
                }
                for (AprilTagDetection d : seen) {
                    // A whole CELL (its 4-tag sticker) is the steadiest reading; single tags are listed under it
                    if (d instanceof AprilTagClusterDetection) {
                        AprilTagClusterDetection c = (AprilTagClusterDetection) d;
                        telemetry.addLine(String.format("<font color='%s'><b>%s CELL</b></font>  <small>(%d%% of its sticker seen)</small>",
                                GOOD, c.metadata.name, c.percentClusterFound));
                        telemetry.addLine("    " + where(d));
                    }
                }
                StringBuilder ids = new StringBuilder();
                for (AprilTagDetection d : seen) {
                    if (!(d instanceof AprilTagSingleDetection)) continue;
                    AprilTagSingleDetection t = (AprilTagSingleDetection) d;
                    ids.append(ids.length() == 0 ? "" : ", ").append(t.id).append(t.metadata == null ? " (not BIOBUZZ)" : "");
                }
                if (ids.length() > 0) telemetry.addLine("Tags seen: <b>" + ids + "</b>");
                telemetry.addLine(String.format("<small><font color='%s'>Camera %.0f frames/s · see it: ⋮ → Camera Stream (before ▶)</font></small>",
                        DIM, camera.getFps()));
            }
            telemetry.update();
            sleep(50);
        }
        camera.close();
    }

    /** Distance and direction from the camera to what it sees, in plain words. */
    private static String where(AprilTagDetection d) {
        if (d.ftcPose == null) return "<small>(no distance yet)</small>";
        return String.format("%.0f in. away · %.0f° %s · %.0f° %s",
                d.ftcPose.range,
                Math.abs(d.ftcPose.bearing), d.ftcPose.bearing >= 0 ? "left" : "right",
                Math.abs(d.ftcPose.elevation), d.ftcPose.elevation >= 0 ? "up" : "down");
    }
}
