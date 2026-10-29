package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.core.PoseFilter;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.horizon36596.simloop.fakehardware.FakeLimelight3A;
import org.horizon36596.simloop.vision.CameraMount;
import org.junit.jupiter.api.Test;

/**
 * The pose filter's idea of where the hive's tags are and how the turret's Limelight sees them (HiveTags,
 * PoseFilter.measure) against the sim's own (SimHives, FakeLimelight3A): the same tags in the same camera-frame
 * places, from a few robot poses, turret angles and arm angles, both hives.
 */
class HiveTagsSimTest {
    @Test
    void theFilterSeesTheTagsWhereTheSimPutsThem() {
        double[][] poses = {{40, 30, Math.toRadians(70)}, {30, 110, Math.toRadians(-60)}, {104, 30, Math.toRadians(120)},
                {110, 112, Math.toRadians(-120)}, {56, 20, Math.toRadians(90)}};
        double[] turrets = {0, Math.toRadians(25), Math.toRadians(-40)};
        double[][] arms = {{Math.toRadians(-28.6), Math.toRadians(29.3)}, {Math.toRadians(29.4), Math.toRadians(-28.7)},
                {Math.toRadians(5), Math.toRadians(-12)}};
        PoseFilter filter = new PoseFilter(new PoseFilter.Camera(RobotConstants.Launcher.TURRET_OFFSET_X,
                RobotConstants.Launcher.TURRET_OFFSET_Y, RobotConstants.Vision.LENS_AHEAD,
                RobotConstants.Vision.LENS_HEIGHT, RobotConstants.Vision.TILT), 0, 0, 0, 0, 0);
        int compared = 0;
        double worst = 0;
        for (double[] arm : arms) {
            for (double[] pose : poses) {
                for (double turret : turrets) {
                    FakeLimelight3A limelight = FakeLimelight3A.builder(SimHives.layout(arm[0], arm[1]),
                            new CameraMount(4.8, 0, RobotConstants.Vision.LENS_HEIGHT, 0, -RobotConstants.Vision.TILT),
                            54.5, 42).build();
                    double h = pose[2];
                    double ax = pose[0] + RobotConstants.Launcher.TURRET_OFFSET_X * Math.cos(h)
                            - RobotConstants.Launcher.TURRET_OFFSET_Y * Math.sin(h);
                    double ay = pose[1] + RobotConstants.Launcher.TURRET_OFFSET_X * Math.sin(h)
                            + RobotConstants.Launcher.TURRET_OFFSET_Y * Math.cos(h);
                    limelight.setRobotPose(ax - 72, ay - 72, h + turret);
                    limelight.start();
                    limelight.update(0.02);
                    limelight.update(0.02);
                    LLResult result = limelight.getLatestResult();
                    if (result == null || !result.isValid()) continue;
                    double[] state = {pose[0], pose[1], h, arm[0], arm[1]};
                    for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
                        Position p = f.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
                        double[] expected = filter.measure(state, turret, f.getFiducialId());
                        String what = String.format("tag %d, robot %.0f,%.0f,%.0f°, turret %.0f°, arms %.1f°/%.1f°",
                                f.getFiducialId(), pose[0], pose[1], Math.toDegrees(h), Math.toDegrees(turret),
                                Math.toDegrees(arm[0]), Math.toDegrees(arm[1]));
                        assertTrue(expected != null, what + ": the filter has it behind the lens");
                        double off = Math.sqrt(Math.pow(p.x - expected[0], 2) + Math.pow(p.y - expected[1], 2)
                                + Math.pow(p.z - expected[2], 2));
                        worst = Math.max(worst, off);
                        assertEquals(0, off, 0.05, what + String.format(": sim %.2f %.2f %.2f, filter %.2f %.2f %.2f",
                                p.x, p.y, p.z, expected[0], expected[1], expected[2]));
                        compared++;
                    }
                }
            }
        }
        System.out.printf("Hive tags: %d sightings compared, worst %.4f in apart%n", compared, worst);
        assertTrue(compared > 30, "the Limelight should have seen tags from these poses: " + compared);
    }
}
