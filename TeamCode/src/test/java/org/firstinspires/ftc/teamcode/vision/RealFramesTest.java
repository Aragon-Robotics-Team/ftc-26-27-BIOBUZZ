package org.firstinspires.ftc.teamcode.vision;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.BallVision;
import org.junit.jupiter.api.Test;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Scores the robot's ball vision (BallVisionProcessor and BallPipeline, as configured in RobotConstants.BallVision)
 * against the hand-labeled Arducam frames in biobuzz_detection's data/biobuzz_gt: 329 frames from three videos, every
 * pollen and nectar boxed. The same rules as its evaluate.py: same-class boxes matching at IoU 0.3 or better, labels
 * marked iscrowd ignored either way, and a detection with no label is a false one. Frames go through in order, one
 * processor per video, like a live stream. The videos were taken with the lens 7.2 in up and tilted 30° down, so the
 * geometry uses that mount with the robot's calibration; the labels are moved through the same undistortion.
 *
 * Prints recall and false detections by video, by condition (clear or blur) and by class, and how long the pipeline
 * took per frame on this computer (the Control Hub is several times slower). Skipped when the dataset isn't there:
 * biobuzz_detection next to this repo, or -DbiobuzzDetection=path.
 */
class RealFramesTest {
    private static final double IOU = 0.3;
    private static final double VIDEO_HEIGHT = 7.2, VIDEO_TILT = Math.toRadians(30);
    private static final String[] CLASSES = {"pollen", "red_nectar", "blue_nectar"};

    private static final class Label {
        final String cls;
        final double[] box; // x0, y0, x1, y1, pipeline pixels
        final boolean ignore;
        boolean found;

        Label(String cls, double[] box, boolean ignore) {
            this.cls = cls;
            this.box = box;
            this.ignore = ignore;
        }
    }

    private static final class Tally {
        int images, tp, fp, fn;
    }

    @Test
    void scoreBallVisionOnLabeledArducamFrames() throws IOException {
        Path root = Paths.get(System.getProperty("biobuzzDetection", "../../biobuzz_detection"));
        Path gt = root.resolve("data/biobuzz_gt/annotations.coco.json");
        assumeTrue(Files.isRegularFile(gt), "no labeled frames at " + gt.toAbsolutePath());
        nu.pattern.OpenCV.loadLocally();

        String json = new String(Files.readAllBytes(gt), StandardCharsets.UTF_8);
        List<Map<String, String>> images = objects(json, "images");
        Map<String, String> classById = new TreeMap<>();
        for (Map<String, String> c : objects(json, "categories")) classById.put(c.get("id"), c.get("name"));
        Map<String, List<Map<String, String>>> labelsByImage = new LinkedHashMap<>();
        for (Map<String, String> a : objects(json, "annotations")) {
            labelsByImage.computeIfAbsent(a.get("image_id"), k -> new ArrayList<>()).add(a);
        }
        images.sort(Comparator.comparing((Map<String, String> im) -> im.get("video"))
                .thenComparingInt(im -> Integer.parseInt(im.get("frame"))));

        BallGeometry robot = BallVision.geometry();
        BallGeometry.Mount mount = new BallGeometry.Mount(RobotConstants.BallVision.LENS_FORWARD,
                RobotConstants.BallVision.LENS_LEFT, VIDEO_HEIGHT, 0, VIDEO_TILT, 0);
        BallGeometry geometry = BallGeometry.fromCalibration(robot.width, robot.height, robot.width, robot.height,
                robot.fx, robot.fy, robot.cx, robot.cy, mount);
        Mat k = new Mat(3, 3, CvType.CV_64F);
        k.put(0, 0, geometry.fx, 0, geometry.cx, 0, geometry.fy, geometry.cy, 0, 0, 1);
        double[] distortion = RobotConstants.BallVision.DISTORTION;
        Mat d = new Mat(1, distortion.length, CvType.CV_64F);
        d.put(0, 0, distortion);

        Map<String, Tally> tallies = new LinkedHashMap<>();
        List<Double> ms = new ArrayList<>();
        String video = null;
        BallVisionProcessor processor = null;
        for (Map<String, String> im : images) {
            if (!im.get("video").equals(video)) {
                video = im.get("video");
                processor = new BallVisionProcessor(geometry, new BallPipeline(geometry, BallVision.pipelineSettings()),
                        distortion);
                processor.init(Integer.parseInt(im.get("width")), Integer.parseInt(im.get("height")), null);
            }
            Mat bgr = Imgcodecs.imread(root.resolve(im.get("file_name")).toString());
            assertTrue(!bgr.empty(), "can't read " + im.get("file_name"));
            Mat rgb = new Mat();
            Imgproc.cvtColor(bgr, rgb, Imgproc.COLOR_BGR2RGB);
            BallVisionProcessor.Result result = (BallVisionProcessor.Result) processor.processFrame(rgb, 0);
            bgr.release();
            rgb.release();
            ms.add(result.detectMs);

            double scale = geometry.width / Double.parseDouble(im.get("width"));
            List<Label> labels = new ArrayList<>();
            for (Map<String, String> a : labelsByImage.getOrDefault(im.get("id"), new ArrayList<>())) {
                double[] b = Arrays.stream(a.get("bbox").split(",")).mapToDouble(s -> Double.parseDouble(s.trim()) * scale)
                        .toArray();
                labels.add(new Label(classById.get(a.get("category_id")),
                        undistortBox(b[0], b[1], b[0] + b[2], b[1] + b[3], k, d), "1".equals(a.get("iscrowd"))));
            }
            score(result.balls, labels, geometry, im, tallies);
        }

        System.out.println("Ball vision on " + images.size() + " labeled Arducam frames (RealFramesTest):");
        System.out.printf("  %-18s %6s %6s %6s %7s %6s %9s%n", "", "images", "balls", "found", "recall", "false",
                "precision");
        for (Map.Entry<String, Tally> e : tallies.entrySet()) {
            Tally t = e.getValue();
            System.out.printf("  %-18s %6d %6d %6d %7s %6d %9s%n", e.getKey(), t.images, t.tp + t.fn, t.tp,
                    pct(t.tp, t.tp + t.fn), t.fp, pct(t.tp, t.tp + t.fp));
        }
        ms.sort(null);
        System.out.printf("  pipeline per frame on this computer: median %.1f ms, 95th percentile %.1f ms, max %.1f ms%n",
                ms.get(ms.size() / 2), ms.get((int) (ms.size() * 0.95)), ms.get(ms.size() - 1));
        assertTrue(tallies.get("all").images == images.size());
    }

    /** Greedy same-class IoU matching, as evaluate.py does; adds the outcome to the image's tallies. */
    private static void score(List<Ball> balls, List<Label> labels, BallGeometry geometry, Map<String, String> im,
            Map<String, Tally> tallies) {
        List<double[]> pairs = new ArrayList<>(); // iou, ball, label
        for (int bi = 0; bi < balls.size(); bi++) {
            for (int li = 0; li < labels.size(); li++) {
                if (!name(balls.get(bi).type).equals(labels.get(li).cls)) continue;
                double v = iou(box(balls.get(bi), geometry), labels.get(li).box);
                if (v >= IOU) pairs.add(new double[] {v, bi, li});
            }
        }
        pairs.sort((a, b) -> Double.compare(b[0], a[0]));
        int[] ballTo = new int[balls.size()];
        Arrays.fill(ballTo, -1);
        for (double[] p : pairs) {
            int bi = (int) p[1], li = (int) p[2];
            if (ballTo[bi] < 0 && !labels.get(li).found) {
                ballTo[bi] = li;
                labels.get(li).found = true;
            }
        }
        String[] keys = {"all", "video " + im.get("video"), im.get("condition")};
        for (String key : keys) tallies.computeIfAbsent(key, x -> new Tally()).images++;
        for (String cls : CLASSES) tallies.computeIfAbsent("class " + cls, x -> new Tally());
        for (int bi = 0; bi < balls.size(); bi++) {
            String cls = name(balls.get(bi).type);
            boolean tp = ballTo[bi] >= 0 && !labels.get(ballTo[bi]).ignore;
            boolean free = ballTo[bi] >= 0 ? labels.get(ballTo[bi]).ignore : onIgnored(box(balls.get(bi), geometry), labels);
            if (free) continue;
            for (String key : keys) {
                if (tp) tallies.get(key).tp++;
                else tallies.get(key).fp++;
            }
            if (tp) tallies.get("class " + cls).tp++;
            else tallies.get("class " + cls).fp++;
        }
        for (Label l : labels) {
            if (l.ignore || l.found) continue;
            for (String key : keys) tallies.get(key).fn++;
            tallies.get("class " + l.cls).fn++;
        }
        for (String cls : CLASSES) {
            for (Label l : labels) {
                if (l.cls.equals(cls)) {
                    tallies.get("class " + cls).images++;
                    break;
                }
            }
        }
    }

    private static boolean onIgnored(double[] box, List<Label> labels) {
        for (Label l : labels) if (l.ignore && iou(box, l.box) >= IOU) return true;
        return false;
    }

    private static String name(Ball.Type type) {
        return type.name().toLowerCase();
    }

    /** The detection's box, as evaluate.py makes it: its circle's square, clipped to the frame. */
    private static double[] box(Ball b, BallGeometry g) {
        return new double[] {Math.max(0, b.x - b.radius), Math.max(0, b.y - b.radius), Math.min(g.width, b.x + b.radius),
                Math.min(g.height, b.y + b.radius)};
    }

    /** The box around a label's corners and edge middles once undistorted, as the pipeline sees the frame. */
    private static double[] undistortBox(double x0, double y0, double x1, double y1, Mat k, Mat d) {
        double mx = (x0 + x1) / 2, my = (y0 + y1) / 2;
        MatOfPoint2f in = new MatOfPoint2f(new Point(x0, y0), new Point(mx, y0), new Point(x1, y0), new Point(x1, my),
                new Point(x1, y1), new Point(mx, y1), new Point(x0, y1), new Point(x0, my));
        MatOfPoint2f out = new MatOfPoint2f();
        Calib3d.undistortPoints(in, out, k, d, new Mat(), k);
        double[] b = {Double.MAX_VALUE, Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
        for (Point p : out.toArray()) {
            b[0] = Math.min(b[0], p.x);
            b[1] = Math.min(b[1], p.y);
            b[2] = Math.max(b[2], p.x);
            b[3] = Math.max(b[3], p.y);
        }
        return b;
    }

    private static double iou(double[] a, double[] b) {
        double w = Math.min(a[2], b[2]) - Math.max(a[0], b[0]), h = Math.min(a[3], b[3]) - Math.max(a[1], b[1]);
        if (w <= 0 || h <= 0) return 0;
        double inter = w * h;
        return inter / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter);
    }

    private static String pct(int a, int b) {
        return b == 0 ? "n/a" : String.format("%.1f%%", 100.0 * a / b);
    }

    /**
     * The flat objects in the named top-level array of a COCO file, each as field name to value (arrays as their
     * comma-separated contents, strings without quotes). Enough for this file; not a JSON parser.
     */
    private static List<Map<String, String>> objects(String json, String array) {
        int start = json.indexOf('[', json.indexOf("\"" + array + "\""));
        int depth = 0, end = start;
        for (int i = start; i < json.length(); i++) {
            char c = json.charAt(i);
            if (c == '[') depth++;
            else if (c == ']' && --depth == 0) {
                end = i;
                break;
            }
        }
        List<Map<String, String>> out = new ArrayList<>();
        Matcher object = Pattern.compile("\\{([^{}]*)\\}").matcher(json.substring(start, end));
        Pattern field = Pattern.compile("\"(\\w+)\"\\s*:\\s*(\"([^\"]*)\"|\\[([^\\]]*)\\]|[^,}\\s]+)");
        while (object.find()) {
            Map<String, String> m = new LinkedHashMap<>();
            Matcher f = field.matcher(object.group(1));
            while (f.find()) {
                m.put(f.group(1), f.group(3) != null ? f.group(3) : f.group(4) != null ? f.group(4) : f.group(2));
            }
            out.add(m);
        }
        return out;
    }
}
