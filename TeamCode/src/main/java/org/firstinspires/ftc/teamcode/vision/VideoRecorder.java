package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.media.MediaScannerConnection;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.io.BufferedOutputStream;
import java.io.BufferedWriter;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

/**
 * Records the raw webcam frames (full capture size, not undistorted) for building a dataset later. Each clip is two
 * files in /sdcard/FIRST/data/recordings, named by when it started:
 * - name.mjpeg: the frames as JPEGs one after another. VLC plays it, cv2.VideoCapture and biobuzz_detection's
 *   main.py --video read it, and ffmpeg turns it into an mp4.
 * - name.csv: one row per frame: seconds since the clip started (camera capture time) and the robot pose then.
 * Download them from http://192.168.43.1:8080/recordings (see {@link org.firstinspires.ftc.teamcode.util.RecordingsPage})
 * or over USB.
 *
 * Frames are copied on the camera's thread and compressed on a writer thread of the clip's own, so the camera never
 * waits on the disk. If the writer falls behind, frames are dropped (and counted) rather than queued without end.
 */
public class VideoRecorder implements VisionProcessor {
    public static final File DIR = new File(AppUtil.ROBOT_DATA_DIR, "recordings");
    private static final int QUEUE = 8;

    private final int jpegQuality;
    private final double maxFps;
    private volatile Clip clip; // null while not recording
    private volatile double[] pose = {Double.NaN, Double.NaN, Double.NaN};
    private volatile String problem;

    /** @param maxFps frames a second to keep at most (the camera sends about 30); 0 keeps them all */
    public VideoRecorder(int jpegQuality, double maxFps) {
        this.jpegQuality = jpegQuality;
        this.maxFps = maxFps;
    }

    /** Starts a new clip (finishing the one recording, if any). */
    public void start() {
        stop();
        try {
            String time = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss", Locale.US).format(new Date());
            // Two clips started in the same second (cross pressed twice) mustn't share the files
            String name = time;
            for (int i = 2; new File(DIR, name + ".mjpeg").exists(); i++) name = time + "_" + i;
            clip = new Clip(name);
            problem = null;
        } catch (IOException e) {
            RobotLog.ee("VideoRecorder", e, "Not recording");
            problem = "can't record: " + e.getMessage();
        }
    }

    /** Finishes the clip: the writer saves the frames it has queued, then closes the files. */
    public void stop() {
        Clip c = clip;
        clip = null;
        if (c != null) c.finish();
    }

    public boolean isRecording() {
        return clip != null;
    }

    /** The robot pose (x, y in inches, heading in radians) to write next to the frames from now on. */
    public void setPose(double x, double y, double heading) {
        pose = new double[]{x, y, heading};
    }

    /** One phrase for telemetry. */
    public String status() {
        if (problem != null) return problem;
        Clip c = clip;
        if (c == null) return "not recording";
        return String.format(Locale.US, "%s: %d frames (%d dropped), %.0f MB, %.0f s", c.name, c.written, c.dropped,
                c.bytes / 1e6, c.seconds);
    }

    // ---- VisionProcessor ----

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Clip c = clip;
        if (c != null) c.add(frame, captureTimeNanos, pose);
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx,
                            float scaleCanvasDensity, Object userContext) {}

    // ---- One clip ----

    private static class Frame {
        final Mat image;
        final long nanos;
        final double[] pose;

        Frame(Mat image, long nanos, double[] pose) {
            this.image = image;
            this.nanos = nanos;
            this.pose = pose;
        }
    }

    private class Clip implements Runnable {
        final String name;
        final File videoFile, csvFile;
        final BufferedOutputStream video;
        final BufferedWriter csv;
        final ArrayBlockingQueue<Frame> queue = new ArrayBlockingQueue<>(QUEUE);
        volatile boolean finishing = false;
        volatile int written = 0, dropped = 0;
        volatile long bytes = 0;
        volatile double seconds = 0;
        long firstNanos = -1, lastKeptNanos = System.nanoTime() - 10_000_000_000L;

        Clip(String name) throws IOException {
            if (!DIR.isDirectory() && !DIR.mkdirs()) throw new IOException("can't create " + DIR);
            this.name = name;
            videoFile = new File(DIR, name + ".mjpeg");
            csvFile = new File(DIR, name + ".csv");
            video = new BufferedOutputStream(new FileOutputStream(videoFile), 1 << 20);
            csv = new BufferedWriter(new FileWriter(csvFile), 1 << 16);
            csv.write("frame,t,x,y,heading\n");
            Thread writer = new Thread(this, "VideoRecorder " + name);
            writer.setPriority(Thread.NORM_PRIORITY - 1);
            writer.start();
        }

        /** Camera thread: copy the frame if it's time for one and the writer has room. */
        void add(Mat frame, long captureNanos, double[] pose) {
            // The camera's capture time, unless it's implausible (then now)
            long now = System.nanoTime();
            long nanos = captureNanos > now || captureNanos < now - 500_000_000L ? now : captureNanos;
            if (maxFps > 0 && nanos - lastKeptNanos < 1e9 / maxFps * 0.9) return;
            lastKeptNanos = nanos;
            if (queue.remainingCapacity() == 0) {
                dropped++;
                return;
            }
            queue.offer(new Frame(frame.clone(), nanos, pose));
        }

        void finish() {
            finishing = true;
        }

        @Override
        public void run() {
            ByteArrayOutputStream jpeg = new ByteArrayOutputStream(1 << 18);
            Bitmap bitmap = null;
            try {
                while (!finishing || !queue.isEmpty()) {
                    Frame f = queue.poll(100, TimeUnit.MILLISECONDS);
                    if (f == null) continue;
                    try {
                        if (bitmap == null || bitmap.getWidth() != f.image.cols() || bitmap.getHeight() != f.image.rows()) {
                            bitmap = Bitmap.createBitmap(f.image.cols(), f.image.rows(), Bitmap.Config.ARGB_8888);
                        }
                        Utils.matToBitmap(f.image, bitmap); // RGB or RGBA, as VisionPortal sends it
                    } finally {
                        f.image.release();
                    }
                    jpeg.reset();
                    bitmap.compress(Bitmap.CompressFormat.JPEG, jpegQuality, jpeg);
                    jpeg.writeTo(video);
                    if (firstNanos < 0) firstNanos = f.nanos;
                    seconds = (f.nanos - firstNanos) / 1e9;
                    csv.write(String.format(Locale.US, "%d,%.4f,%.3f,%.3f,%.4f\n", written, seconds,
                            f.pose[0], f.pose[1], f.pose[2]));
                    bytes += jpeg.size();
                    written++;
                }
            } catch (IOException | InterruptedException e) {
                RobotLog.ee("VideoRecorder", e, "Stopped recording " + name);
                problem = "recording stopped: " + e.getMessage();
            } finally {
                for (Frame f : queue) f.image.release();
                close();
            }
        }

        /** Finishes the files and tells Android about them, so they show up over USB without a restart. */
        private void close() {
            try {
                video.close();
                csv.close();
            } catch (IOException e) {
                RobotLog.ee("VideoRecorder", e, "Couldn't finish " + name);
            }
            MediaScannerConnection.scanFile(AppUtil.getDefContext(),
                    new String[]{videoFile.getAbsolutePath(), csvFile.getAbsolutePath()}, null, null);
        }
    }
}
