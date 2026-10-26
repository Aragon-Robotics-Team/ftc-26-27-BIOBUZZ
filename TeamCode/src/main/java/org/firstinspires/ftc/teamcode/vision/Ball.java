package org.firstinspires.ftc.teamcode.vision;

/**
 * One ball the {@link BallPipeline} found in a frame, in pixels of the frame it was given. Plain data: where it is on
 * the field comes from {@link BallGeometry}.
 */
public class Ball {
    /** The three BIOBUZZ scoring elements. Diameters from the AndyMark product pages. */
    public enum Type {
        POLLEN(2.8),
        RED_NECTAR(3.6),
        BLUE_NECTAR(3.6);

        /** Inches. */
        public final double diameter;

        Type(double diameter) {
            this.diameter = diameter;
        }
    }

    public final Type type;
    public final double x, y;   // center, pixels from the top-left corner
    public final double radius; // pixels
    /** Partly outside the frame; the circle was fitted to the part that's visible. */
    public final boolean truncated;
    /** Merged with same-color structure (e.g. NECTAR on alliance tape); the circle is the largest one inside the blob. */
    public final boolean recovered;

    public Ball(Type type, double x, double y, double radius, boolean truncated, boolean recovered) {
        this.type = type;
        this.x = x;
        this.y = y;
        this.radius = radius;
        this.truncated = truncated;
        this.recovered = recovered;
    }

    public Ball(Type type, double x, double y, double radius) {
        this(type, x, y, radius, false, false);
    }

    @Override
    public String toString() {
        return String.format("%s r=%.0fpx at (%.0f,%.0f)%s%s", type, radius, x, y,
                truncated ? " truncated" : "", recovered ? " recovered" : "");
    }
}
