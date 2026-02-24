package frc.robot.vision;

/**
 * Simple data-holder representing a single detection (target) from the vision system.
 * The fields are public and final for lightweight usage.
 */
public class Target {
    public final double pitch; // degrees (or same unit used by camera API)
    public final double yaw;   // degrees
    public final double area;  // relative area (pixels, normalized area, etc.)
    public final int id;       // optional id/source index

    public Target(int id, double pitch, double yaw, double area) {
        this.id = id;
        this.pitch = pitch;
        this.yaw = yaw;
        this.area = area;
    }

    @Override
    public String toString() {
        return String.format("Target{id=%d, pitch=%.3f, yaw=%.3f, area=%.3f}", id, pitch, yaw, area);
    }
}
