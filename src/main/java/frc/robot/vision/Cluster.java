package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class Cluster {

    private List<PhotonTrackedTarget> fuel;
    private List<TargetCorner> corners;
    private ArrayList<Integer> ids;
    private double area;
    private double pitch;
    private double yaw;

    public Cluster(List<PhotonTrackedTarget> fuelList, int[] ids) {
        this.ids = new ArrayList<>();
        if (ids != null) {
            for (int id : ids) {
                this.ids.add(id);
            }
        }
        this.fuel = (fuelList != null) ? new ArrayList<>(fuelList) : new ArrayList<>();
        this.corners = this.getClusterCorners();
        this.area = calculateArea();
        this.pitch = getMeanPitch();
        this.yaw = getMeanYaw();
    }

    public Cluster() {
        this(null, new int[]{});
    }

    private void updateCluster() {
        this.corners = this.getClusterCorners();
        this.area = calculateArea();
        this.pitch = getMeanPitch();
        this.yaw = getMeanYaw();
    }

    public void addFuel(PhotonTrackedTarget fuel) {
        this.fuel.add(fuel);
        this.updateCluster();
    }

    public void addFuel(List<PhotonTrackedTarget> fuelList) {
        if (fuelList != null && !fuelList.isEmpty()) {
            this.fuel.addAll(fuelList);
            this.updateCluster();
        }
    }

    public void addIDs(List<Integer> neighbors) {
        for (int i : neighbors) {
            this.ids.add(i);
        }
    }

    private double calculateArea() {
        if (corners == null || corners.size() != 4) {
            return 0.0; // Not a valid rectangle
        }
        double width = Math.abs(corners.get(1).x - corners.get(0).x); // bottom-right x - bottom-left x
        double height = Math.abs(corners.get(3).y - corners.get(0).y); // top-left y - bottom-left y
        return width * height;
    }

    

    private List<TargetCorner> getClusterCorners() {
        if (this.fuel == null || this.fuel.isEmpty())
            return new ArrayList<>();

        double left = Double.POSITIVE_INFINITY, down = Double.POSITIVE_INFINITY;
        double right = Double.NEGATIVE_INFINITY, up = Double.NEGATIVE_INFINITY;

        for (PhotonTrackedTarget t : fuel) {
            if (t == null)
                continue;
            var detected = t.getDetectedCorners();
            if (detected == null)
                continue;
            for (TargetCorner c : detected) {
                if (c == null)
                    continue;
                if (c.x > right)
                    right = c.x;
                if (c.x < left)
                    left = c.x;
                if (c.y < down)
                    down = c.y;
                if (c.y > up)
                    up = c.y;
            }
        }

        // If we never found any corners (all infinities), return empty list
        if (Double.isInfinite(left) || Double.isInfinite(down) || Double.isInfinite(right)
                || Double.isInfinite(up)) {
            return new ArrayList<>();
        }

        List<TargetCorner> out = new ArrayList<>();
        out.add(new TargetCorner(left, down));
        out.add(new TargetCorner(right, down));
        out.add(new TargetCorner(right, up));
        out.add(new TargetCorner(left, up));
        return out;
    }

    /**
     * Returns the corners of the bounding box around this cluster, in the order:
     * bottom-left, bottom-right, top-right, top-left. If no corners are found,
     * returns an empty list.
     */
    public List<TargetCorner> getCorners() {
        return corners;
    }

    public int getSize() {
        return (fuel == null) ? 0 : fuel.size();
    }

    /**
     * Arithmetic mean of the pitch values (degrees). Returns NaN for empty cluster.
     */
    public double getMeanPitch() {
        int n = getSize();
        if (n == 0) return Double.NaN;
        double sum = 0.0;
        for (PhotonTrackedTarget t : fuel) sum += t.getPitch();
        return sum / n;
    }

    /**
     * Area-weighted mean of the pitch values. Falls back to arithmetic mean if all areas are <=0.
     */
    public double getWeightedMeanPitch() {
        double totalW = 0.0, sum = 0.0;
        for (PhotonTrackedTarget t : fuel) {
            double w = Math.max(0.0, t.getArea());
            sum += t.getPitch() * w;
            totalW += w;
        }
        if (totalW == 0.0) return getMeanPitch();
        return sum / totalW;
    }

    /**
     * Circular arithmetic mean of yaw (degrees), handling wrap-around. Returns NaN for empty cluster.
     */
    public double getMeanYaw() {
        int n = getSize();
        if (n == 0) return Double.NaN;
        double sumX = 0.0, sumY = 0.0;
        for (PhotonTrackedTarget t : fuel) {
            double r = Math.toRadians(t.getYaw());
            sumX += Math.cos(r);
            sumY += Math.sin(r);
        }
        return Math.toDegrees(Math.atan2(sumY, sumX));
    }

    /**
     * Area-weighted circular mean of yaw (degrees). Falls back to unweighted mean if all areas <=0.
     */
    public double getWeightedMeanYaw() {
        int n = getSize();
        if (n == 0) return Double.NaN;
        double sumX = 0.0, sumY = 0.0, totalW = 0.0;
        for (PhotonTrackedTarget t : fuel) {
            double w = Math.max(0.0, t.getArea());
            double r = Math.toRadians(t.getYaw());
            sumX += w * Math.cos(r);
            sumY += w * Math.sin(r);
            totalW += w;
        }
        if (totalW == 0.0) return getMeanYaw();
        return Math.toDegrees(Math.atan2(sumY, sumX));
    }

    /**
     * Sum of areas of all targets in the cluster.
     */
    public double getSumArea() {
        double s = 0.0;
        for (PhotonTrackedTarget t : fuel) s += Math.max(0.0, t.getArea());
        return s;
    }

    /**
     * Average area across cluster targets. Returns NaN for empty clusters.
     */
    public double getAverageArea() {
        int n = getSize();
        return (n == 0) ? Double.NaN : getSumArea() / n;
    }

    /**
     * Max area among cluster targets, or 0 for empty cluster.
     */
    public double getMaxArea() {
        double m = 0.0;
        for (PhotonTrackedTarget t : fuel) m = Math.max(m, t.getArea());
        return m;
    }

    /**
     * Defensive copy of the underlying target list.
     */
    public List<PhotonTrackedTarget> getTargets() {
        return (fuel == null) ? List.of() : new ArrayList<>(fuel);
    }

    public ArrayList<Integer> getIDs() {
        return ids;
    }

    public double getArea() {
        return area;
    }

    public double getPitch() {
        return pitch;
    }

    public double getYaw() {
        return yaw;
    }

    public String toString() {
        return String.format("Cluster with %d fuel, area %.2f, pitch, yaw (%.2f, %.2f), IDs: %s",
                getSize(), getArea(), getPitch(), getYaw(), ids.toString());
    }
}