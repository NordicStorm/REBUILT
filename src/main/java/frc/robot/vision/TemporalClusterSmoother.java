package frc.robot.vision;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Tracks clusters across frames and applies exponential smoothing to reduce jitter.
 * Matching is done greedily by nearest angular distance (pitch/yaw) with an optional
 * area-similarity check.
 */
public class TemporalClusterSmoother {

    private final double alpha; // smoothing factor, 0..1 (1 => no smoothing)
    private final double matchThresholdDeg; // max angular distance to consider a match
    private final double areaTolerance; // multiplicative tolerance for area similarity
    private final int maxMissedFrames; // frames to keep a track after it stops matching

    private final AtomicInteger nextId = new AtomicInteger(1);
    private final List<Track> tracks = new ArrayList<>();

    public TemporalClusterSmoother(double alpha, double matchThresholdDeg, double areaTolerance, int maxMissedFrames) {
        if (alpha < 0 || alpha > 1) throw new IllegalArgumentException("alpha must be in [0,1]");
        this.alpha = alpha;
        this.matchThresholdDeg = matchThresholdDeg;
        this.areaTolerance = Math.max(1.0, areaTolerance);
        this.maxMissedFrames = Math.max(1, maxMissedFrames);
    }

    /**
     * A lightweight representation of the smoothed cluster returned to callers.
     */
    public static class SmoothedCluster {
        public final int id;
        public final double pitch; // degrees
        public final double yaw;   // degrees
        public final double area;
        public final List<Integer> ids; // source detection ids that contributed most recently

        public SmoothedCluster(int id, double pitch, double yaw, double area, List<Integer> ids) {
            this.id = id;
            this.pitch = pitch;
            this.yaw = yaw;
            this.area = area;
            this.ids = (ids == null) ? List.of() : new ArrayList<>(ids);
        }
    }

    private static class Track {
        int id;
        double pitch; // degrees (smoothed)
        double yaw;   // degrees (smoothed)
        double area;  // smoothed area
        int missedFrames;
        List<Integer> ids = new ArrayList<>();
    }

    /**
     * Update the smoother with clusters detected in the current frame.
     * Returns the list of smoothed clusters after matching/updating.
     */
    public List<SmoothedCluster> update(List<Cluster> clusters) {
        if (clusters == null) clusters = List.of();

        int n = clusters.size();
        boolean[] clusterAssigned = new boolean[n];
        Set<Track> matchedTracks = new HashSet<>();

        // Precompute measurements
        double[] measPitch = new double[n];
        double[] measYaw = new double[n];
        double[] measArea = new double[n];
        List<List<Integer>> measIDs = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            Cluster c = clusters.get(i);
            double p = c.getWeightedMeanPitch();
            double y = c.getWeightedMeanYaw();
            double a = c.getAverageArea();
            if (Double.isNaN(p)) p = c.getMeanPitch();
            if (Double.isNaN(y)) y = c.getMeanYaw();
            if (Double.isNaN(a)) a = 0.0;
            measPitch[i] = p;
            measYaw[i] = y;
            measArea[i] = a;
            measIDs.add(new ArrayList<>(c.getIDs()));
        }

        // Greedy matching: for each cluster, find the nearest existing track
        boolean[] trackTaken = new boolean[tracks.size()];
        for (int ci = 0; ci < n; ci++) {
            double bestD = Double.POSITIVE_INFINITY;
            int bestT = -1;
            for (int t = 0; t < tracks.size(); t++) {
                if (trackTaken[t]) continue;
                Track tr = tracks.get(t);
                double d = angularDistanceDeg(measPitch[ci], measYaw[ci], tr.pitch, tr.yaw);
                if (d < bestD) {
                    bestD = d;
                    bestT = t;
                }
            }
            if (bestT != -1 && bestD <= matchThresholdDeg && areaSimilar(measArea[ci], tracks.get(bestT).area, areaTolerance)) {
                // match
                Track tr = tracks.get(bestT);
                // update smoothed values using exponential smoothing
                tr.pitch = alpha * measPitch[ci] + (1.0 - alpha) * tr.pitch;
                // yaw smoothing on unit-vector components to avoid wrap issues
                double prevRadX = Math.cos(Math.toRadians(tr.yaw));
                double prevRadY = Math.sin(Math.toRadians(tr.yaw));
                double measRadX = Math.cos(Math.toRadians(measYaw[ci]));
                double measRadY = Math.sin(Math.toRadians(measYaw[ci]));
                double newX = alpha * measRadX + (1.0 - alpha) * prevRadX;
                double newY = alpha * measRadY + (1.0 - alpha) * prevRadY;
                tr.yaw = Math.toDegrees(Math.atan2(newY, newX));
                tr.area = alpha * measArea[ci] + (1.0 - alpha) * tr.area;
                tr.missedFrames = 0;
                tr.ids = measIDs.get(ci);
                clusterAssigned[ci] = true;
                trackTaken[bestT] = true;
                matchedTracks.add(tr);
            }
        }

        // Create new tracks for unmatched clusters
        for (int ci = 0; ci < n; ci++) {
            if (clusterAssigned[ci]) continue;
            Track tr = new Track();
            tr.id = nextId.getAndIncrement();
            tr.pitch = measPitch[ci];
            tr.yaw = measYaw[ci];
            tr.area = measArea[ci];
            tr.missedFrames = 0;
            tr.ids = measIDs.get(ci);
            tracks.add(tr);
            matchedTracks.add(tr);
        }

        // Increment missedFrames for unmatched tracks and remove stale ones
        List<Track> toRemove = new ArrayList<>();
        for (Track tr : tracks) {
            if (!matchedTracks.contains(tr)) {
                tr.missedFrames++;
                if (tr.missedFrames > maxMissedFrames) toRemove.add(tr);
            }
        }
        tracks.removeAll(toRemove);

        // Build result list from active tracks
        List<SmoothedCluster> out = new ArrayList<>();
        for (Track tr : tracks) {
            out.add(new SmoothedCluster(tr.id, tr.pitch, tr.yaw, tr.area, tr.ids));
        }
        return out;
    }

    // Helper: minimal angular/bearing distance combining pitch (linear) and yaw (circular)
    private static double angularDistanceDeg(double pitchA, double yawA, double pitchB, double yawB) {
        double dp = pitchA - pitchB;
        double dy = angleDiffDegrees(yawA, yawB);
        return Math.hypot(dp, dy);
    }

    private static double angleDiffDegrees(double a, double b) {
        double rad = Math.toRadians(a - b);
        double diff = Math.toDegrees(Math.atan2(Math.sin(rad), Math.cos(rad)));
        return diff;
    }

    private static boolean areaSimilar(double a, double b, double tol) {
        if (a <= 0 || b <= 0) return false;
        double ratio = a / b;
        if (ratio < 1.0) ratio = 1.0 / ratio;
        return ratio <= tol;
    }
}
