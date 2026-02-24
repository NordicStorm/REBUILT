package frc.robot.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Lightweight DBSCAN-like clusterer for vision targets. Clusters are formed
 * based on angular
 * proximity in pitch/yaw space and optionally constrained by area similarity.
 *
 * Usage:
 * - Use clusterByAngularDistance(...) for pure angular clustering.
 * - Use clusterByAngularAndArea(...) to require area similarity (useful to
 * avoid grouping targets
 * that are at very different ranges/scales).
 */
public class TargetClusterer {

    public TargetClusterer() {
        /* static utility */ }

    /**
     * DBSCAN-like clustering using angular Euclidean distance between pitch/yaw.
     *
     * @param pts    list of targets
     * @param epsDeg max angular distance in degrees for neighbor relation
     * @param minPts minimum points to form a core (1 => every point is its own
     *               cluster)
     * @return list of clusters (each cluster is a list of targets). Noise points
     *         are returned as
     *         single-member clusters if minPts==1, otherwise they are omitted.
     */
    public static List<List<PhotonTrackedTarget>> clusterByAngularDistance(List<PhotonTrackedTarget> pts, double epsDeg,
            int minPts) {
        return clusterByAngularAndArea(pts, epsDeg, minPts, Double.POSITIVE_INFINITY);
    }

    /**
     * DBSCAN-like clustering using angular distance and an area ratio constraint.
     * A candidate neighbor must satisfy: angular distance <= epsDeg and area ratio
     * within tolerance.
     * The area tolerance is expressed as a multiplicative ratio: e.g.
     * areaTolerance=1.25 allows
     * up to 25% difference (larger/smaller) between areas.
     *
     * @param pts           list of targets
     * @param epsDeg        angular neighborhood radius (degrees)
     * @param minPts        minimum points to form cluster
     * @param areaTolerance multiplicative area tolerance >= 1.0; pass
     *                      Double.POSITIVE_INFINITY to ignore
     * @return clusters as list of lists
     */
    public static List<List<PhotonTrackedTarget>> clusterByAngularAndArea(List<PhotonTrackedTarget> pts, double epsDeg,
            int minPts, double areaTolerance) {
        if (pts == null || pts.isEmpty())
            return Collections.emptyList();
        final int n = pts.size();
        final boolean[] visited = new boolean[n];
        final int[] clusterId = new int[n];
        for (int i = 0; i < n; i++)
            clusterId[i] = -1; // -1 = unassigned

        List<List<PhotonTrackedTarget>> clusters = new ArrayList<>();
        int nextCluster = 0;

        for (int i = 0; i < n; i++) {
            if (visited[i])
                continue;
            visited[i] = true;
            List<Integer> neighbors = regionQuery(pts, i, epsDeg, areaTolerance);
            if (neighbors.size() < minPts) {
                // mark as noise by leaving clusterId as -1 (we'll optionally include later)
                continue;
            }
            // create new cluster
            List<PhotonTrackedTarget> cluster = new ArrayList<>();
            clusters.add(cluster);
            expandCluster(pts, i, neighbors, cluster, visited, clusterId, nextCluster, epsDeg, minPts, areaTolerance);
            nextCluster++;
        }

        // Optionally include noise points as one-member clusters if minPts == 1 or user
        // expects all points
        // For now, include any unassigned points as singletons so caller sees all
        // detections.
        for (int i = 0; i < n; i++) {
            if (clusterId[i] == -1) {
                List<PhotonTrackedTarget> single = new ArrayList<>();
                single.add(pts.get(i));
                clusters.add(single);
            }
        }

        return clusters;
    }

    private static void expandCluster(List<PhotonTrackedTarget> pts, int idx, List<Integer> neighbors,
            List<PhotonTrackedTarget> cluster,
            boolean[] visited, int[] clusterId, int clusterIndex,
            double epsDeg, int minPts, double areaTolerance) {
        // add idx
        cluster.add(pts.get(idx));
        clusterId[idx] = clusterIndex;

        // iterate over neighbors
        for (int i = 0; i < neighbors.size(); i++) {
            int nid = neighbors.get(i);
            if (!visited[nid]) {
                visited[nid] = true;
                List<Integer> nidNeighbors = regionQuery(pts, nid, epsDeg, areaTolerance);
                if (nidNeighbors.size() >= minPts) {
                    // append new neighbors to processing list
                    for (int m : nidNeighbors)
                        if (!neighbors.contains(m))
                            neighbors.add(m);
                }
            }
            if (clusterId[nid] == -1) {
                cluster.add(pts.get(nid));
                clusterId[nid] = clusterIndex;
            }
        }
    }

    private static List<Integer> regionQuery(List<PhotonTrackedTarget> pts, int idx, double epsDeg,
            double areaTolerance) {
        List<Integer> neighbors = new ArrayList<>();
        PhotonTrackedTarget t = pts.get(idx);
        for (int i = 0; i < pts.size(); i++) {
            if (i == idx)
                continue;
            PhotonTrackedTarget o = pts.get(i);
            double d = angularDistanceDeg(t, o);
            if (d <= epsDeg) {
                if (areaTolerance == Double.POSITIVE_INFINITY || areaSimilar(t.area, o.area, areaTolerance)) {
                    neighbors.add(i);
                }
            }
        }
        return neighbors;
    }

    private static boolean areaSimilar(double a, double b, double tol) {
        if (a <= 0 || b <= 0)
            return false; // invalid areas treated as dissimilar
        double ratio = a / b;
        if (ratio < 1.0)
            ratio = 1.0 / ratio; // >=1
        return ratio <= tol;
    }

    private static double angularDistanceDeg(PhotonTrackedTarget a, PhotonTrackedTarget b) {
        double dp = a.pitch - b.pitch;
        double dy = a.yaw - b.yaw;
        return Math.hypot(dp, dy);
    }
}
