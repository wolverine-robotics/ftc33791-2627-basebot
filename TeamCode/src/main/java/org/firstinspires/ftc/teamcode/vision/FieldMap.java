package org.firstinspires.ftc.teamcode.vision;

import java.util.HashMap;
import java.util.Map;

public final class FieldMap {
    private FieldMap() {}

    public static final class ClusterPose {
        public final double x, y, z, heading;
        public final int[] tagIds;
        public ClusterPose(double x, double y, double z, double heading, int... tagIds) {
            this.x = x; this.y = y; this.z = z; this.heading = heading; this.tagIds = tagIds;
        }
    }

    // BIOBUZZ: Frame 49.46" wide × 38.95" deep, pivot at 43.95" height
    // Two HIVEs (red/blue), each with 2 CELLs, 4 tags per CELL bottom face
    // Tag IDs per manual Figure 9-17:
    // Red far (opposite audience): 30, 31, 32, 33
    // Red audience side: 34, 35, 36, 37
    // Blue audience side: 38, 39, 40, 41
    // Blue far (opposite audience): 42, 43, 44, 45
    // Cluster centers: Red far ~x=72, y=18 | Red aud ~x=72, y=54 | Blue aud ~x=72, y=90 | Blue far ~x=72, y=126
    // Cells ~18.8" apart. Z: UP ~55", DOWN ~20".
    // TODO: Replace with exact CAD / reference-hole measurements for each cluster center.

    public static final Map<Integer, ClusterPose> CLUSTERS = new HashMap<>();
    private static final int[] TAG_TO_CELL = new int[256];

    static {
        for (int i = 0; i < TAG_TO_CELL.length; i++) TAG_TO_CELL[i] = -1;
        add(CLUSTERS, 0, 72.0, 18.0, 55.0, 180, 30, 31, 32, 33);  // Red far (opposite audience)
        add(CLUSTERS, 1, 72.0, 54.0, 20.0, 180, 34, 35, 36, 37);  // Red audience side
        add(CLUSTERS, 2, 72.0, 90.0, 20.0, 0, 38, 39, 40, 41);    // Blue audience side
        add(CLUSTERS, 3, 72.0, 126.0, 55.0, 0, 42, 43, 44, 45);   // Blue far (opposite audience)
    }

    private static void add(Map<Integer, ClusterPose> map, int cellIndex, double x, double y, double z, double heading, int... tagIds) {
        map.put(cellIndex, new ClusterPose(x, y, z, heading, tagIds));
        for (int id : tagIds) TAG_TO_CELL[id] = cellIndex;
    }

    public static ClusterPose getCluster(int cellIndex) {
        return CLUSTERS.get(cellIndex);
    }

    public static int getClusterIndex(int tagId) {
        return (tagId >= 0 && tagId < TAG_TO_CELL.length) ? TAG_TO_CELL[tagId] : -1;
    }

    public static double getGameTagSizeIn() { return 3.25; }
    public static int[] getGameIds() { return new int[]{30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45}; }
}