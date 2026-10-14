package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * The 2026 field obstacles this 30 inch tall robot must respect, from the fuel-sim field model and the AprilTag layout.
 * Blue-alliance side listed; red is the mirror in x (the structure) and the 180 degree rotation (tower, outpost).
 */
final class FieldGeometry {
    static final double FIELD_X = 16.54, FIELD_Y = 8.07;
    // robot footprint (settings.json): 0.73 long (x) x 0.978 wide (y), intake stowed
    static final double ROBOT_LENGTH = 0.73, ROBOT_WIDTH = 0.978;

    record Rect(String name, double x0, double y0, double x1, double y1) {
        double distanceTo(double px, double py) {
            double dx = Math.max(Math.max(x0 - px, 0), px - x1);
            double dy = Math.max(Math.max(y0 - py, 0), py - y1);
            return Math.hypot(dx, dy);
        }
        boolean contains(double px, double py) {
            return px >= x0 && px <= x1 && py >= y0 && py <= y1;
        }
    }

    static final Rect[] OBSTACLES = {
        // trench lanes (cannot go under: the robot is 30 in tall) and the hub, blue then red
        new Rect("blue trench (near wall)", 3.96, 0.0, 5.18, 1.57),
        new Rect("blue trench (far wall)", 3.96, FIELD_Y - 1.57, 5.18, FIELD_Y),
        new Rect("blue hub", 3.96, FIELD_Y / 2 - 0.60, 5.18, FIELD_Y / 2 + 0.60),
        new Rect("red trench (near wall)", FIELD_X - 5.18, 0.0, FIELD_X - 3.96, 1.57),
        new Rect("red trench (far wall)", FIELD_X - 5.18, FIELD_Y - 1.57, FIELD_X - 3.96, FIELD_Y),
        new Rect("red hub", FIELD_X - 5.18, FIELD_Y / 2 - 0.60, FIELD_X - 3.96, FIELD_Y / 2 + 0.60),
        // towers on the alliance walls (tags 29-32 and 13-16 put them at y ~ 3.75..4.3), generous footprint
        new Rect("blue tower", 0.0, 3.3, 1.3, 4.65),
        new Rect("red tower", FIELD_X - 1.3, 3.4, FIELD_X, 4.75),
    };

    /** Distance from the robot's footprint to the nearest obstacle (0 or less if they overlap), and which obstacle. */
    record Clearance(double meters, String obstacle) {}

    static Clearance clearance(Pose2d pose) {
        double best = Double.MAX_VALUE;
        String who = "";
        double c = pose.getRotation().getCos(), s = pose.getRotation().getSin();
        // sample the footprint boundary every ~2 cm
        int nx = (int) (ROBOT_LENGTH / 0.02), ny = (int) (ROBOT_WIDTH / 0.02);
        for (int i = 0; i <= nx; i++) {
            for (int j = 0; j <= ny; j++) {
                boolean boundary = i == 0 || i == nx || j == 0 || j == ny;
                if (!boundary) continue;
                double lx = -ROBOT_LENGTH / 2 + i * ROBOT_LENGTH / nx;
                double ly = -ROBOT_WIDTH / 2 + j * ROBOT_WIDTH / ny;
                double px = pose.getX() + lx * c - ly * s;
                double py = pose.getY() + lx * s + ly * c;
                for (Rect r : OBSTACLES) {
                    double d = r.contains(px, py) ? -0.0 : r.distanceTo(px, py);
                    if (r.contains(px, py)) d = -1.0;
                    if (d < best) {
                        best = d;
                        who = r.name();
                    }
                }
            }
        }
        return new Clearance(best, who);
    }

    /** true if the robot's center is in the x-band of a trench/hub structure. */
    static boolean inStructureBand(Pose2d p) {
        return (p.getX() > 3.96 && p.getX() < 5.18) || (p.getX() > FIELD_X - 5.18 && p.getX() < FIELD_X - 3.96);
    }
}
