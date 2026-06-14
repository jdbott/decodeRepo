package org.firstinspires.ftc.teamcode.autosim.geom;

import java.util.ArrayList;
import java.util.List;

/**
 * Standalone Bezier sampling (De Casteljau). A Bezier curve is fully defined by its
 * control points, so sampling here is mathematically identical to Pedro's geometry — it
 * just carries no Android/Pedro dependency, so the sim core runs on any plain JVM.
 */
public final class Bezier {
    private Bezier() {}

    /** Sample a Bezier of any degree into {@code segments + 1} evenly-parameterized points. */
    public static List<Pt> sample(List<Pt> control, int segments) {
        List<Pt> out = new ArrayList<>(segments + 1);
        for (int i = 0; i <= segments; i++) {
            out.add(at(control, (double) i / segments));
        }
        return out;
    }

    /** Evaluate the curve at parameter t in [0, 1]. */
    public static Pt at(List<Pt> control, double t) {
        int n = control.size();
        double[] xs = new double[n], ys = new double[n];
        for (int i = 0; i < n; i++) {
            xs[i] = control.get(i).x;
            ys[i] = control.get(i).y;
        }
        for (int k = 1; k < n; k++) {
            for (int i = 0; i < n - k; i++) {
                xs[i] = xs[i] * (1 - t) + xs[i + 1] * t;
                ys[i] = ys[i] * (1 - t) + ys[i + 1] * t;
            }
        }
        return new Pt(xs[0], ys[0]);
    }
}
