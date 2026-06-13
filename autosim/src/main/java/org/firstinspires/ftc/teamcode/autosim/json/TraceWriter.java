package org.firstinspires.ftc.teamcode.autosim.json;

import org.firstinspires.ftc.teamcode.autosim.geom.Pose2d;
import org.firstinspires.ftc.teamcode.autosim.geom.Pt;
import org.firstinspires.ftc.teamcode.autosim.model.ActionEvent;
import org.firstinspires.ftc.teamcode.autosim.model.FieldSpec;
import org.firstinspires.ftc.teamcode.autosim.model.Frame;
import org.firstinspires.ftc.teamcode.autosim.model.Meta;
import org.firstinspires.ftc.teamcode.autosim.model.PathSpec;
import org.firstinspires.ftc.teamcode.autosim.model.SimTrace;

import java.util.List;
import java.util.Locale;

/** Minimal, dependency-free JSON serializer for {@link SimTrace}. */
public final class TraceWriter {
    private TraceWriter() {}

    public static String toJson(SimTrace t) {
        StringBuilder b = new StringBuilder();
        b.append("{\n");
        b.append("  \"schemaVersion\": 1,\n");

        Meta m = t.meta;
        b.append("  \"meta\": {\n");
        b.append("    \"autoName\": ").append(str(m.autoName)).append(",\n");
        b.append("    \"alliance\": ").append(str(m.alliance)).append(",\n");
        b.append("    \"simDtMillis\": ").append(m.simDtMillis).append(",\n");
        b.append("    \"totalMillis\": ").append(m.totalMillis).append(",\n");
        b.append("    \"source\": ").append(str(m.source)).append(",\n");
        b.append("    \"generatedAt\": ").append(str(m.generatedAt)).append(",\n");
        b.append("    \"effectProfile\": ").append(str(m.effectProfile)).append(",\n");
        b.append("    \"startPose\": ").append(pose(m.startPose)).append(",\n");
        b.append("    \"follower\": { \"model\": ").append(str(m.followerModel))
                .append(", \"xVel\": ").append(n(m.followerXVel))
                .append(", \"yVel\": ").append(n(m.followerYVel)).append(" }\n");
        b.append("  },\n");

        FieldSpec f = t.field;
        b.append("  \"field\": { \"widthIn\": ").append(n(f.widthIn))
                .append(", \"heightIn\": ").append(n(f.heightIn))
                .append(", \"goal\": ").append(pt(f.goal))
                .append(", \"backgroundImage\": ").append(str(f.backgroundImage)).append(" },\n");

        b.append("  \"paths\": [\n");
        for (int i = 0; i < t.paths.size(); i++) {
            PathSpec p = t.paths.get(i);
            b.append("    { \"id\": ").append(str(p.id))
                    .append(", \"kind\": ").append(str(p.kind))
                    .append(", \"headingDeg\": ").append(n(p.headingDeg))
                    .append(", \"tStartMs\": ").append(p.tStartMs)
                    .append(", \"tEndMs\": ").append(p.tEndMs)
                    .append(", \"controlPoints\": ").append(ptArr(p.controlPoints))
                    .append(", \"polyline\": ").append(ptArr(p.polyline))
                    .append(" }").append(comma(i, t.paths.size())).append("\n");
        }
        b.append("  ],\n");

        b.append("  \"frames\": [\n");
        for (int i = 0; i < t.frames.size(); i++) {
            b.append("    ").append(frame(t.frames.get(i))).append(comma(i, t.frames.size())).append("\n");
        }
        b.append("  ],\n");

        b.append("  \"events\": [\n");
        for (int i = 0; i < t.events.size(); i++) {
            b.append("    ").append(event(t.events.get(i))).append(comma(i, t.events.size())).append("\n");
        }
        b.append("  ]\n");

        b.append("}\n");
        return b.toString();
    }

    private static String comma(int i, int size) {
        return i < size - 1 ? "," : "";
    }

    private static String frame(Frame fr) {
        return "{ \"tMs\": " + fr.tMs
                + ", \"pose\": " + pose(fr.pose)
                + ", \"autoState\": " + str(fr.autoState)
                + ", \"feedState\": " + str(fr.feedState)
                + ", \"followerBusy\": " + fr.followerBusy
                + ", \"flywheelTargetRadS\": " + n(fr.flywheelTargetRadS)
                + ", \"flywheelActualRadS\": " + n(fr.flywheelActualRadS)
                + ", \"turretDeg\": " + n(fr.turretDeg)
                + ", \"hoodDeg\": " + n(fr.hoodDeg)
                + ", \"intakePower\": " + n(fr.intakePower) + " }";
    }

    private static String event(ActionEvent e) {
        return "{ \"tMs\": " + e.tMs
                + ", \"category\": " + str(e.category.name())
                + ", \"label\": " + str(e.label)
                + ", \"durationMs\": " + e.durationMs
                + (e.pose != null ? ", \"pose\": " + pose(e.pose) : "")
                + " }";
    }

    private static String pose(Pose2d p) {
        if (p == null) return "null";
        return "{ \"x\": " + n(p.x) + ", \"y\": " + n(p.y) + ", \"headingDeg\": " + n(p.headingDeg) + " }";
    }

    private static String pt(Pt p) {
        if (p == null) return "null";
        return "{ \"x\": " + n(p.x) + ", \"y\": " + n(p.y) + " }";
    }

    private static String ptArr(List<Pt> pts) {
        StringBuilder b = new StringBuilder("[");
        for (int i = 0; i < pts.size(); i++) {
            b.append(pt(pts.get(i)));
            if (i < pts.size() - 1) b.append(", ");
        }
        return b.append("]").toString();
    }

    private static String str(String s) {
        if (s == null) return "null";
        StringBuilder b = new StringBuilder("\"");
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            switch (c) {
                case '"':  b.append("\\\""); break;
                case '\\': b.append("\\\\"); break;
                case '\n': b.append("\\n"); break;
                case '\r': b.append("\\r"); break;
                case '\t': b.append("\\t"); break;
                default:   b.append(c);
            }
        }
        return b.append("\"").toString();
    }

    /** Compact number: integers without a decimal, otherwise up to 3 trimmed decimals. */
    private static String n(double v) {
        if (Double.isNaN(v) || Double.isInfinite(v)) return "0";
        if (v == Math.rint(v)) return Long.toString((long) v);
        String s = String.format(Locale.US, "%.3f", v);
        int end = s.length();
        while (end > 0 && s.charAt(end - 1) == '0') end--;
        if (end > 0 && s.charAt(end - 1) == '.') end--;
        return s.substring(0, end);
    }
}
