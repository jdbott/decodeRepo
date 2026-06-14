package org.firstinspires.ftc.teamcode.autosim;

import org.firstinspires.ftc.teamcode.autosim.autos.V3FarAutoSim;
import org.firstinspires.ftc.teamcode.autosim.geom.Pt;
import org.firstinspires.ftc.teamcode.autosim.json.TraceWriter;
import org.firstinspires.ftc.teamcode.autosim.model.PathSpec;
import org.firstinspires.ftc.teamcode.autosim.model.SimTrace;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.Instant;
import java.util.Base64;
import java.util.Locale;

/**
 * Builds the trace from a sim copy, injects it (and the field image, base64) into the
 * viewer template, and writes one self-contained standalone HTML.
 *
 * <p>Paths resolve against {@code -Dautosim.base=<module dir>} (defaults to the current
 * working directory). The Gradle {@code autosim} task sets the working dir to this module.
 */
public final class AutoSimGenerator {

    public static void main(String[] args) throws Exception {
        Path base = Paths.get(System.getProperty("autosim.base", "")).toAbsolutePath().normalize();
        Path templatePath = base.resolve("viewer/autosim.template.html");
        Path fieldPath = base.resolve("fields/decode.svg");
        Path outPath = base.resolve("dist/autosim-V3FarAuto.html");

        SimTrace trace = V3FarAutoSim.build();
        trace.meta.generatedAt = Instant.now().toString();
        String json = TraceWriter.toJson(trace);

        String template = new String(Files.readAllBytes(templatePath), StandardCharsets.UTF_8);
        String fieldDataUri = dataUri(fieldPath);

        String html = template
                .replace("__TRACE_JSON__", json)
                .replace("__FIELD_IMAGE_DATAURI__", fieldDataUri)
                .replace("__GENERATED_AT__", trace.meta.generatedAt);

        Files.createDirectories(outPath.getParent());
        Files.write(outPath, html.getBytes(StandardCharsets.UTF_8));

        printSummary(trace, outPath);
    }

    private static void printSummary(SimTrace trace, Path outPath) {
        double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE, maxX = -Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
        int pts = 0;
        for (PathSpec p : trace.paths) {
            for (Pt q : p.polyline) {
                minX = Math.min(minX, q.x);
                maxX = Math.max(maxX, q.x);
                minY = Math.min(minY, q.y);
                maxY = Math.max(maxY, q.y);
                pts++;
            }
        }
        boolean inBounds = minX >= 0 && minY >= 0 && maxX <= trace.field.widthIn && maxY <= trace.field.heightIn;
        String lastState = trace.frames.isEmpty() ? "-" : trace.frames.get(trace.frames.size() - 1).autoState;
        System.out.println("AutoSim: wrote " + outPath);
        System.out.printf(Locale.US, "  paths=%d  polylinePts=%d%n", trace.paths.size(), pts);
        System.out.printf(Locale.US, "  bbox x[%.1f..%.1f] y[%.1f..%.1f]  (field 0..%.0f)  inBounds=%b%n",
                minX, maxX, minY, maxY, trace.field.widthIn, inBounds);
        System.out.printf(Locale.US, "  frames=%d  totalMillis=%d  endState=%s%n",
                trace.frames.size(), trace.meta.totalMillis, lastState);
        System.out.println("  open the file above in any browser.");
    }

    private static String dataUri(Path file) throws Exception {
        if (!Files.exists(file)) {
            System.out.println("WARN: field image not found: " + file + " (rendering grid only)");
            return "";
        }
        byte[] bytes = Files.readAllBytes(file);
        String name = file.getFileName().toString().toLowerCase(Locale.US);
        String mime = name.endsWith(".svg") ? "image/svg+xml"
                : name.endsWith(".png") ? "image/png"
                : (name.endsWith(".jpg") || name.endsWith(".jpeg")) ? "image/jpeg"
                : "application/octet-stream";
        return "data:" + mime + ";base64," + Base64.getEncoder().encodeToString(bytes);
    }

    private AutoSimGenerator() {}
}
