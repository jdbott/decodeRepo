package org.firstinspires.ftc.teamcode.autosim;

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
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;

/**
 * Builds a trace for every auto in {@link AutoRegistry}, embeds them all (plus the field image,
 * base64) into the viewer template, and writes one self-contained standalone HTML. The viewer's
 * dropdown switches between the embedded traces — no rebuild needed to compare autos.
 *
 * <p>Paths resolve against {@code -Dautosim.base=<module dir>}. An optional first CLI arg selects
 * which auto is shown first (defaults to the registry's first entry).
 */
public final class AutoSimGenerator {

    public static void main(String[] args) throws Exception {
        Path base = Paths.get(System.getProperty("autosim.base", "")).toAbsolutePath().normalize();
        Path templatePath = base.resolve("viewer/autosim.template.html");
        Path fieldPath = base.resolve("fields/decode.svg");
        Path outPath = base.resolve("dist/autosim.html");

        String generatedAt = Instant.now().toString();
        Map<String, SimTrace> traces = new LinkedHashMap<>();
        for (Map.Entry<String, java.util.function.Supplier<SimTrace>> e : AutoRegistry.AUTOS.entrySet()) {
            SimTrace t = e.getValue().get();
            t.meta.generatedAt = generatedAt;
            traces.put(e.getKey(), t);
        }

        String defaultAuto = (args.length > 0 && traces.containsKey(args[0]))
                ? args[0] : traces.keySet().iterator().next();

        StringBuilder tracesJson = new StringBuilder("{\n");
        int i = 0;
        for (Map.Entry<String, SimTrace> e : traces.entrySet()) {
            tracesJson.append("  ").append(TraceWriter.str(e.getKey())).append(": ")
                    .append(TraceWriter.toJson(e.getValue()));
            tracesJson.append(++i < traces.size() ? "," : "").append("\n");
        }
        tracesJson.append("}");

        String template = new String(Files.readAllBytes(templatePath), StandardCharsets.UTF_8);
        String html = template
                .replace("__TRACES_JSON__", tracesJson.toString())
                .replace("__DEFAULT_AUTO__", defaultAuto)
                .replace("__FIELD_IMAGE_DATAURI__", dataUri(fieldPath))
                .replace("__GENERATED_AT__", generatedAt);

        Files.createDirectories(outPath.getParent());
        Files.write(outPath, html.getBytes(StandardCharsets.UTF_8));

        System.out.println("AutoSim: wrote " + outPath);
        for (Map.Entry<String, SimTrace> e : traces.entrySet()) printSummary(e.getKey(), e.getValue());
        System.out.println("  default=" + defaultAuto + "  — open the file above in any browser.");
    }

    private static void printSummary(String name, SimTrace trace) {
        double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE, maxX = -Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
        for (PathSpec p : trace.paths) {
            for (Pt q : p.polyline) {
                minX = Math.min(minX, q.x); maxX = Math.max(maxX, q.x);
                minY = Math.min(minY, q.y); maxY = Math.max(maxY, q.y);
            }
        }
        boolean inBounds = minX >= 0 && minY >= 0 && maxX <= trace.field.widthIn && maxY <= trace.field.heightIn;
        System.out.printf(Locale.US, "  %-16s paths=%-3d frames=%-5d total=%5dms events=%-3d bbox x[%.1f..%.1f] y[%.1f..%.1f] inBounds=%b%n",
                name, trace.paths.size(), trace.frames.size(), trace.meta.totalMillis, trace.events.size(),
                minX, maxX, minY, maxY, inBounds);
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
