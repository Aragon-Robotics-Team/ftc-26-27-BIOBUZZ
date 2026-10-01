package org.firstinspires.ftc.teamcode.util;

import android.content.Context;

import com.qualcomm.robotcore.util.WebHandlerManager;

import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

import fi.iki.elonen.NanoHTTPD;

/**
 * A "Path logs" page on the Robot Controller web server, for copying {@link PathLog} files to a laptop:
 * on the robot's Wi-Fi, open http://192.168.43.1:8080/pathlogs and click a run to download it.
 */
public final class PathLogsPage {
    private static final String LIST = "/pathlogs";
    private static final String DOWNLOAD = "/pathlogs/download";

    private PathLogsPage() {}

    @WebHandlerRegistrar
    public static void register(Context context, WebHandlerManager manager) {
        manager.register(LIST, session -> NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, "text/html", page()));
        manager.register(DOWNLOAD, session -> {
            List<String> names = session.getParameters().get("name");
            String name = names == null || names.isEmpty() ? "" : names.get(0);
            File file = new File(PathLog.DIR, name);
            // Only plain file names inside the log folder.
            if (!name.matches("[A-Za-z0-9_.\\-]+\\.csv") || !file.isFile()) {
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.NOT_FOUND, NanoHTTPD.MIME_PLAINTEXT, "No such log");
            }
            try {
                NanoHTTPD.Response response = NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, "text/csv",
                        new BufferedInputStream(new FileInputStream(file)), file.length());
                response.addHeader("Content-Disposition", "attachment; filename=\"" + file.getName() + "\"");
                return response;
            } catch (IOException e) {
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.INTERNAL_ERROR, NanoHTTPD.MIME_PLAINTEXT, e.getMessage());
            }
        });
    }

    private static String page() {
        File[] files = PathLog.DIR.listFiles((dir, name) -> name.endsWith(".csv"));
        List<File> logs = files == null ? Collections.<File>emptyList() : Arrays.asList(files);
        Collections.sort(logs, (a, b) -> b.getName().compareTo(a.getName())); // newest first (names start with the time)
        StringBuilder html = new StringBuilder();
        html.append("<!doctype html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width\">")
                .append("<title>Path logs</title><style>")
                .append("body{font:16px/1.5 Roboto,-apple-system,Segoe UI,Arial,sans-serif;margin:0;background:#f3f4f6}")
                .append("header{background:#2563eb;color:#fff;padding:8px 16px;font-size:1.4rem}")
                .append("main{max-width:720px;margin:16px auto;padding:0 16px}")
                .append("table{border-collapse:collapse;width:100%;background:#fff;border-radius:8px}")
                .append("td{padding:6px 10px;border-bottom:1px solid #e5e7eb}td.n{text-align:right;color:#6b7280}")
                .append("p{color:#4b5563}</style></head><body><header>Path logs</header><main>")
                .append("<p>One file per Auto run. Download the runs you want and drop them into the path planner's Logs tab.</p>");
        if (logs.isEmpty()) {
            html.append("<p>No runs logged yet.</p>");
        } else {
            html.append("<table>");
            for (File f : logs) {
                html.append("<tr><td><a href=\"").append(DOWNLOAD).append("?name=").append(f.getName()).append("\" download>")
                        .append(f.getName()).append("</a></td><td class=\"n\">")
                        .append(String.format(Locale.US, "%.0f KB", f.length() / 1024.0)).append("</td></tr>");
            }
            html.append("</table>");
        }
        return html.append("</main></body></html>").toString();
    }
}
