package org.firstinspires.ftc.teamcode.util;

import android.content.Context;

import com.qualcomm.robotcore.util.WebHandlerManager;

import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import org.firstinspires.ftc.teamcode.core.ShotTable;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;

import fi.iki.elonen.NanoHTTPD;

/**
 * The robot's shot table on the Robot Controller web server, for reading it without USB: on the robot's Wi-Fi,
 * http://192.168.43.1:8080/shottable.csv. 404 when the robot has no table of its own yet.
 */
public final class ShotTablePage {
    private static final String PATH = "/shottable.csv";

    private ShotTablePage() {}

    @WebHandlerRegistrar
    public static void register(Context context, WebHandlerManager manager) {
        manager.register(PATH, session -> {
            File file = ShotTable.file();
            if (!file.isFile()) {
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.NOT_FOUND, NanoHTTPD.MIME_PLAINTEXT,
                        "No shot table on the robot yet: save one with the Shot Table Calibration OpMode");
            }
            try {
                NanoHTTPD.Response response = NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, "text/csv",
                        new BufferedInputStream(new FileInputStream(file)), file.length());
                response.addHeader("Content-Disposition", "inline; filename=\"" + ShotTable.FILE_NAME + "\"");
                response.addHeader("Cache-Control", "no-store");
                return response;
            } catch (IOException e) {
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.INTERNAL_ERROR,
                        NanoHTTPD.MIME_PLAINTEXT, e.getMessage());
            }
        });
    }
}
