package org.firstinspires.ftc.teamcode.keyboard;

import android.content.Context;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.robocol.RobocolDatagram;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.WebHandlerManager;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.network.NetworkConnectionHandler;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes;
import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;
import org.firstinspires.ftc.robotcore.internal.webserver.WebHandler;
import org.firstinspires.ftc.robotcore.internal.webserver.websockets.FtcWebSocket;
import org.firstinspires.ftc.robotcore.internal.webserver.websockets.FtcWebSocketMessage;
import org.firstinspires.ftc.robotcore.internal.webserver.websockets.WebSocketNamespaceHandler;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.ByteArrayOutputStream;
import java.io.Closeable;
import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.Field;
import java.net.InetAddress;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Set;
import java.util.concurrent.CopyOnWriteArraySet;
import java.util.concurrent.Executors;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import fi.iki.elonen.NanoHTTPD;

/**
 * Drive the robot from a laptop keyboard with no Driver Station, for practice and testing:
 * open http://192.168.43.1:8080/keyboard on a laptop joined to the robot's Wi-Fi (doc/keyboard-drive.md).
 *
 * The page (assets/keyboard/index.html) sends both gamepads every 20 ms over the Robot Controller's own WebSocket.
 * Each message goes into the SDK exactly where Driver Station gamepad packets arrive, so every OpMode reads the
 * keyboard as gamepad1 / gamepad2, edge detection (xWasPressed() ...) included, with no OpMode changes. The page also
 * inits, starts and stops OpModes, and shows their telemetry ({@link MirrorTelemetry}).
 *
 * Safety:
 *   no input for {@link ControlLease#WATCHDOG_MS}: both gamepads go to rest (the SDK would hold the last packet forever)
 *   one page drives at a time; the others watch and can take over
 *   Esc on any page stops the OpMode
 *   once a real Driver Station connects, the bridge stays off until the app restarts, so it can't act at an event
 */
public final class KeyboardBridge implements OpModeManagerNotifier.Notifications, Closeable {
    private static final String TAG = "KeyboardBridge";
    /** Gamepad id the bridge stamps on its packets ("KEYB"), so OpModes can tell; see {@link #isKeyboard}. */
    private static final int GAMEPAD_ID = 0x4B455942;
    private static final String NAMESPACE = "keyboard";
    private static final long TICK_MS = 50;
    private static final long STATUS_MS = 250;

    // Kept in OpMode.blackboard, which every class loader shares: Sloth hot-reloads TeamCode into a new class loader
    // and may run the SDK hooks again, and there must never be two bridges feeding the gamepads.
    private static final String BRIDGE_KEY = "KeyboardBridge.bridge";
    private static final String CONTEXT_KEY = "KeyboardBridge.context";
    private static final String WEB_KEY = "KeyboardBridge.webHandlerManager";
    private static final String EVENT_LOOP_KEY = "KeyboardBridge.eventLoop";
    private static final String DRIVER_STATION_KEY = "KeyboardBridge.driverStationSeen";

    private static Field eventLoopManagerField;

    private final ControlLease<FtcWebSocket> lease = new ControlLease<>();
    private final Set<FtcWebSocket> pages = new CopyOnWriteArraySet<>();
    private final Gamepad scratch1 = new Gamepad();
    private final Gamepad scratch2 = new Gamepad();
    private final ScheduledExecutorService watchdog = Executors.newSingleThreadScheduledExecutor(r -> {
        Thread thread = new Thread(r, TAG);
        thread.setDaemon(true);
        return thread;
    });

    private Object attachedWeb;
    private volatile OpModeManagerImpl opModeManager;
    private volatile boolean driverStationSeen;
    private volatile boolean closed;
    private volatile boolean reflectionFailed;
    private volatile boolean injected;
    /** The OpMode's lifecycle as the page shows it: idle, init, running or stopped. */
    private volatile String phase = "idle";
    private volatile List<String> telemetryLines = Collections.emptyList();
    private volatile Telemetry.DisplayFormat displayFormat = Telemetry.DisplayFormat.CLASSIC;
    private volatile long lastStatusMs;

    private final MirrorTelemetry.Sink telemetrySink = new MirrorTelemetry.Sink() {
        @Override
        public void telemetry(List<String> lines) {
            telemetryLines = lines;
            post(() -> {
                for (FtcWebSocket page : pages) sendTelemetry(page);
            });
        }

        @Override
        public void displayFormat(Telemetry.DisplayFormat format) {
            displayFormat = format;
        }
    };

    /** True if this gamepad's state came from the keyboard page rather than a Driver Station. */
    public static boolean isKeyboard(Gamepad gamepad) {
        return gamepad.getGamepadId() == GAMEPAD_ID;
    }

    @WebHandlerRegistrar
    public static void attachWebServer(Context context, WebHandlerManager manager) {
        synchronized (OpMode.blackboard) {
            OpMode.blackboard.put(CONTEXT_KEY, context);
            OpMode.blackboard.put(WEB_KEY, manager);
            current().attach();
        }
    }

    @OnCreateEventLoop
    public static void attachEventLoop(Context context, FtcEventLoop eventLoop) {
        synchronized (OpMode.blackboard) {
            OpMode.blackboard.put(EVENT_LOOP_KEY, eventLoop);
            current().attach();
        }
    }

    /** This class loader's bridge. One left by an older class loader (before a Sloth hot reload) is shut down. */
    private static KeyboardBridge current() {
        Object existing = OpMode.blackboard.get(BRIDGE_KEY);
        if (existing instanceof KeyboardBridge) return (KeyboardBridge) existing;
        if (existing instanceof Closeable) {
            try {
                ((Closeable) existing).close();
            } catch (IOException e) {
                RobotLog.ee(TAG, e, "couldn't close the previous bridge");
            }
        }
        KeyboardBridge bridge = new KeyboardBridge(Boolean.TRUE.equals(OpMode.blackboard.get(DRIVER_STATION_KEY)));
        OpMode.blackboard.put(BRIDGE_KEY, bridge);
        return bridge;
    }

    private KeyboardBridge(boolean driverStationSeen) {
        this.driverStationSeen = driverStationSeen;
        watchdog.scheduleAtFixedRate(this::tick, TICK_MS, TICK_MS, TimeUnit.MILLISECONDS);
    }

    /** Registers with whatever the SDK hooks have handed over so far. Safe to repeat. */
    private void attach() {
        Object context = OpMode.blackboard.get(CONTEXT_KEY);
        Object web = OpMode.blackboard.get(WEB_KEY);
        if (context instanceof Context && web instanceof WebHandlerManager && web != attachedWeb) {
            registerWeb((Context) context, (WebHandlerManager) web);
            attachedWeb = web;
        }
        Object eventLoop = OpMode.blackboard.get(EVENT_LOOP_KEY);
        if (eventLoop instanceof FtcEventLoop) {
            OpModeManagerImpl manager = ((FtcEventLoop) eventLoop).getOpModeManager();
            if (manager != opModeManager) {
                if (opModeManager != null) opModeManager.unregisterListener(this);
                manager.registerListener(this); // held weakly; the blackboard keeps this bridge alive
                opModeManager = manager;
            }
        }
    }

    private void registerWeb(Context context, WebHandlerManager web) {
        WebHandler page = session -> asset(context, "keyboard/index.html", "text/html; charset=utf-8");
        web.register("/keyboard", page);
        web.register("/keyboard/", page);
        web.register("/keyboard/labels.json",
                session -> asset(context, "keyboard/control-labels.json", "application/json; charset=utf-8"));
        web.getWebServer().getWebSocketManager().registerNamespaceHandler(new Pages());
    }

    private static NanoHTTPD.Response asset(Context context, String path, String mimeType) throws IOException {
        try (InputStream in = context.getAssets().open(path)) {
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            byte[] buffer = new byte[8192];
            for (int n; (n = in.read(buffer)) > 0; ) out.write(buffer, 0, n);
            NanoHTTPD.Response response =
                    NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, mimeType, out.toString("UTF-8"));
            response.addHeader("Cache-Control", "no-store"); // a new deploy shows up on reload
            return response;
        }
    }

    private boolean enabled() {
        return !closed && !driverStationSeen;
    }

    @Override
    public void close() {
        closed = true;
        watchdog.shutdownNow();
        OpModeManagerImpl manager = opModeManager;
        if (manager != null) manager.unregisterListener(this);
        if (lease.holder() != null) restPads();
        lease.clear();
    }

    // ---- Watchdog ----

    private void tick() {
        try {
            if (closed) return;
            if (!driverStationSeen && NetworkConnectionHandler.getInstance().isPeerConnected()) {
                driverStationSeen = true;
                synchronized (OpMode.blackboard) {
                    OpMode.blackboard.put(DRIVER_STATION_KEY, true);
                }
                RobotLog.ii(TAG, "Driver Station connected: keyboard driving is off until the app restarts");
                // A Driver Station without gamepads sends none, so the keyboard's last state would stay
                if (injected) restPads();
                lease.clear();
                sendStatus();
            }
            FtcWebSocket before = lease.holder();
            if (lease.tick(nowMs())) restPads();
            if (lease.holder() != before || nowMs() - lastStatusMs >= STATUS_MS) sendStatus();
        } catch (RuntimeException e) {
            RobotLog.ee(TAG, e, "watchdog");
        }
    }

    // ---- Gamepads ----

    private void restPads() {
        sendPads(PadState.REST, PadState.REST);
    }

    /** Hands both gamepads to the SDK the way a Driver Station packet arrives. */
    private void sendPads(PadState pad1, PadState pad2) {
        EventLoopManager manager = eventLoopManager();
        if (manager == null) return;
        synchronized (scratch1) {
            try {
                manager.gamepadEvent(datagram(scratch1, pad1, GamepadUser.ONE));
                manager.gamepadEvent(datagram(scratch2, pad2, GamepadUser.TWO));
                injected = true;
            } catch (RobotCoreException | RuntimeException e) {
                RobotLog.ee(TAG, e, "couldn't hand the gamepads to the SDK");
            }
        }
    }

    private static RobocolDatagram datagram(Gamepad gamepad, PadState pad, GamepadUser user) throws RobotCoreException {
        gamepad.left_stick_x = pad.leftStickX;
        gamepad.left_stick_y = pad.leftStickY;
        gamepad.right_stick_x = pad.rightStickX;
        gamepad.right_stick_y = pad.rightStickY;
        gamepad.left_trigger = pad.leftTrigger;
        gamepad.right_trigger = pad.rightTrigger;
        // Only the Xbox names travel in a packet; the SDK fills in cross, circle, share ... from them
        gamepad.a = pad.pressed(PadState.Button.A);
        gamepad.b = pad.pressed(PadState.Button.B);
        gamepad.x = pad.pressed(PadState.Button.X);
        gamepad.y = pad.pressed(PadState.Button.Y);
        gamepad.guide = pad.pressed(PadState.Button.GUIDE);
        gamepad.start = pad.pressed(PadState.Button.START);
        gamepad.back = pad.pressed(PadState.Button.BACK);
        gamepad.left_bumper = pad.pressed(PadState.Button.LEFT_BUMPER);
        gamepad.right_bumper = pad.pressed(PadState.Button.RIGHT_BUMPER);
        gamepad.dpad_up = pad.pressed(PadState.Button.DPAD_UP);
        gamepad.dpad_down = pad.pressed(PadState.Button.DPAD_DOWN);
        gamepad.dpad_left = pad.pressed(PadState.Button.DPAD_LEFT);
        gamepad.dpad_right = pad.pressed(PadState.Button.DPAD_RIGHT);
        gamepad.left_stick_button = pad.pressed(PadState.Button.LEFT_STICK_BUTTON);
        gamepad.right_stick_button = pad.pressed(PadState.Button.RIGHT_STICK_BUTTON);
        gamepad.setUser(user);
        gamepad.setGamepadId(GAMEPAD_ID);
        gamepad.refreshTimestamp();
        return new RobocolDatagram(gamepad, InetAddress.getLoopbackAddress());
    }

    /** Where Driver Station packets arrive. The SDK keeps it in a protected field with no getter. */
    private EventLoopManager eventLoopManager() {
        OpModeManagerImpl manager = opModeManager;
        if (manager == null || reflectionFailed) return null;
        try {
            if (eventLoopManagerField == null) {
                Field field = OpModeManagerImpl.class.getDeclaredField("eventLoopManager");
                field.setAccessible(true);
                eventLoopManagerField = field;
            }
            return (EventLoopManager) eventLoopManagerField.get(manager);
        } catch (ReflectiveOperationException | RuntimeException e) {
            reflectionFailed = true; // log once, not 50 times a second
            RobotLog.ee(TAG, e, "can't reach the EventLoopManager; keyboard driving doesn't work with this SDK");
            return null;
        }
    }

    // ---- OpMode lifecycle ----

    private static boolean isStopRobot(OpModeManagerImpl manager) {
        return manager == null || OpModeManager.DEFAULT_OP_MODE_NAME.equals(manager.getActiveOpModeName());
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {
        if (closed) return;
        if (isStopRobot(opModeManager)) {
            phase = "idle"; // after a stop; the last telemetry stays up, as on the Driver Station
        } else {
            phase = "init";
            telemetryLines = Collections.emptyList();
            displayFormat = Telemetry.DisplayFormat.CLASSIC;
            if (enabled()) opMode.telemetry = new MirrorTelemetry(opMode, telemetrySink);
            post(() -> {
                for (FtcWebSocket page : pages) sendTelemetry(page);
            });
        }
        post(this::sendStatus);
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {
        if (closed || isStopRobot(opModeManager)) return;
        phase = "running";
        post(this::sendStatus);
    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        if (closed || isStopRobot(opModeManager)) return;
        phase = "stopped";
        post(this::sendStatus);
    }

    // ---- Pages ----

    private final class Pages extends WebSocketNamespaceHandler {
        Pages() {
            super(NAMESPACE);
        }

        @Override
        public void onSubscribe(FtcWebSocket page) {
            pages.add(page);
            sendOpModes(page);
            sendTelemetry(page);
            sendStatus();
        }

        @Override
        public void onUnsubscribe(FtcWebSocket page) {
            pages.remove(page);
            if (lease.release(page)) restPads();
            sendStatus();
        }

        @Override
        public boolean onMessage(FtcWebSocketMessage message, FtcWebSocket page) {
            handle(message.getType(), message.getPayload(), page);
            return true;
        }
    }

    private void handle(String type, String payload, FtcWebSocket page) {
        if (!enabled()) return;
        FtcWebSocket before = lease.holder();
        OpModeManagerImpl manager = opModeManager;
        switch (type) {
            case "input":
                PadState[] pads;
                try {
                    pads = PadState.parsePair(payload);
                } catch (IllegalArgumentException e) {
                    RobotLog.ww(TAG, "ignoring bad input: %s", e.getMessage());
                    return;
                }
                if (lease.input(page, nowMs())) sendPads(pads[0], pads[1]);
                break;
            case "takeOver":
                lease.takeOver(page, nowMs());
                restPads();
                break;
            case "stopRobot": // Esc, from any page
                restPads();
                if (manager != null) manager.initOpMode(OpModeManager.DEFAULT_OP_MODE_NAME);
                break;
            case "init":
                if (manager != null && lease.holder() == page && isListed(payload)) manager.initOpMode(payload);
                break;
            case "start":
                if (manager != null && lease.holder() == page && "init".equals(phase)) manager.startActiveOpMode();
                break;
            case "stop":
                if (manager != null && lease.holder() == page) manager.stopActiveOpMode();
                break;
            default:
                RobotLog.ww(TAG, "unknown message type %s", type);
        }
        if (lease.holder() != before) sendStatus();
    }

    private static List<OpModeMeta> listedOpModes() {
        List<OpModeMeta> listed = new ArrayList<>();
        for (OpModeMeta meta : RegisteredOpModes.getInstance().getOpModes()) {
            if (meta.flavor == OpModeMeta.Flavor.SYSTEM || OpModeMeta.isSystemName(meta.name)) continue;
            listed.add(meta);
        }
        return listed;
    }

    private static boolean isListed(String name) {
        for (OpModeMeta meta : listedOpModes()) if (meta.name.equals(name)) return true;
        return false;
    }

    /** Runs on the watchdog thread, so the OpMode and event loop threads never wait on the network. */
    private void post(Runnable task) {
        try {
            watchdog.execute(task);
        } catch (RejectedExecutionException e) {
            // closed
        }
    }

    private void sendOpModes(FtcWebSocket page) {
        JSONArray list = new JSONArray();
        try {
            for (OpModeMeta meta : listedOpModes()) {
                list.put(new JSONObject()
                        .put("name", meta.name)
                        .put("group", meta.group)
                        .put("flavor", meta.flavor.name()));
            }
        } catch (JSONException e) {
            throw new IllegalStateException(e);
        }
        send(page, "opModes", list.toString());
    }

    private void sendTelemetry(FtcWebSocket page) {
        try {
            send(page, "telemetry", new JSONObject()
                    .put("format", displayFormat.name())
                    .put("lines", new JSONArray(telemetryLines))
                    .toString());
        } catch (JSONException e) {
            throw new IllegalStateException(e);
        }
    }

    private void sendStatus() {
        lastStatusMs = nowMs();
        FtcWebSocket holder = lease.holder();
        OpModeManagerImpl manager = opModeManager;
        String opMode = isStopRobot(manager) ? null : manager.getActiveOpModeName();
        boolean connected = eventLoopManager() != null;
        for (FtcWebSocket page : pages) {
            try {
                send(page, "status", new JSONObject()
                        .put("enabled", enabled())
                        .put("driverStation", driverStationSeen)
                        .put("connectedToSdk", connected)
                        .put("controller", holder == null ? JSONObject.NULL : address(holder))
                        .put("you", holder == page)
                        .put("yourAddress", address(page))
                        .put("opMode", opMode == null ? JSONObject.NULL : opMode)
                        .put("phase", phase)
                        .toString());
            } catch (JSONException e) {
                throw new IllegalStateException(e);
            }
        }
    }

    private static String address(FtcWebSocket page) {
        InetAddress address = page.getRemoteIpAddress();
        return address == null ? "?" : address.getHostAddress();
    }

    private void send(FtcWebSocket page, String type, String payload) {
        if (!page.isOpen()) return;
        try {
            page.send(new FtcWebSocketMessage(NAMESPACE, type, payload));
        } catch (RuntimeException e) {
            RobotLog.ww(TAG, "couldn't send %s to %s: %s", type, address(page), e.getMessage());
        }
    }

    private static long nowMs() {
        return System.nanoTime() / 1_000_000;
    }
}
