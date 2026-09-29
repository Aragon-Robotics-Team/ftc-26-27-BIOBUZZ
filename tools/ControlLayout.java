import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.GraphicsEnvironment;
import java.awt.RenderingHints;
import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Path2D;
import java.awt.geom.Rectangle2D;
import java.awt.geom.RoundRectangle2D;
import java.awt.image.BufferedImage;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.MessageDigest;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashSet;
import java.util.HexFormat;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import java.util.zip.CRC32;
import javax.imageio.ImageIO;

/**
 * Draws doc/controls/&lt;opmode&gt;.png: a gamepad diagram for every OpMode, read straight from the Java source.
 *
 * <pre>
 *     java tools/ControlLayout.java            (or ./gradlew :TeamCode:controlLayout)
 *     java tools/ControlLayout.java --list     every binding with its code location
 * </pre>
 *
 * Every build runs this too (see TeamCode/build.gradle), so the diagrams follow the code on their own.
 * It only needs the JDK that Gradle already runs on.
 *
 * <p>How it works
 * <ul>
 *   <li>Finds each @TeleOp / @Autonomous class under TeamCode (skipping @Disabled ones).
 *   <li>Picks up every gamepad1 / gamepad2 read: xWasPressed(), xWasReleased(), plain fields (a, left_bumper,
 *       left_trigger, left_stick_y, ...), and helper methods that take a Gamepad (e.g. MatchState.selectAlliance).
 *   <li>Reads in init() / init_loop() (or before waitForStart()) go under "Init"; everything else is "Match".
 *   <li>Each binding gets an automatic label from the code (schedule(robot.intake.toggleIntake()) -> "toggle intake").
 * </ul>
 *
 * <p>Better labels go in tools/control-labels.json (auto labels show in italics until they have one):
 * <pre>
 *     {
 *       "*":      { "init.gamepad1.square": "Blue alliance" },        &lt;- applies to every OpMode
 *       "TeleOp": { "gamepad1": "Driver",                             &lt;- the gamepad's role
 *                   "gamepad1.triangle": "Toggle intake",
 *                   "gamepad1.left_stick": "Drive (field-centric)" }
 *     }
 * </pre>
 * Keys are [init.]gamepad&lt;N&gt;.&lt;control&gt;. Controls use PlayStation names: cross circle square triangle share
 * options ps touchpad left_bumper right_bumper left_trigger right_trigger dpad_up dpad_down dpad_left
 * dpad_right left_stick right_stick left_stick_button right_stick_button. The tool prints any binding
 * that still needs a label and any label that no longer matches a binding.
 *
 * <p>Each PNG stores a fingerprint of what it shows, so an image is only redrawn when its bindings, labels
 * or this file change. That keeps git quiet when teammates with different fonts build.
 */
public class ControlLayout {

    static final String SOURCE_DIR = "TeamCode/src/main/java";
    static final String PACKAGE_DIR = SOURCE_DIR + "/org/firstinspires/ftc/teamcode";
    static final String LABELS_FILE = "tools/control-labels.json";
    static final String THIS_FILE = "tools/ControlLayout.java";
    static final String OUTPUT_DIR = "doc/controls";
    static final String FINGERPRINT_KEY = "control-layout";

    // SDK name -> control on the diagram (PlayStation names; the SDK treats a/cross etc. as the same button)
    static final Map<String, String> ALIASES = Map.ofEntries(
            Map.entry("a", "cross"), Map.entry("b", "circle"), Map.entry("x", "square"), Map.entry("y", "triangle"),
            Map.entry("back", "share"), Map.entry("start", "options"), Map.entry("guide", "ps"),
            Map.entry("left_trigger_pressed", "left_trigger"), Map.entry("right_trigger_pressed", "right_trigger"),
            Map.entry("left_stick_x", "left_stick"), Map.entry("left_stick_y", "left_stick"),
            Map.entry("right_stick_x", "right_stick"), Map.entry("right_stick_y", "right_stick"),
            Map.entry("touchpad_finger_1", "touchpad"), Map.entry("touchpad_finger_2", "touchpad"),
            Map.entry("touchpad_finger_1_x", "touchpad"), Map.entry("touchpad_finger_1_y", "touchpad"),
            Map.entry("touchpad_finger_2_x", "touchpad"), Map.entry("touchpad_finger_2_y", "touchpad"));
    static final List<String> CONTROLS = List.of(
            "cross", "circle", "square", "triangle", "share", "options", "ps", "touchpad",
            "left_bumper", "right_bumper", "left_trigger", "right_trigger",
            "dpad_up", "dpad_down", "dpad_left", "dpad_right",
            "left_stick", "right_stick", "left_stick_button", "right_stick_button");
    static final Set<String> ANALOG_FIELDS = Set.of(
            "left_trigger", "right_trigger", "left_stick_x", "left_stick_y", "right_stick_x", "right_stick_y",
            "touchpad_finger_1_x", "touchpad_finger_1_y", "touchpad_finger_2_x", "touchpad_finger_2_y");
    static final Set<String> FIELDS = new HashSet<>();
    static {
        FIELDS.addAll(CONTROLS);
        FIELDS.addAll(ALIASES.keySet());
        FIELDS.removeAll(Set.of("left_stick", "right_stick"));  // no such fields; the stick is read by axis
    }

    // Name shown on the diagram: PlayStation name (Xbox name)
    static final Map<String, String> DISPLAY = Map.ofEntries(
            Map.entry("cross", "Cross (A)"), Map.entry("circle", "Circle (B)"),
            Map.entry("square", "Square (X)"), Map.entry("triangle", "Triangle (Y)"),
            Map.entry("share", "Share (Back)"), Map.entry("options", "Options (Start)"),
            Map.entry("ps", "PS (Guide)"), Map.entry("touchpad", "Touchpad"),
            Map.entry("left_bumper", "L1 (LB)"), Map.entry("right_bumper", "R1 (RB)"),
            Map.entry("left_trigger", "L2 (LT)"), Map.entry("right_trigger", "R2 (RT)"),
            Map.entry("dpad_up", "D-pad ↑"), Map.entry("dpad_down", "D-pad ↓"),
            Map.entry("dpad_left", "D-pad ←"), Map.entry("dpad_right", "D-pad →"),
            Map.entry("left_stick", "Left stick"), Map.entry("right_stick", "Right stick"),
            Map.entry("left_stick_button", "L3 (stick click)"), Map.entry("right_stick_button", "R3 (stick click)"));
    static final Set<String> INIT_METHODS = Set.of("init", "init_loop");
    static final Set<String> NOT_METHODS =
            Set.of("if", "for", "while", "switch", "catch", "synchronized", "try", "else", "do", "return", "new");

    record Binding(String gamepad, String control, String kind, String phase, String action, String source,
                   String detail) {}

    record OpModeInfo(String name, String group, String kind, Path path, List<Binding> bindings) {}

    record Method(String name, String params, int start, int end) {}

    record Read(int pos, String control, String kind, String detail) {}

    record Helper(int paramIndex, List<Read> reads, String rel, String src, Method method) {}

    /** One control on one gamepad, with everything it does. */
    record Row(String key, String control, String tags, String auto, String label, List<String> sources) {
        String text() { return label != null ? label : auto; }
    }

    // ---------------------------------------------------------------- Java scanning

    /** Replace comments (not string contents) with spaces, keeping every offset and line number. */
    static String blankComments(String src) {
        char[] out = src.toCharArray();
        int i = 0, n = src.length();
        while (i < n) {
            char c = src.charAt(i);
            if (c == '"' || c == '\'') {
                int j = i + 1;
                while (j < n && src.charAt(j) != c) j += src.charAt(j) == '\\' ? 2 : 1;
                i = j + 1;
            } else if (src.startsWith("//", i)) {
                int j = src.indexOf('\n', i);
                if (j < 0) j = n;
                Arrays.fill(out, i, j, ' ');
                i = j;
            } else if (src.startsWith("/*", i)) {
                int j = src.indexOf("*/", i + 2);
                j = j < 0 ? n : j + 2;
                for (int k = i; k < j; k++) if (out[k] != '\n') out[k] = ' ';
                i = j;
            } else {
                i++;
            }
        }
        return new String(out);
    }

    /** Index of the bracket that closes the one at src[i]. */
    static int matching(String src, int i) {
        char opener = src.charAt(i);
        char closer = opener == '(' ? ')' : opener == '{' ? '}' : ']';
        int depth = 0;
        for (int j = i; j < src.length(); j++) {
            char c = src.charAt(j);
            if (c == opener) depth++;
            else if (c == closer && --depth == 0) return j;
        }
        return src.length() - 1;
    }

    static final Pattern METHOD_HEADER =
            Pattern.compile("(\\w+)\\s*\\(([^()]*(?:\\([^()]*\\)[^()]*)*)\\)\\s*(?:throws\\s+[\\w.,\\s]+)?$");
    static final Pattern NEW_OBJECT = Pattern.compile("\\bnew\\s+\\w");

    /** Every method body in the file. */
    static List<Method> methods(String src) {
        List<Method> found = new ArrayList<>();
        for (int start = src.indexOf('{'); start >= 0; start = src.indexOf('{', start + 1)) {
            int k = start - 1;
            while (k >= 0 && ";{}".indexOf(src.charAt(k)) < 0) k--;
            String header = collapse(src.substring(k + 1, start));
            Matcher h = METHOD_HEADER.matcher(header);
            if (!h.find() || NOT_METHODS.contains(h.group(1)) || header.endsWith("->")
                    || NEW_OBJECT.matcher(header).find()) continue;
            String before = header.substring(0, h.start()).trim();
            String lastWord = before.isEmpty() ? "" : before.substring(before.lastIndexOf(' ') + 1);
            if (lastWord.isEmpty() || NOT_METHODS.contains(lastWord)) continue;  // a call, not a declaration
            found.add(new Method(h.group(1), h.group(2), start, matching(src, start)));
        }
        return found;
    }

    static Method enclosing(List<Method> meths, int pos) {
        return meths.stream().filter(m -> m.start < pos && pos < m.end)
                .min(Comparator.comparingInt(m -> m.end - m.start)).orElse(null);
    }

    static List<String> splitArgs(String s) {
        List<String> args = new ArrayList<>();
        int depth = 0;
        StringBuilder cur = new StringBuilder();
        for (char c : s.toCharArray()) {
            if ("([{".indexOf(c) >= 0) depth++;
            else if (")]}".indexOf(c) >= 0) depth--;
            if (c == ',' && depth == 0) {
                args.add(cur.toString().trim());
                cur.setLength(0);
            } else {
                cur.append(c);
            }
        }
        if (!cur.toString().isBlank()) args.add(cur.toString().trim());
        return args;
    }

    static int lineOf(String src, int pos) {
        int line = 1;
        for (int i = 0; i < pos; i++) if (src.charAt(i) == '\n') line++;
        return line;
    }

    static String collapse(String s) {
        return s.trim().replaceAll("\\s+", " ");
    }

    static boolean isWord(char c) {
        return Character.isLetterOrDigit(c) || c == '_';
    }

    /** The identifier just before pos, skipping whitespace (which must be there if spaceNeeded). */
    static String wordBefore(String src, int pos, boolean spaceNeeded) {
        int j = pos - 1;
        while (j >= 0 && Character.isWhitespace(src.charAt(j))) j--;
        if (spaceNeeded && j == pos - 1) return "";
        int end = j + 1;
        while (j >= 0 && isWord(src.charAt(j))) j--;
        return src.substring(j + 1, end);
    }

    // ---------------------------------------------------------------- auto labels

    static String words(String name) {
        return name.replaceAll("(?<=[a-z0-9])(?=[A-Z])", " ").replace('_', ' ').toLowerCase();
    }

    /** Drop qualifiers: Hive.LEFT -> LEFT, robot.intake.foo -> foo. */
    static String shortExpr(String expr) {
        return collapse(expr).replaceAll("\\b(?:\\w+\\.)+(?=\\w)", "");
    }

    static final Pattern SCHEDULE = Pattern.compile("(?:\\w+\\.)*schedule\\((.*)\\)");
    static final Pattern CALL = Pattern.compile("(?:\\w+\\.)*(\\w+)\\((.*)\\)");
    static final Pattern ASSIGN = Pattern.compile("([\\w.]+)\\s*=\\s*([\\w.]+)");

    /** schedule(robot.launcher.nudgeTurret(+1)) -> "nudge turret +1". */
    static String describe(String stmt) {
        stmt = collapse(stmt);
        if (stmt.endsWith(";")) stmt = stmt.substring(0, stmt.length() - 1).trim();
        Matcher m = SCHEDULE.matcher(stmt);
        if (m.matches()) stmt = m.group(1);
        m = CALL.matcher(stmt);
        if (m.matches()) {
            String args = splitArgs(m.group(2)).stream().map(ControlLayout::shortExpr).collect(Collectors.joining(", "));
            return (words(m.group(1)) + (args.isEmpty() ? "" : " " + args)).trim();
        }
        m = ASSIGN.matcher(stmt);
        if (m.matches()) return words(shortExpr(m.group(1))) + " → " + shortExpr(m.group(2));
        return shortExpr(stmt);
    }

    /** Auto label for the gamepad read at pos. */
    static String actionAt(String src, int pos, Method meth) {
        // Inside an if condition: label is what the if does
        int depth = 0;
        for (int k = pos - 1; k >= 0; k--) {
            char c = src.charAt(k);
            if (c == ')') {
                depth++;
            } else if (c == '(') {
                if (depth == 0) {
                    if (wordBefore(src, k, false).equals("if")) {
                        String body = src.substring(matching(src, k) + 1).stripLeading();
                        if (body.startsWith("{")) {
                            String inner = body.substring(1, matching(body, 0));
                            return Arrays.stream(inner.split(";")).filter(s -> !s.isBlank())
                                    .map(ControlLayout::describe).collect(Collectors.joining("; "));
                        }
                        return describe(body.substring(0, Math.max(0, body.indexOf(';'))));
                    }
                    break;
                }
                depth--;
            } else if (";{}".indexOf(c) >= 0) {
                break;
            }
        }
        // Otherwise: the call it's an argument of, e.g. driveFieldCentric(() -> -gamepad1.left_stick_y, ...)
        depth = 0;
        for (int k = pos - 1; k >= 0; k--) {
            char c = src.charAt(k);
            if (c == ')') {
                depth++;
            } else if (c == '(') {
                if (depth == 0) {
                    String call = wordBefore(src, k, false);
                    if (!call.isEmpty() && !NOT_METHODS.contains(call)) return words(call);
                } else {
                    depth--;
                }
            } else if (";{}".indexOf(c) >= 0 && depth == 0) {
                break;
            }
        }
        return meth != null ? words(meth.name) : "?";
    }

    // ---------------------------------------------------------------- binding extraction

    static final Pattern EVENT = Pattern.compile("(\\w+?)Was(Pressed|Released)");
    static final Pattern COMPARED = Pattern.compile("\\s*[<>]=?");

    /** Every read of the gamepad variable var. */
    static List<Read> reads(String src, String var) {
        List<Read> out = new ArrayList<>();
        Matcher m = Pattern.compile("\\b" + Pattern.quote(var) + "\\s*\\.\\s*(\\w+)\\b(\\s*\\(\\s*\\))?").matcher(src);
        while (m.find()) {
            String member = m.group(1);
            boolean call = m.group(2) != null;
            Matcher ev = EVENT.matcher(member);
            if (ev.matches() && call) {
                String sdk = ev.group(1).replaceAll("(?<=[a-z])(?=[A-Z])", "_").toLowerCase();
                String control = ALIASES.getOrDefault(sdk, sdk);
                if (CONTROLS.contains(control)) {
                    out.add(new Read(m.start(), control, ev.group(2).equals("Pressed") ? "press" : "release", ""));
                }
            } else if (FIELDS.contains(member) && !call) {
                String control = ALIASES.getOrDefault(member, member);
                String kind = "hold", detail = "";
                if (ANALOG_FIELDS.contains(member)) {
                    boolean compared = COMPARED.matcher(src).region(m.end(), src.length()).lookingAt();
                    kind = compared ? "hold" : "analog";
                    if (control.endsWith("stick")) detail = member.substring(member.lastIndexOf('_') + 1);
                }
                out.add(new Read(m.start(), control, kind, detail));
            }
        }
        return out;
    }

    static String phaseOf(String src, List<Method> meths, Method meth, int pos, int depth) {
        if (meth == null) return "match";
        if (INIT_METHODS.contains(meth.name)) return "init";
        if (meth.name.equals("runOpMode")) {
            int wait = src.indexOf("waitForStart", meth.start);
            return wait >= 0 && wait < meth.end && pos < wait ? "init" : "match";
        }
        if (Set.of("loop", "start", "stop").contains(meth.name) || depth > 3) return "match";
        // A helper: use its callers' phase (init only if every caller is init)
        Set<String> phases = new HashSet<>();
        Matcher c = Pattern.compile("\\b" + Pattern.quote(meth.name) + "\\s*\\(").matcher(src);
        while (c.find()) {
            Method caller = enclosing(meths, c.start());
            if (caller != null && !caller.equals(meth)) phases.add(phaseOf(src, meths, caller, c.start(), depth + 1));
        }
        return phases.equals(Set.of("init")) ? "init" : "match";
    }

    static final Pattern GAMEPAD_PARAM = Pattern.compile("(?:final\\s+)?(?:[\\w.]+\\.)?Gamepad\\s+(\\w+)");

    /** Methods anywhere in TeamCode with a Gamepad parameter, by name. */
    static Map<String, List<Helper>> helperMethods(Map<String, String> files) {
        Map<String, List<Helper>> helpers = new LinkedHashMap<>();
        files.forEach((rel, src) -> {
            for (Method meth : methods(src)) {
                List<String> params = splitArgs(meth.params);
                for (int idx = 0; idx < params.size(); idx++) {
                    Matcher pm = GAMEPAD_PARAM.matcher(params.get(idx));
                    if (!pm.matches()) continue;
                    List<Read> rs = reads(src.substring(meth.start, meth.end), pm.group(1)).stream()
                            .map(r -> new Read(meth.start + r.pos, r.control, r.kind, r.detail)).toList();
                    if (!rs.isEmpty()) {
                        helpers.computeIfAbsent(meth.name, k -> new ArrayList<>()).add(new Helper(idx, rs, rel, src, meth));
                    }
                }
            }
        });
        return helpers;
    }

    static final Pattern OPMODE = Pattern.compile("@(?:[\\w.]+\\.)?(TeleOp|Autonomous)\\s*\\(([^)]*)\\)");
    static final Pattern DISABLED = Pattern.compile("@(?:[\\w.]+\\.)?Disabled\\b");
    static final Set<String> TYPES = Set.of("void", "boolean", "int", "double", "float", "long");

    static List<OpModeInfo> scan(Path root) throws IOException {
        Map<String, String> files = new LinkedHashMap<>();
        Map<String, Path> paths = new LinkedHashMap<>();
        Path sourceDir = root.resolve(SOURCE_DIR), packageDir = root.resolve(PACKAGE_DIR);
        List<Path> javaFiles;
        try (Stream<Path> walk = Files.walk(sourceDir)) {
            javaFiles = walk.filter(p -> p.toString().endsWith(".java")).sorted().toList();
        }
        for (Path path : javaFiles) {
            // Short names in the listing: opmodes/TeleOp.java rather than the full package path
            String rel = slashes((path.startsWith(packageDir) ? packageDir : sourceDir).relativize(path));
            files.put(rel, blankComments(Files.readString(path)));
            paths.put(rel, path);
        }
        Map<String, List<Helper>> helpers = helperMethods(files);

        List<OpModeInfo> opmodes = new ArrayList<>();
        for (var file : files.entrySet()) {
            String rel = file.getKey(), src = file.getValue();
            Matcher ann = OPMODE.matcher(src);
            if (!ann.find()) continue;
            String around = src.substring(0, ann.start()) + src.substring(ann.end(), Math.min(src.length(), ann.end() + 200));
            if (DISABLED.matcher(around).find()) continue;
            Matcher cls = Pattern.compile("class\\s+(\\w+)").matcher(src);
            Matcher name = Pattern.compile("name\\s*=\\s*\"([^\"]*)\"").matcher(ann.group(2));
            Matcher group = Pattern.compile("group\\s*=\\s*\"([^\"]*)\"").matcher(ann.group(2));
            OpModeInfo op = new OpModeInfo(
                    name.find() ? name.group(1) : cls.find(ann.end()) ? cls.group(1) : rel,
                    group.find() ? group.group(1) : "", ann.group(1), paths.get(rel), new ArrayList<>());
            List<Method> meths = methods(src);

            for (String gp : List.of("gamepad1", "gamepad2")) {
                for (Read r : reads(src, gp)) {
                    Method meth = enclosing(meths, r.pos);
                    op.bindings.add(new Binding(gp, r.control, r.kind, phaseOf(src, meths, meth, r.pos, 0),
                            actionAt(src, r.pos, meth), rel + ":" + lineOf(src, r.pos), r.detail));
                }
            }

            // Calls to helpers that take a Gamepad, e.g. MatchState.selectAlliance(gamepad1, telemetry)
            helpers.forEach((hname, variants) -> {
                Matcher call = Pattern.compile("\\b" + Pattern.quote(hname) + "\\s*\\(").matcher(src);
                while (call.find()) {
                    String typeBefore = wordBefore(src, call.start(), true);
                    if (TYPES.contains(typeBefore) || (!typeBefore.isEmpty() && Character.isUpperCase(typeBefore.charAt(0)))) {
                        continue;  // the declaration, not a call
                    }
                    int open = call.end() - 1;
                    List<String> args = splitArgs(src.substring(open + 1, matching(src, open)));
                    Method caller = enclosing(meths, call.start());
                    String phase = phaseOf(src, meths, caller, call.start(), 0);
                    for (Helper h : variants) {
                        if (h.paramIndex >= args.size()) continue;
                        String gp = args.get(h.paramIndex);
                        if (!gp.equals("gamepad1") && !gp.equals("gamepad2")) continue;
                        for (Read r : h.reads) {
                            op.bindings.add(new Binding(gp, r.control, r.kind, phase, actionAt(h.src, r.pos, h.method),
                                    h.rel + ":" + lineOf(h.src, r.pos), r.detail));
                        }
                    }
                }
            });
            if (!op.bindings.isEmpty()) opmodes.add(op);
        }

        Map<String, Integer> order = Map.of("Competition", 0, "", 1);
        opmodes.sort(Comparator.<OpModeInfo>comparingInt(o -> order.getOrDefault(o.group, 2))
                .thenComparing(OpModeInfo::group)
                .thenComparing(o -> !o.kind.equals("TeleOp"))
                .thenComparing(OpModeInfo::name));
        return opmodes;
    }

    // ---------------------------------------------------------------- grouping and labels

    static List<Row> rowsFor(OpModeInfo op, String phase, String gamepad, Map<String, Map<String, String>> labels) {
        Map<String, List<Binding>> byControl = new LinkedHashMap<>();
        for (Binding b : op.bindings) {
            if (b.phase.equals(phase) && b.gamepad.equals(gamepad)) {
                byControl.computeIfAbsent(b.control, k -> new ArrayList<>()).add(b);
            }
        }
        List<Row> rows = new ArrayList<>();
        for (String control : CONTROLS) {
            List<Binding> bs = byControl.get(control);
            if (bs == null) continue;
            String key = (phase.equals("init") ? "init." : "") + gamepad + "." + control;
            List<String> kinds = bs.stream().map(Binding::kind).distinct().toList();
            Set<String> axes = bs.stream().map(Binding::detail).filter(d -> !d.isEmpty()).collect(Collectors.toCollection(TreeSet::new));
            String tags = String.join(" / ", axes.isEmpty() ? kinds : axes);
            List<List<String>> actions = bs.stream().map(b -> List.of(b.kind, b.action)).distinct().toList();
            String auto = actions.size() > 1 && kinds.size() > 1
                    ? actions.stream().map(a -> a.get(0) + ": " + a.get(1)).collect(Collectors.joining("; "))
                    : actions.stream().map(a -> a.get(1)).distinct().collect(Collectors.joining("; "));
            rows.add(new Row(key, control, tags, auto, label(labels, op.name, key),
                    bs.stream().map(Binding::source).distinct().toList()));
        }
        return rows;
    }

    static String label(Map<String, Map<String, String>> labels, String opmode, String key) {
        String own = labels.getOrDefault(opmode, Map.of()).get(key);
        return own != null && !own.isBlank() ? own : labels.getOrDefault("*", Map.of()).get(key);
    }

    @SuppressWarnings("unchecked")
    static Map<String, Map<String, String>> loadLabels(Path file) throws IOException {
        Map<String, Map<String, String>> labels = new LinkedHashMap<>();
        if (!Files.exists(file)) return labels;
        Object parsed = new Json(Files.readString(file)).parse();
        if (!(parsed instanceof Map)) return labels;
        ((Map<String, Object>) parsed).forEach((opmode, section) -> {
            if (opmode.startsWith("_") || !(section instanceof Map)) return;
            Map<String, String> entries = new LinkedHashMap<>();
            ((Map<String, Object>) section).forEach((k, v) -> {
                if (v instanceof String s) entries.put(k, s);
            });
            labels.put(opmode, entries);
        });
        return labels;
    }

    /** Just enough JSON for control-labels.json. */
    static final class Json {
        final String s;
        int i;

        Json(String s) { this.s = s; }

        Object parse() {
            Object v = value();
            ws();
            if (i < s.length()) fail("unexpected text");
            return v;
        }

        Object value() {
            ws();
            if (i >= s.length()) fail("unexpected end");
            char c = s.charAt(i);
            if (c == '{') {
                i++;
                Map<String, Object> m = new LinkedHashMap<>();
                ws();
                if (peek('}')) return m;
                do {
                    ws();
                    String k = string();
                    ws();
                    expect(':');
                    m.put(k, value());
                    ws();
                } while (peek(','));
                expect('}');
                return m;
            }
            if (c == '[') {
                i++;
                List<Object> list = new ArrayList<>();
                ws();
                if (peek(']')) return list;
                do list.add(value()); while (peekAfterWs(','));
                ws();
                expect(']');
                return list;
            }
            if (c == '"') return string();
            int j = i;
            while (i < s.length() && ",}] \t\r\n".indexOf(s.charAt(i)) < 0) i++;
            String lit = s.substring(j, i);
            return switch (lit) {
                case "true" -> true;
                case "false" -> false;
                case "null" -> null;
                default -> {
                    try {
                        yield Double.parseDouble(lit);
                    } catch (NumberFormatException e) {
                        fail("unexpected '" + lit + "'");
                        yield null;
                    }
                }
            };
        }

        String string() {
            expect('"');
            StringBuilder b = new StringBuilder();
            while (true) {
                if (i >= s.length()) fail("unterminated string");
                char c = s.charAt(i++);
                if (c == '"') return b.toString();
                if (c != '\\') {
                    b.append(c);
                    continue;
                }
                char e = s.charAt(i++);
                switch (e) {
                    case 'n' -> b.append('\n');
                    case 't' -> b.append('\t');
                    case 'r' -> b.append('\r');
                    case 'b' -> b.append('\b');
                    case 'f' -> b.append('\f');
                    case 'u' -> {
                        b.append((char) Integer.parseInt(s.substring(i, i + 4), 16));
                        i += 4;
                    }
                    default -> b.append(e);
                }
            }
        }

        void ws() {
            while (i < s.length() && Character.isWhitespace(s.charAt(i))) i++;
        }

        boolean peek(char c) {
            if (i < s.length() && s.charAt(i) == c) {
                i++;
                return true;
            }
            return false;
        }

        boolean peekAfterWs(char c) {
            ws();
            return peek(c);
        }

        void expect(char c) {
            if (!peek(c)) fail("expected '" + c + "'");
        }

        void fail(String why) {
            throw new IllegalArgumentException(LABELS_FILE + " line " + lineOf(s, Math.min(i, s.length())) + ": " + why);
        }
    }

    // ---------------------------------------------------------------- drawing

    static final int W = 1040;
    static final int SCALE = 2;               // pixels per unit, so the PNG stays sharp when printed
    static final double OFFSET_X = 20;        // the controller is drawn around x = 500; this centers it
    static final double CX = 500;
    static final double BODY_TOP = 80, BODY_BOTTOM = 414;
    static final double LABEL_WIDTH = 232;

    // Where each control sits on the drawing (leader lines start here)
    static final Map<String, double[]> ANCHORS = Map.ofEntries(
            Map.entry("left_trigger", new double[] {355, 92}), Map.entry("right_trigger", new double[] {645, 92}),
            Map.entry("left_bumper", new double[] {360, 124}), Map.entry("right_bumper", new double[] {640, 124}),
            Map.entry("share", new double[] {420, 168}), Map.entry("options", new double[] {580, 168}),
            Map.entry("touchpad", new double[] {500, 180}), Map.entry("ps", new double[] {500, 296}),
            Map.entry("dpad_up", new double[] {372, 196}), Map.entry("dpad_down", new double[] {372, 244}),
            Map.entry("dpad_left", new double[] {348, 220}), Map.entry("dpad_right", new double[] {396, 220}),
            Map.entry("triangle", new double[] {628, 194}), Map.entry("cross", new double[] {628, 246}),
            Map.entry("square", new double[] {602, 220}), Map.entry("circle", new double[] {654, 220}),
            Map.entry("left_stick", new double[] {435, 290}), Map.entry("right_stick", new double[] {565, 290}),
            Map.entry("left_stick_button", new double[] {435, 290}), Map.entry("right_stick_button", new double[] {565, 290}));
    static final String BODY = "M 380 140 C 420 128 580 128 620 140 C 670 150 692 190 710 260 C 728 330 732 380 700 398 "
            + "C 670 414 642 392 624 360 C 610 336 592 322 560 322 L 440 322 C 408 322 390 336 376 360 "
            + "C 358 392 330 414 300 398 C 268 380 272 330 290 260 C 308 190 330 150 380 140 Z";

    static final Color BG = new Color(0xf7f7f5), PANEL = Color.WHITE, INK = new Color(0x1c1d1f),
            MUTED = new Color(0x6b6f76), LINE = new Color(0xd9dadc), BODY_FILL = new Color(0xe9eaec),
            PART = new Color(0xf7f7f8), ON = new Color(0x2563eb), ON_FILL = new Color(0xdbe7ff),
            LEAD = new Color(0x2563eb & 0xffffff | 190 << 24, true), AUTO = new Color(0x9a6700);

    static final String FAMILY = pickFamily("Helvetica Neue", "Segoe UI", "Arial", "Liberation Sans", "DejaVu Sans");
    static final Font H1 = new Font(FAMILY, Font.BOLD, 28), H3 = new Font(FAMILY, Font.BOLD, 16),
            SUB = new Font(FAMILY, Font.PLAIN, 14), CTL = new Font(FAMILY, Font.BOLD, 14),
            TAG = new Font(FAMILY, Font.PLAIN, 14), ACT = new Font(FAMILY, Font.PLAIN, 15),
            ACT_AUTO = new Font(FAMILY, Font.ITALIC, 15);

    static String pickFamily(String... preferred) {
        Set<String> installed = Set.of(GraphicsEnvironment.getLocalGraphicsEnvironment().getAvailableFontFamilyNames());
        for (String f : preferred) if (installed.contains(f)) return f;
        return Font.SANS_SERIF;
    }

    /** A graphics context for measuring text before the image exists. */
    static final Graphics2D MEASURE = hinted(new BufferedImage(1, 1, BufferedImage.TYPE_INT_RGB).createGraphics());

    static Graphics2D hinted(Graphics2D g) {
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
        g.setRenderingHint(RenderingHints.KEY_FRACTIONALMETRICS, RenderingHints.VALUE_FRACTIONALMETRICS_ON);
        g.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL, RenderingHints.VALUE_STROKE_PURE);
        g.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_QUALITY);
        return g;
    }

    static double width(String text, Font font) {
        return font.getStringBounds(text, MEASURE.getFontRenderContext()).getWidth();
    }

    /** Arrows if the font has them, words if it doesn't. */
    static String display(String control) {
        String name = DISPLAY.get(control);
        if (CTL.canDisplayUpTo(name) == -1) return name;
        return name.replace("↑", "up").replace("↓", "down").replace("←", "left").replace("→", "right");
    }

    static List<String> wrap(String text, Font font) {
        List<String> lines = new ArrayList<>();
        String cur = "";
        for (String word : text.trim().split("\\s+")) {
            String next = cur.isEmpty() ? word : cur + " " + word;
            if (!cur.isEmpty() && width(next, font) > LABEL_WIDTH) {
                lines.add(cur);
                cur = word;
            } else {
                cur = next;
            }
        }
        if (!cur.isEmpty()) lines.add(cur);
        if (lines.size() > 3) {
            lines = new ArrayList<>(lines.subList(0, 3));
            String last = lines.get(2);
            while (!last.isEmpty() && width(last + "…", font) > LABEL_WIDTH) last = last.substring(0, last.length() - 1);
            lines.set(2, last.stripTrailing() + "…");
        }
        return lines;
    }

    record Callout(Row row, List<String> lines, double y, double height, int side) {
        double textX() { return CX + side * 250; }
    }

    /** One gamepad's drawing: which controls light up and where each label goes. */
    record Pad(Set<String> used, List<Callout> callouts, double top, double bottom) {}

    static Pad layout(List<Row> rows) {
        Set<String> used = rows.stream().map(Row::control).collect(Collectors.toCollection(HashSet::new));
        if (used.contains("left_stick_button")) used.add("left_stick");
        if (used.contains("right_stick_button")) used.add("right_stick");

        // Labels: left-side controls in the left column, right-side in the right, center ones where there's room
        List<Row> left = new ArrayList<>(), right = new ArrayList<>();
        for (Row r : rows) {
            double x = ANCHORS.get(r.control)[0];
            if (x < CX) left.add(r);
            else if (x > CX) right.add(r);
        }
        for (Row r : rows) {
            if (ANCHORS.get(r.control)[0] == CX) (left.size() <= right.size() ? left : right).add(r);
        }

        List<Callout> callouts = new ArrayList<>();
        double top = BODY_TOP, bottom = BODY_BOTTOM;
        for (int side : new int[] {-1, 1}) {
            List<Row> column = side < 0 ? left : right;
            // Top to bottom, outer button first so the leader lines of side-by-side buttons don't cross
            column.sort(Comparator.<Row>comparingDouble(r -> ANCHORS.get(r.control)[1])
                    .thenComparingDouble(r -> -ANCHORS.get(r.control)[0] * side));
            // Stack the labels near their controls without overlapping
            double y = Double.NEGATIVE_INFINITY;
            for (Row r : column) {
                List<String> lines = wrap(r.text(), r.label != null ? ACT : ACT_AUTO);
                double h = 18 + 17 * lines.size();
                y = Math.max(y, ANCHORS.get(r.control)[1] - h / 2);
                callouts.add(new Callout(r, lines, y, h, side));
                top = Math.min(top, y);
                bottom = Math.max(bottom, y + h);
                y += h + 10;
            }
        }
        return new Pad(used, callouts, top, bottom);
    }

    static Path2D svgPath(String d) {
        Path2D p = new Path2D.Double();
        String[] t = d.trim().split("\\s+");
        for (int i = 0; i < t.length; ) {
            switch (t[i++]) {
                case "M" -> p.moveTo(num(t, i++), num(t, i++));
                case "L" -> p.lineTo(num(t, i++), num(t, i++));
                case "C" -> p.curveTo(num(t, i++), num(t, i++), num(t, i++), num(t, i++), num(t, i++), num(t, i++));
                case "Z" -> p.closePath();
                default -> throw new IllegalArgumentException("path: " + t[i - 1]);
            }
        }
        return p;
    }

    static double num(String[] t, int i) {
        return Double.parseDouble(t[i]);
    }

    static void stroke(Graphics2D g, Shape s, Color c, double w) {
        g.setColor(c);
        g.setStroke(new BasicStroke((float) w, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
        g.draw(s);
    }

    static void part(Graphics2D g, Shape s, boolean on) {
        g.setColor(on ? ON_FILL : PART);
        g.fill(s);
        stroke(g, s, on ? ON : LINE, on ? 2 : 1.5);
    }

    static RoundRectangle2D rect(double x, double y, double w, double h, double r) {
        return new RoundRectangle2D.Double(x, y, w, h, 2 * r, 2 * r);
    }

    static Ellipse2D circle(double x, double y, double r) {
        return new Ellipse2D.Double(x - r, y - r, 2 * r, 2 * r);
    }

    static void text(Graphics2D g, String s, Font f, Color c, double x, double y) {
        g.setFont(f);
        g.setColor(c);
        g.drawString(s, (float) x, (float) y);
    }

    static void drawPad(Graphics2D g, Pad pad) {
        Set<String> used = pad.used;
        Path2D body = svgPath(BODY);
        g.setColor(BODY_FILL);
        g.fill(body);
        stroke(g, body, LINE, 2);

        // Leader lines go over the body but under the buttons, so each one comes out of its button
        for (Callout c : pad.callouts) {
            double[] a = ANCHORS.get(c.row.control);
            double mid = c.y + c.height / 2;
            Path2D lead = new Path2D.Double();
            lead.moveTo(a[0], a[1]);
            lead.lineTo(CX + c.side * 232, mid);
            lead.lineTo(c.textX() - c.side * 6, mid);
            stroke(g, lead, LEAD, 1.2);
        }

        part(g, rect(330, 80, 60, 24, 7), used.contains("left_trigger"));
        part(g, rect(610, 80, 60, 24, 7), used.contains("right_trigger"));
        part(g, rect(322, 116, 80, 16, 7), used.contains("left_bumper"));
        part(g, rect(598, 116, 80, 16, 7), used.contains("right_bumper"));
        part(g, rect(445, 146, 110, 64, 10), used.contains("touchpad"));
        part(g, rect(410, 158, 20, 10, 5), used.contains("share"));
        part(g, rect(570, 158, 20, 10, 5), used.contains("options"));
        part(g, circle(500, 296, 9), used.contains("ps"));
        part(g, rect(363, 186, 18, 18, 3), used.contains("dpad_up"));
        part(g, rect(363, 236, 18, 18, 3), used.contains("dpad_down"));
        part(g, rect(338, 211, 18, 18, 3), used.contains("dpad_left"));
        part(g, rect(388, 211, 18, 18, 3), used.contains("dpad_right"));

        // Face buttons with their PlayStation symbols, drawn as shapes so they don't depend on the font
        for (String c : List.of("triangle", "cross", "square", "circle")) {
            double x = ANCHORS.get(c)[0], y = ANCHORS.get(c)[1];
            boolean on = used.contains(c);
            part(g, circle(x, y, 12), on);
            Shape glyph = switch (c) {
                case "triangle" -> {
                    Path2D t = new Path2D.Double();
                    t.moveTo(x, y - 5.5);
                    t.lineTo(x + 5.5, y + 4);
                    t.lineTo(x - 5.5, y + 4);
                    t.closePath();
                    yield t;
                }
                case "cross" -> {
                    Path2D t = new Path2D.Double();
                    t.moveTo(x - 4.5, y - 4.5);
                    t.lineTo(x + 4.5, y + 4.5);
                    t.moveTo(x + 4.5, y - 4.5);
                    t.lineTo(x - 4.5, y + 4.5);
                    yield t;
                }
                case "square" -> new Rectangle2D.Double(x - 4.5, y - 4.5, 9, 9);
                default -> circle(x, y, 5);
            };
            stroke(g, glyph, on ? ON : MUTED, on ? 1.8 : 1.3);
        }
        for (String side : List.of("left", "right")) {
            double x = ANCHORS.get(side + "_stick")[0], y = ANCHORS.get(side + "_stick")[1];
            part(g, circle(x, y, 26), used.contains(side + "_stick") || used.contains(side + "_stick_button"));
            stroke(g, circle(x, y, 15), LINE, 1.5);
        }

        for (Callout c : pad.callouts) {
            String name = display(c.row.control), tags = " = " + c.row.tags;
            double total = width(name, CTL) + width(tags, TAG);
            double x = c.side > 0 ? c.textX() : c.textX() - total;
            text(g, name, CTL, INK, x, c.y + 13);
            text(g, tags, TAG, MUTED, x + width(name, CTL), c.y + 13);
            Font f = c.row.label != null ? ACT : ACT_AUTO;
            for (int i = 0; i < c.lines.size(); i++) {
                String line = c.lines.get(i);
                double lx = c.side > 0 ? c.textX() : c.textX() - width(line, f);
                text(g, line, f, c.row.label != null ? INK : AUTO, lx, c.y + 31 + 17 * i);
            }
        }
    }

    record Card(String title, List<Row> rows, Pad pad) {
        double height() { return 44 + (pad.bottom - pad.top) + 16; }
    }

    static List<Card> cards(OpModeInfo op, Map<String, Map<String, String>> labels) {
        List<Card> cards = new ArrayList<>();
        for (String phase : List.of("init", "match")) {
            for (String gp : List.of("gamepad1", "gamepad2")) {
                List<Row> rows = rowsFor(op, phase, gp, labels);
                if (rows.isEmpty()) continue;
                String role = label(labels, op.name, gp);
                String when = phase.equals("init") ? "Init (before start)" : op.kind.equals("TeleOp") ? "Match" : "Running";
                String title = "Gamepad " + gp.charAt(gp.length() - 1) + (role != null ? ": " + role : "") + " — " + when;
                cards.add(new Card(title, rows, layout(rows)));
            }
        }
        return cards;
    }

    /** One OpMode's gamepads, stacked top to bottom. */
    static BufferedImage draw(OpModeInfo op, String subtitle, List<Card> cards) {
        boolean anyAuto = cards.stream().flatMap(c -> c.rows.stream()).anyMatch(r -> r.label == null);
        double top = anyAuto ? 112 : 90;
        double height = top + cards.stream().mapToDouble(c -> c.height() + 12).sum() + 4;

        BufferedImage img = new BufferedImage(W * SCALE, (int) Math.ceil(height * SCALE), BufferedImage.TYPE_INT_RGB);
        Graphics2D g = hinted(img.createGraphics());
        g.scale(SCALE, SCALE);
        g.setColor(BG);
        g.fillRect(0, 0, W, (int) Math.ceil(height));

        text(g, op.name, H1, INK, 24, 46);
        text(g, subtitle, SUB, MUTED, 24, 72);
        if (anyAuto) {
            String note = " labels come straight from the code; describe them in " + LABELS_FILE + ".";
            text(g, "Italic", ACT_AUTO.deriveFont(14f), AUTO, 24, 96);
            text(g, note, SUB, MUTED, 24 + width("Italic", ACT_AUTO.deriveFont(14f)), 96);
        }

        double y = top;
        for (Card card : cards) {
            g.setColor(PANEL);
            RoundRectangle2D panel = rect(16, y, W - 32, card.height(), 12);
            g.fill(panel);
            stroke(g, panel, LINE, 1);
            text(g, card.title, H3, MUTED, 36, y + 30);
            AffineTransform saved = g.getTransform();
            g.translate(OFFSET_X, y + 44 - card.pad.top);
            drawPad(g, card.pad);
            g.setTransform(saved);
            y += card.height() + 12;
        }
        g.dispose();
        return img;
    }

    // ---------------------------------------------------------------- PNG fingerprint

    /** Everything an image shows, hashed together with this file, so unchanged images aren't redrawn. */
    static String fingerprint(Path root, String subtitle, OpModeInfo op, List<Card> cards) throws Exception {
        MessageDigest sha = MessageDigest.getInstance("SHA-256");
        Path self = root.resolve(THIS_FILE);
        if (Files.exists(self)) sha.update(Files.readAllBytes(self));
        StringBuilder b = new StringBuilder(op.name).append('\n').append(subtitle).append('\n');
        for (Card c : cards) {
            b.append(c.title).append('\n');
            for (Row r : c.rows) b.append(r.key).append('|').append(r.tags).append('|').append(r.label).append('|').append(r.auto).append('\n');
        }
        sha.update(b.toString().getBytes(StandardCharsets.UTF_8));
        return HexFormat.of().formatHex(sha.digest());
    }

    static String pngText(byte[] png, String key) {
        int pos = 8;
        while (pos + 8 <= png.length) {
            int len = ByteBuffer.wrap(png, pos, 4).getInt();
            String type = new String(png, pos + 4, 4, StandardCharsets.ISO_8859_1);
            if (type.equals("tEXt") && pos + 8 + len <= png.length) {
                String body = new String(png, pos + 8, len, StandardCharsets.ISO_8859_1);
                int nul = body.indexOf('\0');
                if (nul >= 0 && body.substring(0, nul).equals(key)) return body.substring(nul + 1);
            }
            if (type.equals("IEND")) break;
            pos += 12 + len;
        }
        return null;
    }

    /** The PNG with a tEXt chunk added just before IEND. */
    static byte[] withPngText(byte[] png, String key, String value) {
        byte[] type = "tEXt".getBytes(StandardCharsets.ISO_8859_1);
        byte[] body = (key + "\0" + value).getBytes(StandardCharsets.ISO_8859_1);
        CRC32 crc = new CRC32();
        crc.update(type);
        crc.update(body);
        ByteBuffer chunk = ByteBuffer.allocate(12 + body.length).putInt(body.length).put(type).put(body).putInt((int) crc.getValue());
        int iend = png.length - 12;
        byte[] out = new byte[png.length + chunk.capacity()];
        System.arraycopy(png, 0, out, 0, iend);
        System.arraycopy(chunk.array(), 0, out, iend, chunk.capacity());
        System.arraycopy(png, iend, out, iend + chunk.capacity(), 12);
        return out;
    }

    // ---------------------------------------------------------------- main

    static String slashes(Path p) {
        return p.toString().replace('\\', '/');
    }

    static String slug(String name) {
        return name.toLowerCase().replaceAll("[^a-z0-9]+", "-").replaceAll("^-|-$", "");
    }

    static Path findRoot() {
        for (Path p = Path.of("").toAbsolutePath(); p != null; p = p.getParent()) {
            if (Files.isDirectory(p.resolve(SOURCE_DIR))) return p;
        }
        throw new IllegalStateException("Run this from inside the robot project (couldn't find " + SOURCE_DIR + ")");
    }

    public static void main(String[] args) throws Exception {
        System.setProperty("java.awt.headless", "true");
        List<String> flags = List.of(args);
        boolean quiet = flags.contains("--quiet"), list = flags.contains("--list");
        if (flags.contains("--help") || flags.contains("-h")) {
            System.out.println("usage: java tools/ControlLayout.java [--list] [--quiet]\n"
                    + "  Draws " + OUTPUT_DIR + "/<opmode>.png from the gamepad reads in TeamCode.\n"
                    + "  --list   print every binding with its code location instead of drawing\n"
                    + "  --quiet  only print problems and redrawn images");
            return;
        }

        Path root = findRoot();
        List<OpModeInfo> opmodes = scan(root);
        Map<String, Map<String, String>> labels = loadLabels(root.resolve(LABELS_FILE));

        List<String> missing = new ArrayList<>();
        Set<String> used = new HashSet<>();
        for (OpModeInfo op : opmodes) {
            for (String phase : List.of("init", "match")) {
                for (String gp : List.of("gamepad1", "gamepad2")) {
                    List<Row> rows = rowsFor(op, phase, gp, labels);
                    if (!rows.isEmpty()) {
                        used.add(op.name + "\0" + gp);
                        used.add("*\0" + gp);
                    }
                    for (Row r : rows) {
                        used.add(op.name + "\0" + r.key);
                        used.add("*\0" + r.key);
                        if (r.label == null) {
                            missing.add(String.format("  \"%s\": \"%s\"   # %s  (%s)", op.name, r.key, r.auto, r.sources.get(0)));
                        }
                        if (list) {
                            System.out.printf("%-22s %-32s %-18s %s   %s%n", op.name, r.key, r.tags,
                                    r.label != null ? r.label : "(" + r.auto + ")", r.sources.get(0));
                        }
                    }
                }
            }
        }
        if (list) return;

        List<String> stale = new ArrayList<>();
        labels.forEach((name, section) -> section.keySet().forEach(key -> {
            if (!used.contains(name + "\0" + key)) stale.add(String.format("  \"%s\": \"%s\"", name, key));
        }));

        Path outDir = root.resolve(OUTPUT_DIR);
        Files.createDirectories(outDir);
        Set<String> written = new LinkedHashSet<>();
        List<String> redrawn = new ArrayList<>();
        for (OpModeInfo op : opmodes) {
            String file = slug(op.name) + ".png";
            if (!written.add(file)) continue;  // two OpModes with the same name; keep the first
            Path out = outDir.resolve(file);
            String subtitle = op.kind + (op.group.isEmpty() ? "" : ", " + op.group + " group") + "    "
                    + slashes(root.relativize(op.path));
            List<Card> cards = cards(op, labels);
            String print = fingerprint(root, subtitle, op, cards);
            if (Files.exists(out) && print.equals(pngText(Files.readAllBytes(out), FINGERPRINT_KEY))) continue;

            ByteArrayOutputStream png = new ByteArrayOutputStream();
            ImageIO.write(draw(op, subtitle, cards), "png", png);
            Files.write(out, withPngText(png.toByteArray(), FINGERPRINT_KEY, print));
            redrawn.add(file);
        }
        // Images for OpModes that are gone (renamed, deleted or @Disabled)
        List<String> removed = new ArrayList<>();
        try (Stream<Path> old = Files.list(outDir)) {
            for (Path p : old.filter(p -> p.toString().endsWith(".png")).toList()) {
                if (!written.contains(p.getFileName().toString())) {
                    Files.delete(p);
                    removed.add(p.getFileName().toString());
                }
            }
        }

        if (!quiet || !redrawn.isEmpty() || !removed.isEmpty()) {
            int reads = opmodes.stream().mapToInt(o -> o.bindings.size()).sum();
            String what = redrawn.isEmpty() && removed.isEmpty() ? "unchanged"
                    : (redrawn.isEmpty() ? "" : "drew " + String.join(", ", redrawn))
                    + (!redrawn.isEmpty() && !removed.isEmpty() ? "; " : "")
                    + (removed.isEmpty() ? "" : "removed " + String.join(", ", removed));
            System.out.printf("control layout: %d OpModes, %d gamepad reads -> %s/ (%s)%n", opmodes.size(), reads, OUTPUT_DIR, what);
        }
        if (!missing.isEmpty() && !quiet) {
            System.out.printf("%n%d binding(s) without a label in %s (auto label shown):%n", missing.size(), LABELS_FILE);
            missing.forEach(System.out::println);
        }
        if (!stale.isEmpty()) {
            System.out.printf("%n%d label(s) in %s no longer match any binding:%n", stale.size(), LABELS_FILE);
            stale.forEach(System.out::println);
        }
    }
}
