# Path planner

Plans auto paths: you place the points, it finds the fastest path that keeps clear of everything, and gives you code
to paste into a routine. How it works and why: [DESIGN.md](DESIGN.md).

## Using it

1. Double-click `tools/path-planner.html` (Chrome or Edge).
2. **+ New**, then drag the numbered points on the field. Drag a point's arrow to turn it; double-click to add a point.
3. **Optimize**.
4. **Copy code** and paste it into your routine's class, then drive it:

   ```java
   Path cycle = gardenCycle(poseFactory);
   sequential(
           deadline(follow(follower, cycle),
                    sequential(passed(cycle, GARDEN_CYCLE_INTAKE_ON), robot.intake.intake())),
           robot.shoot());
   ```

To change a path later, **Paste code** (from its `// path-planner` line, or the whole routine file) and copy it again.

To try a path on the robot, run **Planned Path Test** (Tuning). Every run is logged; download logs from
`http://192.168.43.1:8080/pathlogs` and drop them into **Logs** to check and fit the speed model.

## Working on the planner

```sh
cd tools/path-planner
npm install
npm run dev     # the page with hot reload
npm test        # vitest
npm run build   # rebuild tools/path-planner.html; commit it
```

`src/core/` is plain TypeScript (field model, speed model, CMA-ES, export); `src/gui/` is the page.
