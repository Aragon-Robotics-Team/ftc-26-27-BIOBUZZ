# Setting up the sim

Run any Auto in the sim on your laptop and watch it in AdvantageScope.

1. Install [Android Studio](https://developer.android.com/studio) and open this project in it once. No admin
   access: on Windows, take the .zip from its download page ("No .exe installer") and unzip it in your own folder; on
   a Mac, drag it into the Applications folder inside your home folder.
2. Switch to the `simloop` branch (Android Studio: Git > Branches > `origin/simloop` > Checkout).
3. In the project folder, double-click `setup-sim.bat` (Windows) or `setup-sim.command` (Mac).
   It installs everything in your own folders, so it doesn't need admin. The first time takes a while.
4. Close AdvantageScope, then run every Auto on one field: double-click `replay-auto.bat` (Windows), or type
   `./replay-auto.sh` in Android Studio's Terminal tab (Mac). Press space in AdvantageScope to play.

One Auto, with the balls (Mac: `./replay-auto.sh` instead of `replay-auto.bat`):

```
replay-auto.bat one_cycle_garden_park
replay-auto.bat one_cycle_garden_park --blue
replay-auto.bat --no-sim          reopen the last run without running the sim again
```

The routine names are in `TeamCode/.../routines/Routines.java`, in lower case.
After pulling new code, run the setup again if the sim field looks out of date.

## Your own auto

1. Your branch needs the sim: merge `simloop` into it (Android Studio: Git > Branches > `origin/simloop` >
   Merge into Current).
2. Add your routine to `Routines.java` and take `@Disabled` off it: the sim runs the routines in the Auto menu.
3. `replay-auto.bat josh` runs the routine named `JOSH` in `Routines.java` (Mac: `./replay-auto.sh josh`).

It starts where your routine's `startPose()` says. If a check fails ("Auto should drive off its start", "the two
preloads should tip our hive"), the log still opens: watch it to see what your auto did.
