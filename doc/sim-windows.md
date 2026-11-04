# The sim on Windows

Run any Auto in the sim on your laptop and watch it in AdvantageScope.

1. Install [Android Studio](https://developer.android.com/studio) and open this project in it once.
2. Switch to the `simloop` branch (Android Studio: Git > Branches > `origin/simloop` > Checkout).
3. Double-click `setup-sim.bat` in the project folder. Say yes when Windows asks to install things.
   The first time takes a while.
4. Close AdvantageScope, then double-click `replay-auto.bat`. It runs every Auto and opens them on one field.
   Press space to play.

One Auto, with the balls, from a terminal in the project folder:

```
replay-auto.bat one_cycle_garden_park
replay-auto.bat one_cycle_garden_park --blue
replay-auto.bat --no-sim          reopen the last run without running the sim again
```

The routine names are in `TeamCode/.../routines/Routines.java`, in lower case.
After pulling new code, run `setup-sim.bat` again if the sim field looks out of date.
