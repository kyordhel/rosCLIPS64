rosCLIPS64
==========

rosCLIPS64 is a package that integrates CLIPS 6.4 with ROS in pure C/C++.

This MWE allows you to load CLIPS and interact with it using two string topics: `clips_in` and `clips_out`. You can load `clp` files and/or inject code
directly using `clips_in`, then read the output in `clips_out`.
It won't launch C functions or trigger stuff, or do anything fancy.
All that pumped up stuff is future work...
for someone else to do...
if ever...

The package has two nodes:
- console: whatever you type, the console sends it to `clips_in`, and whatever lands in `clips_out` you'll se on the screen.
- rosclips64: The beast to tame, CLIPS itself!

**Important:** This version uses a precompiled library of CLIPS 6.4 (see lib).
The shared object was built in Ubuntu 20.04 using g++ 9.4.0 with the C++11 standard.
It may not work in other OSs.


How to run
----------
1. Clone the repo inside your `catkin_ws/src` directory
2. Run `catkin_make` after sourcing
3. Open another two terminals and ROS-source them
4. Launch *roscore* in terminal 3
5. Launch *console* in terminal 2 with `roslaunch rosclip64 console`
6. Run `roscd rosclip64`
  - This step is optional but necessary for the program to find and load `cubes.clp`
7. Launch *rosclip64* with `roslaunch rosclip64 console`
8. Inspect the results in both *rosclip64* and *console*
  - Send `(facts)` from *console*
  - Send `(rules)` from *console*
  - Send `(reset)` and then `(run)` from *console*
  - Keep at it.


ToDo
----
- Document the code
- Properly setup the bloody routers
- Check there are no memory leaks
- Add several functions to work as services
- Maybe add clips sources so the shared library builds along with this package
- Refractor the CLIPSCore wrapper because right now is patchy and makes no sense (shame on me but don't judge me, I did what I was told!)
