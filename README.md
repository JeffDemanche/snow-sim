# Snow Simulation

[Original research paper](https://www.math.ucla.edu/~jteran/papers/SSCTS13.pdf)

[Paper with more implementation details](https://pdfs.semanticscholar.org/df01/a2bd3f3936c6024d0ae90b57bd02fa2b7435.pdf?fbclid=IwAR1f5QhNSPvMw1cTX1axYkfrhdq4LskVqkSE_ttsOkiKmKXV72CNnkuBnFE)

## Bringing simulation into Blender

We wrote a script to bring the outputted files from our sim into Blender. The
application exports a series of files to a specified output directory. Each of
these is a point cloud of particles for a specific frame. The blender_volume.py
script is run inside Blender to set up a simple scene using those points clouds.

This was written using Blender 2.8. Older versions might not work.

Open with the following steps:

1. Open Blender.

2. The default cube should be selected by default. Delete it.

3. Go to the tab at the top of the GUI that says "Scripting."

4. In the big middle panel of this view, find the button that says "Open" at the
top. Open the blender_volume.py script.

5. There are a few fields at the top of the script you need to set before
running it.

6. Click "Run Script" at the top-right of the panel.

7. This should bring in the point cloud and cube. Go to the "Animation" tab now
and you can scrub through the timeline to see the particle animation.

8. You can go to the "Shading" tab to change the shading network properties. By
default the script makes a simple volumetric output. Press F12 to render.
