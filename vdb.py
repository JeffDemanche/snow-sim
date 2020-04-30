# This needs pyopenvdb to be available in order to be run. That requires OpenVDB
# be built on the machine this is running on.

import pyopenvdb as vdb
import os

num_frames = 60

# Will look for ordered grid.***** files in dir_path
dir_path = os.path.dirname(os.path.realpath(__file__))
dir_path += "/grids"

out_path = "/mnt/d/CSCI2240/snow-vdb/openvdb/vdb_out/"

for frame in range(0, num_frames):
    density_grid = vdb.FloatGrid()
    density_grid.name = "density"

    v_grid = vdb.Vec3SGrid()
    v_grid.name = "v"

    density_accessor = density_grid.getAccessor()
    v_accessor = v_grid.getAccessor()

    filepath = dir_path + "/grid." + str(frame).zfill(5)

    with open(filepath) as f:
        for line in f:
            if (" " in line) and ("[" in line) and ("]" in line) and ("m:" in line):
                index = line.split(" ")[0].split("[")[1].split("]")[0]
                i = int(index.split(",")[0])
                j = int(index.split(",")[1])
                k = int(index.split(",")[2])

                density = float(line.split("density:")[1].split(" ")[0])
                velocity = line.split("v:")[1].split("[")[1].split("]")[0]
                vel_x = float(velocity.split(",")[0])
                vel_y = float(velocity.split(",")[1])
                vel_x = float(velocity.split(",")[2])

                ijk = (i, j, k)

                density_accessor.setValueOn(ijk, density)
                v_accessor.setValueOn(ijk, (vel_x, vel_y, vel_z))

    del density_accessor
    vdb.write(out_path + "vdb" + str(frame).zfill(5) + ".vdb", grids=[density_grid, v_grid])
