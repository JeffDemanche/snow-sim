import pyopenvdb as vdb
import os
import sys

argv = sys.argv[1:]

if (len(argv) < 2):
    print("Arguments: <num_frames> <output_dir> [-f(lip y and z)]")
    exit(0)

num_frames = int(argv[0])
out_path = str(argv[1])
flip_y_z = len(argv) > 2 and argv[2] == "-f"

# Will look for ordered grid.***** files in dir_path
dir_path = os.path.dirname(os.path.realpath(__file__))
dir_path += "/grids"

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
            if (" " in line) and ("[" in line) and ("]" in line) and ("density:" in line):
                index = line.split(" ")[0].split("[")[1].split("]")[0]
                i = int(index.split(",")[0])
                j = int(index.split(",")[1])
                k = int(index.split(",")[2])
                if (flip_y_z):
                    j = int(index.split(",")[2])
                    k = int(index.split(",")[1])

                density = float(line.split("density:")[1].split(" ")[0])
                velocity = line.split("v:")[1].split("[")[1].split("]")[0]
                vel_x = float(velocity.split(",")[0])
                vel_y = float(velocity.split(",")[1])
                vel_z = float(velocity.split(",")[2])
                if (flip_y_z):
                    vel_y = float(velocity.split(",")[2])
                    vel_z = float(velocity.split(",")[1])

                ijk = (i, j, k)

                density_accessor.setValueOn(ijk, density)
                v_accessor.setValueOn(ijk, (vel_x, vel_y, vel_z))

    del density_accessor
    vdb.write(out_path + "/vdb" + str(frame).zfill(5) + ".vdb", grids=[density_grid, v_grid])
