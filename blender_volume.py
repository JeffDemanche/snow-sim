import bpy
import bmesh

### PARAMETERS TO SET

num_frames = 36
filepath_base = 'D:/CSCI2240/snow-sim/out/particles'
particle_density=0.2
particle_diameter = 0.001
render_as_volume = False
render_as_particles = True
snow_shadows_off = False

###


mesh = bpy.data.meshes.new("mesh")
obj = bpy.data.objects.new("SnowPointCloud", mesh)

scene = bpy.context.scene
bpy.context.collection.objects.link(obj)  # put the object into the scene (link)
bpy.context.view_layer.objects.active = obj  # set as the active object in the scene
obj.select_set(True)  # select object

mesh = bpy.context.object.data

x_max = float("-inf")
x_min = float("inf")
y_max = float("-inf")
y_min = float("inf")
z_max = float("-inf")
z_min = float("inf")

def update_bounds(x, y, z):
    global x_max
    global x_min
    global y_max
    global y_min
    global z_max
    global z_min
    
    if x > x_max:
        x_max = x
    if x < x_min:
        x_min = x
    if y > y_max:
        y_max = y
    if y < y_min:
        y_min = y
    if z > z_max:
        z_max = z
    if z < z_min:
        z_min = z

for frame in range(0, num_frames):
    filepath = filepath_base + "." + str(frame).zfill(5)

    vertsPos = []
    
    if frame == 0:
        with open(filepath) as f:
            for line in f:
                pos_str = line.split(']')[0].split('[')[1]
                x = float(pos_str.split(',')[0])
                z = float(pos_str.split(',')[1])
                y = float(pos_str.split(',')[2])
                update_bounds(x, y, z)
                vertsPos.append((x, y, z))
                
        mesh.from_pydata(vertsPos, [], [])
        basis_key = obj.shape_key_add(from_mix=False)
    else:
        this_key = obj.shape_key_add(from_mix=False)
        with open(filepath) as f:
            vert_index = 0
            for line in f:
                pos_str = line.split(']')[0].split('[')[1]
                x = float(pos_str.split(',')[0])
                z = float(pos_str.split(',')[1])
                y = float(pos_str.split(',')[2])
                update_bounds(x, y, z)
                
                this_key.data[vert_index].co.x = x
                this_key.data[vert_index].co.y = y
                this_key.data[vert_index].co.z = z
                
                this_key.value = 0
                this_key.keyframe_insert("value", frame=frame - 1)
                this_key.value = 1
                this_key.keyframe_insert("value", frame=frame)
                this_key.value = 0
                this_key.keyframe_insert("value", frame=frame + 1)
                
                vert_index += 1
         
if render_as_particles:
    bpy.context.scene.render.engine = 'CYCLES'
    
    mesh = bpy.data.meshes.new('Basic_Sphere')
    part_obj = bpy.data.objects.new("BaseParticle", mesh)
    
    # Add the object into the scene.
    bpy.context.collection.objects.link(part_obj)

    # Construct the bmesh sphere and assign it to the blender mesh.
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm, u_segments=32, v_segments=16, diameter=particle_diameter)
    bm.to_mesh(mesh)
    #bm.free()
    
    # Parent sphere to snow cloud
    part_obj.parent = obj
    # Instance base particles at every point in cloud
    obj.instance_type = "VERTS"
    
    
    part_mat = bpy.data.materials.new("Snow Particles")
    part_mat.use_nodes = True
    part_obj.data.materials.append(part_mat)
    links = part_mat.node_tree.links
    default_shade = part_mat.node_tree.nodes['Principled BSDF']
    default_shade.inputs[5].default_value = 0.2
    default_shade.inputs[17].default_value = [0.35, 0.35, 0.35, 1.0]
    
    if snow_shadows_off: 
        part_obj.cycles_visibility.shadow = False
    
         
if render_as_volume:    
    mesh = bpy.data.meshes.new('Basic_Cube')
    mat_obj = bpy.data.objects.new("SnowBound", mesh)
    mat_obj.display_type = "WIRE"
    
    bpy.context.collection.objects.link(mat_obj)
    bpy.context.view_layer.objects.active = mat_obj
    mat_obj.select_set(True)
    
    # Creates cube that bounds all particles for all frames
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bm.to_mesh(mesh)
    bm.free()
    
    for v in mat_obj.data.vertices:
        if v.co.z == 0.5:
            v.co.z = z_max
        if v.co.z == -0.5:
            v.co.z = z_min
        if v.co.y == 0.5:
            v.co.y = y_max
        if v.co.y == -0.5:
            v.co.y = y_min
        if v.co.x == 0.5:
            v.co.x = x_max
        if v.co.x == -0.5:
            v.co.x = x_min
    
    vol_mat = bpy.data.materials.new("Snow Volume")
    vol_mat.use_nodes = True
    mat_obj.data.materials.append(vol_mat)
    links = vol_mat.node_tree.links
    
    default_shade = vol_mat.node_tree.nodes['Principled BSDF']
    vol_mat.node_tree.nodes.remove(default_shade)
    scatter_bsdf = vol_mat.node_tree.nodes.new("ShaderNodeVolumePrincipled")
    
    mat_output = vol_mat.node_tree.nodes['Material Output']
    links.new(scatter_bsdf.outputs["Volume"], mat_output.inputs["Volume"])
    
    multiply = vol_mat.node_tree.nodes.new("ShaderNodeMath")
    multiply.operation = "MULTIPLY"
    multiply.inputs[1].default_value = particle_density
    links.new(multiply.outputs["Value"], scatter_bsdf.inputs["Density"])

    point_density = vol_mat.node_tree.nodes.new("ShaderNodeTexPointDensity")
    point_density.point_source = "OBJECT"
    point_density.object = obj
    links.new(point_density.outputs["Density"], multiply.inputs[0])
    
