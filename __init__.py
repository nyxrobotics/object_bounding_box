import math
import random
import time

import bmesh
import bpy
import numpy as np
from bpy.props import BoolProperty
from bpy.props import EnumProperty
from bpy.props import FloatProperty
from bpy.props import IntProperty
from mathutils import Matrix
from mathutils import Vector

bl_info = {
    "name": "Object Bounding Box",
    "author": "Patrick R. Moore",
    "version": (0, 2),
    "blender": (2, 82, 0),
    "location": "View3D > Add > Mesh > New Object",
    "description": "Adds new cube which is minimum bounding box!",
    "warning": "",
    "wiki_url": "",
    "category": "Add Mesh"
}


def bbox_orient(bme_verts, mx):
    if hasattr(bme_verts[0], 'co'):
        verts = [mx @ v.co for v in bme_verts]
    else:
        verts = [mx @ v for v in bme_verts]

    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    zs = [v[2] for v in verts]

    return (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))


def bbox_vol(box):
    V = (box[1] - box[0]) * (box[3] - box[2]) * (box[5] - box[4])
    return V


def box_cords(box):
    cords = [
        Vector((box[0], box[2], box[4])),
        Vector((box[0], box[2], box[5])),
        Vector((box[0], box[3], box[4])),
        Vector((box[0], box[3], box[5])),
        Vector((box[1], box[2], box[4])),
        Vector((box[1], box[2], box[5])),
        Vector((box[1], box[3], box[4])),
        Vector((box[1], box[3], box[5])),
    ]
    return cords


def main(context, rand_sample, spin_res, make_sphere):
    start = time.time()

    world_mx = context.object.matrix_world
    scale = world_mx.to_scale()
    trans = world_mx.to_translation()

    tr_mx = Matrix.Identity(4)
    sc_mx = Matrix.Identity(4)

    tr_mx.translation = trans
    sc_mx[0][0], sc_mx[1][1], sc_mx[2][2] = scale[0], scale[1], scale[2]
    r_mx = world_mx.to_quaternion().to_matrix().to_4x4()

    me = context.object.data
    bme = bmesh.new()
    bme.from_mesh(me)

    convex_hull = bmesh.ops.convex_hull(
        bme, input=bme.verts, use_existing_faces=True)
    total_hull = convex_hull['geom']

    hull_verts = [item for item in total_hull if isinstance(
        item, bmesh.types.BMVert)]

    min_mx = Matrix.Identity(4)
    min_box = bbox_orient(hull_verts, min_mx)
    min_V = bbox_vol(min_box)
    min_axis = Vector((0, 0, 1))
    min_angle = 0
    axes = []
    for i in range(rand_sample):
        u = random.random()
        v = random.random()

        theta = math.pi * u
        phi = math.acos(2 * v - 1)

        x = math.cos(theta) * math.sin(phi)
        y = math.sin(theta) * math.sin(phi)
        z = math.cos(phi)

        axis = Vector((x, y, z))
        axes.append(axis)
        for n in range(spin_res):
            angle = math.pi / 2 * n / spin_res
            rot_mx = Matrix.Rotation(angle, 4, axis)

            box = bbox_orient(hull_verts, rot_mx)
            test_V = bbox_vol(box)

            if test_V < min_V:
                min_V = test_V
                min_axis = axis
                min_angle = angle
                min_box = box
                min_mx = rot_mx

    elapsed_time = time.time() - start
    print('Did %i iterations in %f seconds' %
          (rand_sample * spin_res, elapsed_time))
    print("Final volume %f" % bbox_vol(min_box))

    box_verts = box_cords(min_box)
    bpy.ops.mesh.primitive_cube_add()

    fmx = tr_mx @ r_mx @ min_mx.inverted() @ sc_mx
    context.object.matrix_world = fmx
    context.object.display_type = 'BOUNDS'
    for i, v in enumerate(box_verts):
        context.object.data.vertices[i].co = v

    if make_sphere:
        sample_sphere = bmesh.new()
        for ax in axes:
            sample_sphere.verts.new(ax)

        sphere_me = bpy.data.meshes.new('Bound Samples')
        dest_ob = bpy.data.objects.new('Bound Samples', sphere_me)
        sample_sphere.to_mesh(sphere_me)
        context.collection.objects.link(dest_ob)
        sample_sphere.free()

    bme.free()


def main_SVD(context, down_sample, method, spin_res, make_box):
    start = time.time()

    world_mx = context.object.matrix_world
    scale = world_mx.to_scale()
    trans = world_mx.to_translation()

    tr_mx = Matrix.Identity(4)
    sc_mx = Matrix.Identity(4)

    tr_mx.translation = trans
    sc_mx[0][0], sc_mx[1][1], sc_mx[2][2] = scale[0], scale[1], scale[2]
    r_mx = world_mx.to_quaternion().to_matrix().to_4x4()

    me = context.object.data
    bme = bmesh.new()
    bme.from_mesh(me)

    convex_hull = bmesh.ops.convex_hull(
        bme, input=bme.verts, use_existing_faces=True)
    total_hull = convex_hull['geom']

    hull_verts = [item for item in total_hull if isinstance(
        item, bmesh.types.BMVert)]

    vert_data = [v.co for v in hull_verts]
    v0 = np.array(vert_data, dtype=np.float64, copy=True).T

    t0 = -np.mean(v0, axis=1)
    v0 += t0.reshape(-1, 1)

    U, s, V = np.linalg.svd(v0, full_matrices=True)

    rmx = Matrix.Identity(4)
    for i in range(3):
        rmx[i][:3] = V[i]

    min_box = bbox_orient(vert_data, rmx)
    min_vol = bbox_vol(min_box)
    min_mx = rmx

    X = Vector(V[0])
    Y = Vector(V[1])
    Z = Vector(V[2])

    for n in range(2 * spin_res):
        angle = math.pi * n / (2 * spin_res)
        rmx = Matrix.Identity(4)

        if method == 'pca_x':
            rmx[0][:3] = X

            y = math.cos(angle) * Y + math.sin(angle) * Z
            rmx[1][:3] = y.normalized()

            z = -math.sin(angle) * Y + math.cos(angle) * Z
            rmx[2][:3] = z.normalized()

        elif method == 'pca_y':
            x = math.cos(angle) * X - math.sin(angle) * Z
            rmx[0][:3] = x.normalized()
            rmx[1][:3] = Y
            z = math.sin(angle) * X + math.cos(angle) * Z
            rmx[2][:3] = z.normalized()

        else:
            x = math.cos(angle) * X + math.sin(angle) * Y
            rmx[0][:3] = x.normalized()
            y = -math.sin(angle) * X + math.cos(angle) * Y
            rmx[1][:3] = y.normalized()
            rmx[2][:3] = Z

        box = bbox_orient(vert_data, rmx)
        test_V = bbox_vol(box)
        if test_V < min_vol:
            min_box = box
            min_mx = rmx
            min_vol = test_V

        if make_box:
            box_verts = box_cords(box)
            bpy.ops.mesh.primitive_cube_add()
            context.object.matrix_world = rmx.transposed().inverted() @ world_mx
            context.object.display_type = 'BOUNDS'
            for i, v in enumerate(box_verts):
                context.object.data.vertices[i].co = v

    elapsed_time = time.time() - start
    print('Found bbox of volume %f in %f seconds with SVD followed by rotating calipers' % (
        min_vol, elapsed_time))

    box_verts = box_cords(min_box)
    bpy.ops.mesh.primitive_cube_add()
    fmx = tr_mx @ r_mx @ min_mx.inverted() @ sc_mx

    context.object.matrix_world = fmx
    context.object.display_type = 'BOUNDS'
    for i, v in enumerate(box_verts):
        context.object.data.vertices[i].co = v

    bme.free()


class OBJECT_OT_min_bound_box(bpy.types.Operator):
    """Find approximate minimum bounding box of object"""
    bl_idname = "object.min_bound_box"
    bl_label = "Min Bounding Box"

    sample_vis: BoolProperty(
        name="Visualize Sample",
        description='Add a sphere to the scene showing random direction sample',
        default=False,
    )

    make_box: BoolProperty(
        name="Visualize Boxes",
        description='Add a cube for all bounding boxes tried. VERY MESSY!',
        default=False,
    )
    area_sample: IntProperty(
        name="Area Samples",
        description='Number of random directions to test calipers in',
        default=200
    )
    angular_sample: IntProperty(
        name="Angular Sample",
        description='Angular step to rotate calipers. 90 = 1 degree steps, 180 = 1/2 degree steps',
        default=50
    )

    method_enum = [
        ('brute_force', "BRUTE FORCE", 'Checks a bunch of random boxes'),
        ('pca_y', 'PCAY', 'Good for linear objects'),
        ('pca_x', 'PCAX', 'Good for flat objects'),
        ('pca_z', 'PCAZ', 'Good for some things')
    ]

    method: EnumProperty(
        name="Method",
        description="Min BBox method to use",
        items=method_enum,
        default='brute_force',
    )

    @classmethod
    def poll(cls, context):
        return context.active_object is not None and context.active_object.type == 'MESH'

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "sample_vis")
        layout.prop(self, "make_box")
        layout.prop(self, "area_sample")
        layout.prop(self, "angular_sample")
        layout.prop(self, "method")

    def execute(self, context):
        if self.method == 'brute_force':
            main(context, self.area_sample, self.angular_sample, self.sample_vis)
        else:
            main_SVD(context, 1, self.method,
                     self.angular_sample, self.make_box)
        return {'FINISHED'}


def register():
    bpy.utils.register_class(OBJECT_OT_min_bound_box)


def unregister():
    bpy.utils.unregister_class(OBJECT_OT_min_bound_box)


if __name__ == "__main__":
    register()
