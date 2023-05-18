import math
import numpy as np

def translate(pos):
    x,y,z = pos
    return np.array([
        [1,0,0,0],
        [0,1,0,0],
        [0,0,1,0],
        [x,y,z,1]
    ])


def rotate_x(a):
    return np.array([
        [1, 0, 0, 0],
        [0, math.cos(a), math.sin(a), 0],
        [0, -math.sin(a), math.cos(a), 0],
        [0, 0, 0, 1]
    ])


def rotate_y(a):
    return np.array([
        [math.cos(a), 0, -math.sin(a), 0],
        [0, 1, 0, 0],
        [math.sin(a), 0, math.cos(a), 0],
        [0, 0, 0, 1]
    ])


def rotate_z(a):
    return np.array([
        [math.cos(a), math.sin(a), 0, 0],
        [-math.sin(a), math.cos(a), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])


def scale(n):
    return np.array([
        [n, 0, 0, 0],
        [0, n, 0, 0],
        [0, 0, n, 0],
        [0, 0, 0, 1]
    ])

def quaternion_from_euler_deg(v):
    return quaternion_from_euler_rad(np.radians(v))

def quaternion_from_euler_rad(v):
    x,y,z = v
    cr = math.cos(x * 0.5)
    sr = math.sin(x * 0.5)
    cp = math.cos(y * 0.5)
    sp = math.sin(y * 0.5)
    cy = math.cos(z * 0.5)
    sy = math.sin(z * 0.5)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy
    ])

def quaternion_to_euler_deg(q):
    roll, pitch, yaw = quaternion_to_euler_rad(q)
    return np.degrees(np.array([roll, pitch, yaw]))

def quaternion_to_euler_rad(q):
    # roll (x-axis rotation)
    sinr_cosp = 2 * (q[3] * q[0] + q[1] * q[2])
    cosr_cosp = 1 - 2 * (q[0] * q[0] + q[1] * q[1])
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (q[3] * q[1] - q[2] * q[0])
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (q[3] * q[2] + q[0] * q[1])
    cosy_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw])

def vector_IntersectPlane(plane_p, plane_n, lineStart, lineEnd):
    """
    Point on plain, normal to the plain,...
    return intersection 
    """
    plane_n = plane_n / np.linalg.norm(plane_n)
    plane_d = -np.dot(plane_n, plane_p)
    ad = np.dot(lineStart, plane_n)
    bd = np.dot(lineEnd, plane_n)
    if np.abs(bd - ad) < 1e-8:
        return None
    t = (-plane_d - ad) / (bd - ad)
    lineStartToEnd = lineEnd - lineStart
    lineToIntersect = lineStartToEnd * t
    return lineStart + lineToIntersect

def normalize(v):
        return v / np.sqrt(np.sum(v**2)) if np.sqrt(np.sum(v**2)) >= 1e-6 else v

def clip_tri_plane(plane_p, plane_n, in_tri, in_vert, n_vert):
    # Make sure plane normal is indeed normal
    plane_n = normalize(plane_n)

    new_tris = []
    new_vert = []

    # Function to compute signed shortest distance from point to plane
    def dist(p):
        p = normalize(p)
        return np.dot(plane_n, p) - np.dot(plane_n, plane_p)

    # Create two temporary storage arrays to classify points either side of plane
    # If distance sign is positive, point lies on "inside" of plane
    inside_points = []
    inside_verts = []
    outside_points = []
    outside_verts = []

    # Get signed distance of each point in triangle to plane
    d0 = dist(in_vert[0])
    d1 = dist(in_vert[1])
    d2 = dist(in_vert[2])

    if d0 >= 0:
        inside_points.append(in_tri[0])
        inside_verts.append(in_vert[0])
    else:
        outside_points.append(in_tri[0])
        outside_verts.append(in_vert[0])
    if d1 >= 0:
        inside_points.append(in_tri[1])
        inside_verts.append(in_vert[1])
    else:
        outside_points.append(in_tri[1])
        outside_verts.append(in_vert[1])
    if d2 >= 0:
        inside_points.append(in_tri[2])
        inside_verts.append(in_vert[2])
    else:
        outside_points.append(in_tri[2])
        outside_verts.append(in_vert[2])

    # Now classify triangle points, and break the input triangle into 
    # smaller output triangles if required. There are four possible
    # outcomes...

    if len(inside_points) == 0:
        # All points lie on the outside of plane, so clip whole triangle
        # It ceases to exist
        return [], [],[]  # No returned triangles are valid

    if len(inside_points) == 3:
        # All points lie on the inside of plane, so do nothing
        # and allow the triangle to simply pass through
        out_tri1 = in_tri
        return [out_tri1], new_tris, new_vert  # Just the one returned original triangle is valid no new tris or verts

    if len(inside_points) == 1 and len(outside_points) == 2:
        # Triangle should be clipped. As two points lie outside
        # the plane, the triangle simply becomes a smaller triangle

        # Copy appearance info to new triangle
        out_tri1 = in_tri.copy()

        # The inside point is valid, so keep that...
        out_tri1[0] = inside_points[0]

        # but the two new points are at the locations where the 
        # original sides of the triangle (lines) intersect with the plane
        nv1 = vector_IntersectPlane(plane_p, plane_n, inside_verts[0], outside_verts[0])
        nv2 = vector_IntersectPlane(plane_p, plane_n, inside_verts[0], outside_verts[1])
        if nv1 is None or nv2 is None :
            return [] , [] , []
        new_vert.append( np.array([*nv1,1.0]))
        new_vert.append( np.array([*nv2,1.0]))
        out_tri1[1] = n_vert
        out_tri1[2] = n_vert + 1

        return [out_tri1],new_tris ,new_vert  # Return the newly formed single triangle

    if len(inside_points) == 2 and len(outside_points) == 1:
        # Triangle should be clipped. As two points lie inside the plane,
        # the clipped triangle becomes a "quad". Fortunately, we can
        # represent a quad with two new triangles

        # Copy appearance info to new triangles
        out_tri1 = in_tri.copy()
        out_tri2 = in_tri.copy()

        # The first triangle consists of the two inside points and a new
        # point determined by the location where one side of the triangle
        # intersects with the plane
        out_tri1[0] = inside_points[0]
        out_tri1[1] = inside_points[1]
        out_tri1[2] = n_vert
        nv1 = vector_IntersectPlane(plane_p, plane_n, inside_verts[0], outside_verts[0])
        if nv1 is None:
            return [] , [] , []
        new_vert.append(np.array( [*nv1,1.0]))

        # The second triangle is composed of one of the inside points, a
        # new point determined by the intersection of the other side of the 
        # triangle and the plane, and the newly created point above
        out_tri2[0] = inside_points[1]
        out_tri2[1] = out_tri1[2]
        out_tri2[2] = n_vert + 1
        nv2 = vector_IntersectPlane(plane_p, plane_n, inside_verts[1], outside_verts[0])
        if  nv2 is None :
            return [] , [] , []
        new_vert.append(np.array([*nv2,1.0]))
        
        return [out_tri1, out_tri2],new_tris ,new_vert   # Return two newly formed triangles which form a quad

