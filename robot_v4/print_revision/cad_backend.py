"""Millimetre/Z-up CadQuery backend for the archived V4 geometry.

No Fusion document is opened or modified. Body wrappers deliberately keep only
the tiny interface used by the ported geometry recipe.
"""
import cadquery as cq

NEW_BODY, JOIN, CUT = 'new', 'join', 'cut'


class Body:
    def __init__(self, shape, name, material, parent):
        self.shape, self.name, self.material, self.parent = shape, name, material, parent


class Part:
    def __init__(self, parent=None, name='root'):
        self.name, self.children, self.body = name, [], None
        if parent is not None:
            parent.children.append(self)

    def bodies(self):
        return ([self.body] if self.body else []) + [b for c in self.children for b in c.bodies()]


def part(parent, name):
    return Part(parent, name)


group = part


def apply(comp, name, shape, operation, body=None, mat=None):
    if operation == NEW_BODY:
        if comp.body is not None:
            raise ValueError('A component must contain one solid: ' + comp.name)
        comp.body = Body(shape.clean(), name, mat, comp)
    else:
        target = body or comp.body
        target.shape = (target.shape.fuse(shape) if operation == JOIN else target.shape.cut(shape)).clean()
    return comp


def block(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1-x0, y1-y0, z1-z0, cq.Vector(x0, y0, z0))


def box(parent, name, x0, x1, y0, y1, z0, z1, mat):
    return apply(part(parent, name), name, block(x0,x1,y0,y1,z0,z1), NEW_BODY, mat=mat)


def prism(loops, axis, start, end):
    plane = cq.Plane(origin=(start,0,0), xDir=(0,1,0), normal=(1,0,0)) if axis == 'x' else cq.Plane(origin=(0,start,0), xDir=(1,0,0), normal=(0,1,0))
    shapes = []
    for kind, data in loops:
        wp = cq.Workplane(plane)
        if axis == 'y':
            data = [(x,-z) for x,z in data] if kind == 'poly' else (data[0],-data[1],data[2])
        if kind == 'poly':
            wp = wp.polyline(data).close()
        else:
            a,b,r = data
            wp = wp.center(a,b).circle(r)
        shapes.append(wp.extrude(end-start).val())
    # Archived recipe always lists the outside first and all bores afterwards.
    shape = shapes[0]
    for hole in shapes[1:]:
        shape = shape.cut(hole)
    return shape


def extrude_yz(comp, name, loops, x0, x1, operation, body=None):
    return apply(comp,name,prism(loops,'x',x0,x1),operation,body)


def extrude_xz_poly(comp,name,poly,y0,y1,operation,body=None):
    return apply(comp,name,prism([('poly',poly)],'y',y0,y1),operation,body)


def extrude_xz(comp,name,circles,y0,y1,operation,body=None):
    # All circles in this helper are independent cuts, not nested loops.
    for circle in circles:
        apply(comp,name,prism([('circle',circle)],'y',y0,y1),operation,body)
    return comp


def solid_x(parent,name,loops,x0,x1,mat):
    comp = part(parent,name)
    extrude_yz(comp,name,loops,x0,x1,NEW_BODY)
    comp.body.material = mat
    return comp,comp.body


def cyl_x(parent,name,x0,x1,y,z,r,mat,bore=None):
    loops=[('circle',(y,z,r))]
    if bore: loops.append(('circle',(y,z,bore)))
    return solid_x(parent,name,loops,x0,x1,mat)


def cut_box(comp,body,name,x0,x1,y0,y1,z0,z1):
    return apply(comp,name,block(x0,x1,y0,y1,z0,z1),CUT,body)


def join_box(comp,body,name,x0,x1,y0,y1,z0,z1):
    return apply(comp,name,block(x0,x1,y0,y1,z0,z1),JOIN,body)


def bore(body, centre, direction, r, before, after):
    direction = cq.Vector(*direction).normalized()
    start = cq.Vector(*centre)-direction*before
    body.shape=body.shape.cut(cq.Solid.makeCylinder(r,before+after,start,direction)).clean()


def cut_z(comp,body,name,x,y,r,z0,z1):
    bore(body,(x,y,z0),(0,0,1),r,0,z1-z0)


def holes_on_face(comp,body,name,normal_fusion,centres,r,depth):
    nx,ny,nz=normal_fusion
    normal=(nx,-nz,ny)
    for centre in centres: bore(body,centre,normal,r,depth,depth)


def face_holes(comp,body,name,centres,r,depth):
    for centre in centres: bore(body,centre,(0,20/28,1),r,depth,depth)


def find_body(root,name):
    return next(b for b in root.bodies() if b.name == name)
