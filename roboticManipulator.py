# I'd like a centralised arm class that can do everything I need it to.
# It needs:
# - partial and complete forward kinematics
# - link collider data

# needed for infs
import math
import numpy as np


np.set_printoptions(precision=8)
np.set_printoptions(suppress=True)


# main class
class RoboticManipulator:
    # Internal variables
    AXIS_X = 0
    AXIS_Y = 1
    AXIS_Z = 2

    # internal helper methods
    # functions to generate 4x4 transformation matrices
    @staticmethod
    def matrix_rotateX(a):
        return np.array([
            [1,0,0,0],
            [0, np.cos(a), -np.sin(a), 0],
            [0, np.sin(a), np.cos(a), 0],
            [0,0,0,1]
        ])

    @staticmethod
    def matrix_rotateY(a):
        return np.array([
            [np.cos(a), 0, np.sin(a), 0],
            [0,1,0,0],
            [-np.sin(a), 0, np.cos(a), 0],
            [0,0,0,1]
        ])

    @staticmethod
    def matrix_rotateZ(a):
        return np.array([
            [np.cos(a), -np.sin(a), 0, 0],
            [np.sin(a), np.cos(a), 0, 0],
            [0,0,1,0],
            [0,0,0,1]
        ])

    # now the displacement matrices
    @staticmethod
    def matrix_displacement(x,y,z):
        return np.array([
            [1,0,0,x],
            [0,1,0,y],
            [0,0,1,z],
            [0,0,0,1]
        ])

    # identity matrix just in case
    MATRIX_IDENTITY = np.array([
        [1,0,0,0],
        [0,1,0,0],
        [0,0,1,0],
        [0,0,0,1]
    ])

    # we also want vector rotations
    @staticmethod
    def vector_rotateX(a):
        return np.array([
            [1,0,0],
            [0, np.cos(a), -np.sin(a)],
            [0, np.sin(a), np.cos(a)],
        ])

    @staticmethod
    def vector_rotateY(a):
        return np.array([
            [np.cos(a), 0, np.sin(a)],
            [0,1,0],
            [-np.sin(a), 0, np.cos(a)],
        ])

    @staticmethod
    def vector_rotateZ(a):
        return np.array([
            [np.cos(a), -np.sin(a), 0],
            [np.sin(a), np.cos(a), 0],
            [0,0,1],
        ])

    # Internal Subclasses:
    # conforms roughly to the ROS URDF model, unnecessary attributes ignored

    # describes the joint itself, its rotation, and the displacement from the previous joint
    class Joint:
        def __init__(self, XOffset, YOffset, ZOffset, XRotation, YRotation, ZRotation, axis, angle=0, parentJoint=None, children=None, links=None, upperLimit=math.inf, lowerLimit=-math.inf, name="Unnamed Joint"):
            # STATIC VARIABLES:
            # offsets from previous joint
            self.XOffset = XOffset
            self.YOffset = YOffset
            self.ZOffset = ZOffset

            self.XRotation = XRotation
            self.YRotation = YRotation
            self.ZRotation = ZRotation

            self.axis = axis

            self.upperLimit = upperLimit
            self.lowerLimit = lowerLimit

            self.name = name

            # DYNAMIC VARIABLES:
            self.angle = angle

            # CHILDREN AND LINKS:
            # self.children = children
            if children is not None:
                self.children = children
            else:
                self.children = []

            self.parentJoint = parentJoint

            # self.links = links
            if links is not None:
                self.links = links
            else:
                self.links = []

            for l in self.links:
                if l.parentJoint is not self:
                    l.set_parent_joint(self)

            # auto add the joint to the parent's children array
            if parentJoint is not None:
                self.set_parent_joint(parentJoint)

            # NON-USER SET VARIABLES:
            # initialised as None, will be changed later
            self.localMatrix = None
            self.combinedMatrix = None

            # CHANGED FLAGS
            # used to track if the matrices need to be re-calculated
            self.FLAG_local_matrix_updated = False
            self.FLAG_combined_matrix_updated = False

        def set_parent_joint(self, parent):
            # remove self from current parent's children array
            if self.parentJoint is not None and self in self.parentJoint.children:
                self.parentJoint.children.remove(self)

            # THEN change it
            self.parentJoint = parent

            # THEN put in new parent's children array
            parent.children.append(self)

        def mark_as_changed(self):
            # own flags
            self.FLAG_local_matrix_updated = False
            self.FLAG_combined_matrix_updated = False

            # children's flags
            for i in self.children:
                i.mark_as_changed()

            # link flags
            for l in self.links:
                l.mark_as_changed()

        def set_angle(self, angle):
            if self.axis is None:
                print(f"WARNING - attempting to rotate joint with None axis")

            if angle < self.lowerLimit:
                print(f"WARNING - set_angle: angle {angle} < lower limit {self.lowerLimit}. setting to lower limit ")
                self.angle = self.lowerLimit

            elif angle > self.upperLimit:
                print(f"WARNING - set_angle: angle {angle} > upper limit {self.upperLimit}. setting to upper limit ")
                self.angle = self.upperLimit

            else:
                # no issues, just set it
                self.angle = angle
            
            # changing the angle means re-calculating its matrices
            self.mark_as_changed()

        def get_position(self):
            m = self.get_combined_matrix()
            return (m[0][3], m[1][3], m[2][3])

        # matrix of JUST THIS JOINT
        def get_local_matrix(self):
            if not self.FLAG_local_matrix_updated:
                # non commutative operations. order matters!
                m = np.matmul(RoboticManipulator.MATRIX_IDENTITY, RoboticManipulator.matrix_displacement(self.XOffset, self.YOffset, self.ZOffset))
                m = np.matmul(m, RoboticManipulator.matrix_rotateX(self.XRotation))
                m = np.matmul(m, RoboticManipulator.matrix_rotateY(self.YRotation))
                m = np.matmul(m, RoboticManipulator.matrix_rotateZ(self.ZRotation))

                # then apply the rotation of the joint
                match self.axis:
                    case RoboticManipulator.AXIS_X:
                        m = np.matmul(m, RoboticManipulator.matrix_rotateX(self.angle))
                    case RoboticManipulator.AXIS_Y:
                        m = np.matmul(m, RoboticManipulator.matrix_rotateY(self.angle))
                    case RoboticManipulator.AXIS_Z:
                        m = np.matmul(m, RoboticManipulator.matrix_rotateZ(self.angle))
                    # if no axis set, joint does not rotate
                    case None:
                        pass
                    case _:
                        raise TypeError(f"ERROR: INVALID AXIS {self.axis}")# TODO: link to logging
                    
                self.localMatrix = m
                self.FLAG_local_matrix_updated = True
            return self.localMatrix

        # matrix for this joint accounting for its parent
        def get_combined_matrix(self):
            if not self.FLAG_combined_matrix_updated:
                # head recursive function getting children
                # stack size issues? unlikely with 1000 limit
                if self.parentJoint is not None:
                    m = np.matmul(self.parentJoint.get_combined_matrix(), self.get_local_matrix())
                else:
                    m = self.get_local_matrix()
                self.combinedMatrix = m
                self.FLAG_combined_matrix_updated = True
            return self.combinedMatrix

        def check_collisions(self, colliders):
            for i in self.links:
                result = i.check_collisions(colliders)
                if result:
                    return result
            return False

    # describes the geometry of the link. used for collision checking
    class Link:
        def __init__(self, XOffset=0, YOffset=0, ZOffset=0, XRotation=0, YRotation=0, ZRotation=0, colliders=None, parentJoint=None, name="Unnamed Link"):
            # offsets from the asociated joint's origin
            self.XOffset = XOffset
            self.YOffset = YOffset
            self.ZOffset = ZOffset

            self.XRotation = XRotation
            self.YRotation = YRotation
            self.ZRotation = ZRotation

            self.name = name
            
            # self.colliders = colliders
            if colliders is not None:
                self.colliders = colliders
            else:
                self.colliders = []

            self.parentJoint = parentJoint

            # CHANGED FLAGS
            # used to track if the matrices need to be re-calculated
            self.FLAG_local_matrix_updated = False
            self.FLAG_combined_matrix_updated = False

            # matrix stuff
            self.localMatrix = None
            self.combinedMatrix = None

            # parent
            if parentJoint is not None:
                self.set_parent_joint(parentJoint)

            # children
            if colliders is not None:
                for c in colliders:
                    c.set_parent_link(self)

        def add_collider(self, collider):
            # allow for common itterable types
            if isinstance(collider, (list, tuple, np.ndarray)):
                for i in collider:
                    self.add_collider(i)
                return

            # check its a collider
            # note that we specifically check that its a SUBclass,
            # the generic Collider class is to be treated as abstract
            if issubclass(type(collider), Collider) and type(collider) is not Collider:
                # and check its not already in the array
                if collider not in self.colliders:
                    collider.set_parent_link(self)
                else:
                    # error out, object already in colliders array
                    raise ValueError(f"Object {collider} already in colliders array")
            else:
                raise TypeError(f"Object {collider} of type {type(collider)}, is not subclass of type {Collider}")

        def check_collisions(self, otherColliders):
            # check if provided a single collider or an itterable of them
            if issubclass(type(otherColliders), Collider):
                # single collider:
                for c1 in self.colliders:
                    result = Collider.check_collisions(c1, otherColliders)
                    if result:
                        return result
                return False
            else:
                # itterable:
                for c1 in self.colliders:
                    for c2 in otherColliders:
                        result = Collider.check_collisions(c1,c2)
                        if result:
                            return result
                return False

        def mark_as_changed(self):
            # own flags
            self.FLAG_local_matrix_updated = False
            self.FLAG_combined_matrix_updated = False

            # collider flags
            for c in self.colliders:
                c.mark_as_changed()

        def get_local_matrix(self):
            if not self.FLAG_local_matrix_updated:
                # displacement first
                m = RoboticManipulator.matrix_displacement(self.XOffset, self.YOffset, self.ZOffset)

                # then rotation
                m = np.matmul(m, RoboticManipulator.matrix_rotateX(self.XRotation))
                m = np.matmul(m, RoboticManipulator.matrix_rotateY(self.YRotation))
                m = np.matmul(m, RoboticManipulator.matrix_rotateZ(self.ZRotation))

                self.localMatrix = m
                self.FLAG_local_matrix_updated = True

            return m


        def get_combined_matrix(self):
            if not self.FLAG_combined_matrix_updated:
                # links should always have a parent joint, but lets future proof this
                if self.parentJoint is None:
                    print("WARNING: getting combined matrix of link with no parent")
                    self.combinedMatrix = self.get_local_matrix()
                    self.FLAG_combined_matrix_updated = True
                else:
                    m = np.matmul(self.parentJoint.get_combined_matrix(), self.get_local_matrix())

                    self.combinedMatrix = m
                    self.FLAG_combined_matrix_updated = True
                    
            return self.combinedMatrix

        def get_position(self):
            m = self.get_combined_matrix()
            return (m[0][3], m[1][3], m[2][3])

        def set_parent_joint(self, parent):
            # remove self from current parent's children array
            if self.parentJoint is not None and self in self.parentJoint.children:
                self.parentJoint.links.remove(self)

            # THEN change it
            self.parentJoint = parent

            # THEN put in new parent's children array
            parent.links.append(self)

    # constructor all the way down here
    def __init__(self, joints):
        self.joints = joints

    # methods
    def check_collisions(self, colliders):
        for i in self.joints:
            result = i.check_collisions(colliders)
            if result:
                return result
        return False

    def writeSTL(self, outputfile):
        # check if outputfile has ".stl" on it
        o = outputfile.split(".")
        if o[-1] != "stl":
            outputfile += ".stl"

        # main process
        with open(outputfile,"w") as f:
            f.write("solid RM\n")
            for j in self.joints:
                for l in j.links:
                    for c in l.colliders:
                        # TODO: add support for spheres 
                        if type(c) is CuboidCollider or type(c) is AlignedCuboidCollider:
                            vs = c.get_verts()
                            # vertices
                            for i in range(len(vs)):
                                # this looks entirely arbirary, that's because it is!
                                # buuuut, it will actually get all the correct tris

                                f.write(" facet normal 0 0 0\n  outer loop\n")
                                f.write(f"   vertex {vs[7][0]} {vs[7][1]} {vs[7][2]}\n")
                                f.write(f"   vertex {vs[1][0]} {vs[1][1]} {vs[1][2]}\n")
                                f.write(f"   vertex {vs[6][0]} {vs[6][1]} {vs[6][2]}\n")
                                f.write("  endloop\n endfacet\n")

                                f.write(" facet normal 0 0 0\n  outer loop\n")
                                f.write(f"   vertex {vs[6][0]} {vs[6][1]} {vs[6][2]}\n")
                                f.write(f"   vertex {vs[4][0]} {vs[4][1]} {vs[4][2]}\n")
                                f.write(f"   vertex {vs[7][0]} {vs[7][1]} {vs[7][2]}\n")
                                f.write("  endloop\n endfacet\n")

                                f.write(" facet normal 0 0 0\n  outer loop\n")
                                f.write(f"   vertex {vs[4][0]} {vs[4][1]} {vs[4][2]}\n")
                                f.write(f"   vertex {vs[6][0]} {vs[6][1]} {vs[6][2]}\n")
                                f.write(f"   vertex {vs[3][0]} {vs[3][1]} {vs[3][2]}\n")
                                f.write("  endloop\n endfacet\n")

                                f.write(" facet normal 0 0 0\n  outer loop\n")
                                f.write(f"   vertex {vs[3][0]} {vs[3][1]} {vs[3][2]}\n")
                                f.write(f"   vertex {vs[5][0]} {vs[5][1]} {vs[5][2]}\n")
                                f.write(f"   vertex {vs[4][0]} {vs[4][1]} {vs[4][2]}\n")
                                f.write("  endloop\n endfacet\n")

                                f.write(" facet normal 0 0 0\n  outer loop\n")
                                f.write(f"   vertex {vs[5][0]} {vs[5][1]} {vs[5][2]}\n")
                                f.write(f"   vertex {vs[3][0]} {vs[3][1]} {vs[3][2]}\n")
                                f.write(f"   vertex {vs[0][0]} {vs[0][1]} {vs[0][2]}\n")
                                f.write("  endloop\n endfacet\n")

                                f.write(" facet normal 0 0 0\n  outer loop\n")
                                f.write(f"   vertex {vs[0][0]} {vs[0][1]} {vs[0][2]}\n")
                                f.write(f"   vertex {vs[2][0]} {vs[2][1]} {vs[2][2]}\n")
                                f.write(f"   vertex {vs[5][0]} {vs[5][1]} {vs[5][2]}\n")
                                f.write("  endloop\n endfacet\n")

                                f.write(" facet normal 0 0 0\n  outer loop\n")
                                f.write(f"   vertex {vs[2][0]} {vs[2][1]} {vs[2][2]}\n")
                                f.write(f"   vertex {vs[0][0]} {vs[0][1]} {vs[0][2]}\n")
                                f.write(f"   vertex {vs[1][0]} {vs[1][1]} {vs[1][2]}\n")
                                f.write("  endloop\n endfacet\n")

                                f.write(" facet normal 0 0 0\n  outer loop\n")
                                f.write(f"   vertex {vs[1][0]} {vs[1][1]} {vs[1][2]}\n")
                                f.write(f"   vertex {vs[7][0]} {vs[7][1]} {vs[7][2]}\n")
                                f.write(f"   vertex {vs[2][0]} {vs[2][1]} {vs[2][2]}\n")
                                f.write("  endloop\n endfacet\n")

                                f.write(" facet normal 0 0 0\n  outer loop\n")
                                f.write(f"   vertex {vs[4][0]} {vs[4][1]} {vs[4][2]}\n")
                                f.write(f"   vertex {vs[5][0]} {vs[5][1]} {vs[5][2]}\n")
                                f.write(f"   vertex {vs[2][0]} {vs[2][1]} {vs[2][2]}\n")
                                f.write("  endloop\n endfacet\n")

                                f.write(" facet normal 0 0 0\n  outer loop\n")
                                f.write(f"   vertex {vs[2][0]} {vs[2][1]} {vs[2][2]}\n")
                                f.write(f"   vertex {vs[7][0]} {vs[7][1]} {vs[7][2]}\n")
                                f.write(f"   vertex {vs[4][0]} {vs[4][1]} {vs[4][2]}\n")
                                f.write("  endloop\n endfacet\n")

                                f.write(" facet normal 0 0 0\n  outer loop\n")
                                f.write(f"   vertex {vs[3][0]} {vs[3][1]} {vs[3][2]}\n")
                                f.write(f"   vertex {vs[6][0]} {vs[6][1]} {vs[6][2]}\n")
                                f.write(f"   vertex {vs[1][0]} {vs[1][1]} {vs[1][2]}\n")
                                f.write("  endloop\n endfacet\n")

                                f.write(" facet normal 0 0 0\n  outer loop\n")
                                f.write(f"   vertex {vs[1][0]} {vs[1][1]} {vs[1][2]}\n")
                                f.write(f"   vertex {vs[0][0]} {vs[0][1]} {vs[0][2]}\n")
                                f.write(f"   vertex {vs[3][0]} {vs[3][1]} {vs[3][2]}\n")
                                f.write("  endloop\n endfacet\n")

            f.write("endsolid RM\n")


# collider superclass
# got a bunch of "abstract" methods, don't want to bloat with actual abstraction package
class Collider:

    def __init__(self):
        if type(self) == Collider:
            raise NotImplementedError("This is an abstract class. Instantiate a subclass instead")
        else:
            raise NotImplementedError(f"function '__init__' not implemented for Collider sublcass {type(self)}")

    def get_bounding_sphere_origin(self):
        if type(self) == Collider:
            raise NotImplementedError("This is an abstract class. Instantiate a subclass instead")
        else:
            raise NotImplementedError(f"function 'get_bounding_sphere_origin' not implemented for Collider sublcass {type(self)}")

    def get_bounding_sphere_radius(self):
        if type(self) == Collider:
            raise NotImplementedError("This is an abstract class. Instantiate a subclass instead")
        else:
            raise NotImplementedError(f"function 'get_bounding_sphere_radius' not implemented for Collider sublcass {type(self)}")

    def get_local_matrix(self):
        if type(self) == Collider:
            raise NotImplementedError("This is an abstract class. Instantiate a subclass instead")
        else:
            raise NotImplementedError(f"function 'get_local_matrix' not implemented for Collider sublcass {type(self)}")

    def get_combined_matrix(self):
        if type(self) == Collider:
            raise NotImplementedError("This is an abstract class. Instantiate a subclass instead")
        else:
            raise NotImplementedError(f"function 'get_combined_matrix' not implemented for Collider sublcass {type(self)}")

    def get_position(self):
        if type(self) == Collider:
            raise NotImplementedError("This is an abstract class. Instantiate a subclass instead")
        else:
            raise NotImplementedError(f"function 'get_position' not implemented for Collider sublcass {type(self)}")

    def mark_as_changed(self):
        if type(self) == Collider:
            raise NotImplementedError("This is an abstract class. Instantiate a subclass instead")
        else:
            raise NotImplementedError(f"function 'mark_as_changed' not implemented for Collider sublcass {type(self)}")
        
    def set_parent_link(self, parent):
        # remove self from current parent's children array
        if self.parentLink is not None:
            self.parentLink.colliders.remove(self)

        # THEN change it
        self.parentLink = parent

        # THEN put in new parent's children array
        parent.colliders.append(self)

    @staticmethod
    def check_collisions(c1,c2):
        # sanity type checking
        if not issubclass(type(c1),Collider):
            raise TypeError(f"Object {c1} is not a subclass of {Collider} ")
        if not issubclass(type(c2),Collider):
            raise TypeError(f"Object {c2} is not a subclass of {Collider} ")
                 
        # check for early False exit with bounding spheres
        bsp1 = c1.get_bounding_sphere_origin()
        bsp2 = c2.get_bounding_sphere_origin()
        dist = np.sqrt(
            (bsp1[0] - bsp2[0]) * (bsp1[0] - bsp2[0]) +
            (bsp1[1] - bsp2[1]) * (bsp1[1] - bsp2[1]) +
            (bsp1[2] - bsp2[2]) * (bsp1[2] - bsp2[2]) 
            )
        if dist > (c1.get_bounding_sphere_radius() + c2.get_bounding_sphere_radius()):
            return False

        # propper check: need the types of both colliders
        cTypes = (type(c1), type(c2))
        c = (c1,c2)

        if cTypes == (SphereCollider, SphereCollider):
            (SP1, SP2) = c
            # calculate distance between spheres. True if less that either radius
            o1 = list(SP1.get_position())
            o2 = list(SP2.get_position())
            dist = np.sqrt(
                (o1[0] - o2[0]) * (o1[0] - o2[0]) +
                (o1[1] - o2[1]) * (o1[1] - o2[1]) +
                (o1[2] - o2[2]) * (o1[2] - o2[2]) 
                )
            
            if dist <= SP1.radius + SP2.radius:
                return (c1.name, c2.name)
            return False


        elif cTypes ==  (SphereCollider, AlignedCuboidCollider) or cTypes == (AlignedCuboidCollider, SphereCollider):
            if type(c1) == SphereCollider:
                (SP, AC,) = c
            else:
                (AC, SP) = c

            # to get the closest point on a cube to a given point, 
            # simply clamp the values of the point to the cuboid's limits
            SPo = list(SP.get_position())
            ACo = list(AC.get_position())
            closest = [
                max(min(SPo[0], ACo[0] + AC.x/2), ACo[0] - AC.x/2),
                max(min(SPo[1], ACo[1] + AC.y/2), ACo[0] - AC.y/2),
                max(min(SPo[2], ACo[2] + AC.z/2), ACo[0] - AC.z/2),
            ]

            # check if the closest point is within the sphere's radius
            dist = np.sqrt (
                (SPo[0] - closest[0]) * (SPo[0] - closest[0]) +
                (SPo[1] - closest[1]) * (SPo[1] - closest[1]) +
                (SPo[2] - closest[2]) * (SPo[2] - closest[2]) 
                )

            if dist <= SP.radius:
                return (c1.name, c2.name)
            return False
            

        elif cTypes == (SphereCollider, CuboidCollider) or cTypes == (CuboidCollider, SphereCollider):
            if type(c1) == SphereCollider:
                (SP, CU,) = c
            else:
                (CU, SP) = c

            # we will treat this similar to the case above,
            # rotate the sphere origin around the cuboid then just pretend its axis aligned 

            # need of offset by cuboid origin to get rotation right
            SPo = list(SP.get_position())
            CUo = list(CU.get_position())
            new_sp_origin  = [
                SPo[0] - CUo[0],
                SPo[1] - CUo[1],
                SPo[2] - CUo[2], 
                ]
            new_sp_origin = np.matmul(new_sp_origin, RoboticManipulator.vector_rotateZ(-CU.zAngle))
            new_sp_origin = np.matmul(new_sp_origin, RoboticManipulator.vector_rotateY(-CU.yAngle))
            new_sp_origin = np.matmul(new_sp_origin, RoboticManipulator.vector_rotateX(-CU.xAngle))
            # then offset back once rotated
            new_sp_origin[0] += CUo[0]
            new_sp_origin[1] += CUo[1]
            new_sp_origin[2] += CUo[2]

            # copied from above:

            # to get the closest point on a cube to a given point, 
            # simply clamp the values of the point to the cuboid's limits    
            closest = [
                max(min(new_sp_origin[0], CUo[0] + CU.x/2), CUo[0] - CU.x/2),
                max(min(new_sp_origin[1], CUo[1] + CU.y/2), CUo[1] - CU.y/2),
                max(min(new_sp_origin[2], CUo[2] + CU.z/2), CUo[2] - CU.z/2),
            ]

            # check if the closest point is within the sphere's radius
            dist = np.sqrt (
                (new_sp_origin[0] - closest[0]) * (new_sp_origin[0] - closest[0]) +
                (new_sp_origin[1] - closest[1]) * (new_sp_origin[1] - closest[1]) +
                (new_sp_origin[2] - closest[2]) * (new_sp_origin[2] - closest[2]) 
                )

            if dist <= SP.radius:
                return (c1.name, c2.name)
            return False
            
            

        elif cTypes == (AlignedCuboidCollider, AlignedCuboidCollider):
            (AC1, AC2) = c
            # we can use a MASSIVELY simplified and optimised version of SAT here
            # there are only 3 axes we need to test and we can get the min and maxes directly

            AC1o = list(AC1.get_position())
            AC2o = list(AC2.get_position())

            # x axis
            if not ( (AC1o[0]-AC1.x <= AC2o[0]+AC2.x and AC1o[0]-AC1.x >= AC2o[0]-AC2.x) or (AC2o[0]-AC2.x <= AC1o[0]+AC1.x and AC2o[0]-AC2.x >= AC1o[0]-AC1.x) ):
                # no overlap, they dont collide
                return False

            # y axis
            if not ( (AC1o[1]-AC1.y <= AC2o[1]+AC2.y and AC1o[1]-AC1.y >= AC2o[1]-AC2.y) or (AC2o[1]-AC2.y <= AC1o[1]+AC1.y and AC2o[1]-AC2.y >= AC1o[1]-AC1.y) ):
                # no overlap, they dont collide
                return False

            # z axis
            if not ( (AC1o[2]-AC1.z <= AC2o[2]+AC2.z and AC1o[2]-AC1.z >= AC2o[2]-AC2.z) or (AC2o[2]-AC2.z <= AC1o[2]+AC1.z and AC2o[2]-AC2.z >= AC1o[2]-AC1.z) ):
                # no overlap, they dont collide
                return False

            # if we get here and havent returned yet, there are no gaps and thus they collide
            return (c1.name, c2.name)


        elif cTypes == (AlignedCuboidCollider, CuboidCollider) or cTypes == (CuboidCollider, AlignedCuboidCollider) or cTypes == (CuboidCollider, CuboidCollider):
            # in this case, both the cuboid collider classes have the same methods so they are interchangable
            # for consistancy
            (C1, C2) = (c1,c2)

            # SAT time!
            # get complete list of face normals
            faceNormals = np.concatenate([C1.get_normals(), C2.get_normals()])
            # check the face normals
            for i in faceNormals:
                c1_min = float("inf")
                c1_max = -float("inf")
                c2_min = float("inf")
                c2_max = -float("inf")
                for c1v in C1.get_verts():
                    d = np.dot(c1v,i)
                    c1_min = min(c1_min,d)
                    c1_max = max(c1_max,d)
                for c2v in C2.get_verts():
                    d = np.dot(c2v,i)
                    c2_min = min(c2_min,d)
                    c2_max = max(c2_max,d)

                # check for gap
                if not ( (c1_min <= c2_max and c1_min >= c2_min) or (c2_min <= c1_max and c2_min >= c1_min) ):
                    # there is a gap. end early
                    return False
                
            # if not exited yet, face normals not enough. need to check edges now
            for a in C1.get_normals():
                for b in C2.get_normals():
                    # we can do this because in a cube the face normals are parallel with the edges
                    # therefore, we can calc the edge axes using the same normals used in the faces
                    axis = np.cross(a,b)
                    # check for 0 vector, with epsilon because floating point
                    if np.all(np.abs(axis) < 0.0001):
                        continue
                    c1_min = float("inf")
                    c1_max = -float("inf")
                    c2_min = float("inf")
                    c2_max = -float("inf")
                    for c1v in C1.get_verts():
                        d = np.dot(c1v,axis)
                        c1_min = min(c1_min,d)
                        c1_max = max(c1_max,d)
                    for c2v in C2.get_verts():
                        d = np.dot(c2v,axis)
                        c2_min = min(c2_min,d)
                        c2_max = max(c2_max,d)

                    # check for gap
                    if not ( (c1_min <= c2_max and c1_min >= c2_min) or (c2_min <= c1_max and c2_min >= c1_min) ):
                        # there is a gap. end early
                        return False
                    
            # if we get this far and we havent exited yet, there is no gap and the cuboids are colliding!
            return (c1.name, c2.name)

        else:
            # should never trigger, add it just in case
            raise TypeError(f"Objects {c1} and {c2} reached collision check else case. This should never happen! Please fix")


class SphereCollider(Collider):

    def __init__(self, origin, radius, parentLink=None, name="Unnamed Sphere Collider"):
        self.origin = origin
        self.radius = radius
        self.name = name
        self.parentLink = parentLink

        self.FLAG_local_matrix_updated = False
        self.FLAG_combined_matrix_updated = False

        self.localMatrix = None
        self.combinedMatrix = None

        if self.parentLink is not None:
            self.set_parent_link(parentLink)

    def mark_as_changed(self):
        self.FLAG_local_matrix_updated = False
        self.FLAG_combined_matrix_updated = False

    def set_parent_link(self, parent):
        super().set_parent_link(parent)

        # should also invalidate matrix flags
        self.FLAG_local_matrix_updated = False
        self.FLAG_combined_matrix_updated = False

    # matrix function stuff before bounding sphere stuff
    # use the combined matrix to calc the bounding sphere
    def get_local_matrix(self):
        if not self.FLAG_local_matrix_updated:
            self.localMatrix = RoboticManipulator.matrix_displacement(*self.origin)
            self.FLAG_local_matrix_updated = True
        return self.localMatrix

    def get_combined_matrix(self):
        if not self.FLAG_combined_matrix_updated:

            # if no parent, local matrix IS combined matrix
            if self.parentLink is None:
                self.combinedMatrix = self.get_local_matrix()
                self.FLAG_combined_matrix_updated = True
                return self.combinedMatrix

            # NOW do normal process
            self.combinedMatrix = np.matmul(
                self.parentLink.get_combined_matrix(),
                self.get_local_matrix()
            )
            self.FLAG_combined_matrix_updated = True
        return self.combinedMatrix

    def get_position(self):
        m = self.get_combined_matrix()
        return(m[0][3], m[1][3], m[2][3])
    
    def get_bounding_sphere_origin(self):
        return self.get_position()
    
    def get_bounding_sphere_radius(self):
        return self.radius


class AlignedCuboidCollider(Collider):

    def __init__(self, origin, x, y, z, parentLink=None, name="Unnamed Aligned Cuboid Collider"):
        self.origin = origin
        self.x = x
        self.y = y
        self.z = z
        self.name = name
        self.parentLink = parentLink

        self.FLAG_local_matrix_updated = False
        self.FLAG_combined_matrix_updated = False

        self.localMatrix = None
        self.combinedMatrix = None

        self.FLAG_verts_updated = False
        self.verts = None

        if self.parentLink is not None:
            self.set_parent_link(self.parentLink)

        # calc the baseVerts first
        # this will always be needed at some point and never change. just do it now
        self.baseVerts = np.array([
            [+x/2, +y/2, +z/2],
            [-x/2, +y/2, +z/2],
            [+x/2, -y/2, +z/2],
            [+x/2, +y/2, -z/2],

            [-x/2, -y/2, -z/2],
            [+x/2, -y/2, -z/2],
            [-x/2, +y/2, -z/2],
            [-x/2, -y/2, +z/2],
        ])

        # bounding radius is equal to the distance to any vertex
        self.bounding_sphere_radius = np.sqrt(
            self.baseVerts[0][0] * self.baseVerts[0][0] +
            self.baseVerts[0][1] * self.baseVerts[0][1] + 
            self.baseVerts[0][2] * self.baseVerts[0][2]
        )

        # this is always constant. can just define it.
        self.normals = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ])

    def mark_as_changed(self):
        self.FLAG_local_matrix_updated = False
        self.FLAG_combined_matrix_updated = False
        self.FLAG_verts_updated = False

    def set_parent_link(self, parent):
        super().set_parent_link(parent)

        # should also invalidate matrix flags
        self.FLAG_local_matrix_updated = False
        self.FLAG_combined_matrix_updated = False
        self.FLAG_verts_updated = False

    def get_normals(self):
        return self.normals

    def get_verts(self):
        if not self.FLAG_verts_updated:
            # get our current position
            p = np.array(self.get_position())

            # and just add it to all our baseverts
            # I love numpy :)
            self.verts = np.copy(self.baseVerts) + p
            self.FLAG_verts_updated = True
        return self.verts


    def get_local_matrix(self):
        if not self.FLAG_local_matrix_updated:
            m = RoboticManipulator.matrix_displacement(
                self.origin[0], 
                self.origin[1], 
                self.origin[2]
                )
            self.localMatrix = m
            self.FLAG_local_matrix_updated = True
        return self.localMatrix

    def get_combined_matrix(self):
        if not self.FLAG_combined_matrix_updated:

            # if no parent, local matrix IS combined matrix
            if self.parentLink is None:
                self.combinedMatrix = self.get_local_matrix()
                self.FLAG_combined_matrix_updated = True
                return self.combinedMatrix

            # normal process
            m = np.copy(self.parentLink.get_combined_matrix())

            # add offsets to parent matrix
            m[0][3] = m[0][3] + self.origin[0]
            m[1][3] = m[1][3] + self.origin[1]
            m[2][3] = m[2][3] + self.origin[2]

            self.combinedMatrix = m
            self.FLAG_combined_matrix_updated = True

        return self.combinedMatrix

    def get_position(self):
        m = self.get_combined_matrix()
        return (m[0][3], m[1][3], m[2][3])

    def get_bounding_sphere_origin(self):
        return self.get_position()

    def get_bounding_sphere_radius(self):
        return self.bounding_sphere_radius


class CuboidCollider(Collider):

    def __init__(self, origin, x, y, z, xAngle, yAngle, zAngle, parentLink=None, name="Unnamed Cuboid Collider"):
        self.origin = origin
        self.x = x
        self.y = y
        self.z = z
        self.xAngle = xAngle
        self.yAngle = yAngle
        self.zAngle = zAngle
        self.parentLink = parentLink
        self.name = name

        self.FLAG_verts_normals_updated = False
        self.verts = None
        self.normals = None

        self.FLAG_local_matrix_updated = False
        self.FLAG_combined_matrix_updated = False

        self.localMatrix = None
        self.combinedMatrix = None

        if self.parentLink is not None:
            self.set_parent_link(parentLink)

        # this will always be needed at some point and never change. just do it now
        self.baseVerts = np.array([
            # NOTE: these first 4 verts matter. they form a corner. this will be used later
            # DO NOT CHANGE ORDER!!!
            # NOTE: the STL file output method is now explicitly coupled to this exact order.
            # it MUST be updated if this order ever changes. 
            [+x/2, +y/2, +z/2],
            [-x/2, +y/2, +z/2],
            [+x/2, -y/2, +z/2],
            [+x/2, +y/2, -z/2],
            # these 4 also form a corner, so they could theoretically be used instead
            [-x/2, -y/2, -z/2],
            [+x/2, -y/2, -z/2],
            [-x/2, +y/2, -z/2],
            [-x/2, -y/2, +z/2],
        ])

        # bounding radius is equal to the distance to any vertex
        self.bounding_sphere_radius = np.sqrt(
            self.baseVerts[0][0] * self.baseVerts[0][0] +
            self.baseVerts[0][1] * self.baseVerts[0][1] + 
            self.baseVerts[0][2] * self.baseVerts[0][2]
        )

    def mark_as_changed(self):
        self.FLAG_local_matrix_updated = False
        self.FLAG_combined_matrix_updated = False
        self.FLAG_verts_normals_updated = False

    def set_parent_link(self, parent):
        super().set_parent_link(parent)

        # should also invalidate matrix flags
        self.FLAG_local_matrix_updated = False
        self.FLAG_combined_matrix_updated = False

    def get_local_matrix(self):
        if not self.FLAG_local_matrix_updated:
            m = RoboticManipulator.matrix_displacement(*self.origin)
            m = np.matmul(m, RoboticManipulator.matrix_rotateX(self.xAngle)) # WOOO! 1000
            m = np.matmul(m, RoboticManipulator.matrix_rotateY(self.yAngle))
            m = np.matmul(m, RoboticManipulator.matrix_rotateZ(self.zAngle))
            self.localMatrix = m
            self.FLAG_local_matrix_updated = True
        return self.localMatrix

    def get_combined_matrix(self):
        if not self.FLAG_combined_matrix_updated:
            # if no parent, local matrix IS combined matrix
            if self.parentLink is None:
                self.combinedMatrix = self.get_local_matrix()
                self.FLAG_combined_matrix_updated = True
                return self.combinedMatrix

            m = np.matmul(self.parentLink.get_combined_matrix(), self.get_local_matrix())

            self.combinedMatrix = m
            self.FLAG_combined_matrix_updated = True
        return self.combinedMatrix
            

    def get_position(self):
        m = self.get_combined_matrix()
        return (m[0][3], m[1][3], m[2][3])

    def get_bounding_sphere_origin(self):
        return self.get_position()

    def get_bounding_sphere_radius(self):
        return self.bounding_sphere_radius

    def get_normals(self):
        if not self.FLAG_verts_normals_updated:
            self.update_verts_normals()
        return self.normals

    def get_verts(self):
        if not self.FLAG_verts_normals_updated:
            self.update_verts_normals()
        return self.verts

    def update_verts_normals(self):
        # the base matrix we work off of depends on whether there is a parent or not
        # AKA is this a stationary or moving collider
        if self.parentLink is None:
            # stationary, use identity matrix
            # this function can be optimised to skip things better when no parent,
            # but it wont matter much as with no parent it is ran exactly once at the begining
            p = np.copy(RoboticManipulator.MATRIX_IDENTITY)
        else:
            # moving, get parent matrix
            p = np.copy(self.parentLink.get_combined_matrix())

        # need parents rotation matrix
        prot = np.copy(p)
        prot[:-1,-1] = 0

        # need our local rotation matrix
        l = self.get_local_matrix()
        lrot = np.copy(l)
        lrot[:-1,-1] = 0

        # so we can combine it with the parent one
        m = np.matmul(prot, lrot)

        # now we have a rotation we can apply to baseverts to rotate them correctly

        # optimisation: because we are only rotating, we can just calc a corner of 4
        # then negate the coords to get the other 4.
        # saves 4 matmuls per cuboid
        v = np.array([
            np.matmul(m[:-1,:-1], self.baseVerts[0]), # REMEMBER! matrix multiplication is non-commutative!
            np.matmul(m[:-1,:-1], self.baseVerts[1]), # these matmuls were the wrong way around at one
            np.matmul(m[:-1,:-1], self.baseVerts[2]), # point. It took me an entire week to notice and
            np.matmul(m[:-1,:-1], self.baseVerts[3]), # nearly drove me insane! :)
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0]
        ])

        for i in range(0,4):
            v[i+4] = np.array([-v[i][0], -v[i][1], -v[i][2]])

        # now that the verts are rotated correctly, we have to rotate the local positional offsets
        # we do this by treating them as a displacement from 0,0,0 and apply them with matrix mult
        o = np.zeros((4,4))
        o[:-1,:-1] = p[:-1,:-1] # has to be JUST parent matrix, NOT m. no local rotations.
        o[3][3] = 1
        o = np.matmul(o, RoboticManipulator.matrix_displacement(*self.origin))[:3,3]

        # then we can add the global frame offests from the parent transform
        # we can add these directly as they are already in the global axes 
        o += p[:-1,3]


        # and add these element-wise to our vertex array
        v += o
        self.verts = v

        # now we do the normals
        # because of the way the vertices are arranged in the array, this just works
        self.normals = np.array([
            v[0] - v[1],
            v[0] - v[2],
            v[0] - v[3],
        ])

        self.FLAG_verts_normals_updated = True


    def set_angles(self,x,y,z):
        self.xAngle = x
        self.yAngle = y
        self.zAngle = z

        self.FLAG_verts_normals_updated = False

    def set_position(self,pos):
        self.origin = pos

        self.FLAG_verts_normals_updated = False


# # below is the complete setup needed for a Niryo Ned 2 Robotic Manipulator using the above
# # need to build the joints first
# JOINT_BASE = RoboticManipulator.Joint(
#     0, 0, 0, # offsets
#     0, 0, 0, # rotation
#     None,
#     angle=0,
#     name="Base"
# )

# JOINT_SHOULDER = RoboticManipulator.Joint(
#     0, 0, 0.1013, # offsets
#     0, 0, 0, # rotation
#     RoboticManipulator.AXIS_Z,
#     angle=0, 
#     parentJoint=JOINT_BASE, 
#     upperLimit=2.949, 
#     lowerLimit=-2.949,
#     name="Shoulder"
# )

# JOINT_ARM = RoboticManipulator.Joint(
#     0, 0, 0.065, # offsets
#     np.pi/2, 0, 0, # rotation
#     RoboticManipulator.AXIS_Z,
#     angle=0, 
#     parentJoint=JOINT_SHOULDER, 
#     upperLimit=0.61, 
#     lowerLimit=-1.83,
#     name="Arm"
# )

# JOINT_ELBOW = RoboticManipulator.Joint(
#     0.012, 0.221, 0, # offsets
#     0, 0, np.pi/2, # rotation
#     RoboticManipulator.AXIS_Z,
#     angle=0, 
#     parentJoint=JOINT_ARM, 
#     upperLimit=1.57, 
#     lowerLimit=-1.34,
#     name="Elbow"
# )

# JOINT_FOREARM = RoboticManipulator.Joint(
#     0.0325, -0.065, 0, # offsets
#     np.pi/2, 0, 0, # rotation
#     RoboticManipulator.AXIS_Z,
#     angle=0, 
#     parentJoint=JOINT_ELBOW, 
#     upperLimit=2.089,
#     lowerLimit=-2.089,
#     name="Forearm"
# )

# JOINT_WRIST = RoboticManipulator.Joint(
#     0, 0, 0.17, # offsets
#     -np.pi/2, 0, 0, # rotation
#     RoboticManipulator.AXIS_Z,
#     angle=0, 
#     parentJoint=JOINT_FOREARM, 
#     upperLimit=1.922,
#     lowerLimit=-1.919,
#     name="Wrist"
# )

# JOINT_HAND = RoboticManipulator.Joint(
#     0.00925, -0.0197, 0, # offsets
#     np.pi/2, 0, 0, # rotation
#     RoboticManipulator.AXIS_Z,
#     angle=0, 
#     parentJoint=JOINT_WRIST, 
#     upperLimit=2.53,
#     lowerLimit=-2.53,
#     name="Hand"
# )

# JOINT_GRIPPER = RoboticManipulator.Joint(
#     0, 0, 0.01016, # offsets
#     0, 0, 0, # rotation
#     None,
#     angle=0, 
#     parentJoint=JOINT_HAND, 
#     upperLimit=2.53,
#     lowerLimit=-2.53,
#     name="Gripper"
# )

# # now we make the links
# LINK_BASE_SHOULDER = RoboticManipulator.Link(parentJoint=JOINT_BASE, name="Base -> Shoulder Link")
# LINK_SHOULDER_ARM = RoboticManipulator.Link(parentJoint=JOINT_SHOULDER, name="Shoulder -> Arm Link")
# LINK_ARM_ELBOW = RoboticManipulator.Link(parentJoint=JOINT_ARM, name="Arm -> Elbow Link")
# LINK_ELBOW_FOREARM = RoboticManipulator.Link(parentJoint=JOINT_ELBOW, name="Elbow -> Forearm Link")
# LINK_FOREARM_WRIST = RoboticManipulator.Link(parentJoint=JOINT_FOREARM, name="Forearm -> Wrist Link")
# LINK_WRIST_HAND = RoboticManipulator.Link(parentJoint=JOINT_WRIST, name="Wrist -> Hand Link")
# LINK_HAND_GIRPPER = RoboticManipulator.Link(parentJoint=JOINT_HAND, name="Hand -> Gripper Link")
# LINK_GRIPPER = RoboticManipulator.Link(parentJoint=JOINT_GRIPPER, name="Gripper Actual Link")


# #==================================================================
# #
# #       ORIGINAL add_collider()S - IE PRE ROTATED
# #
# #==================================================================

# # and now the colliders. this is going to be fun :)
# # using add_collider to group them and to test the function

# LINK_BASE_SHOULDER.add_collider([
#     AlignedCuboidCollider(
#         (0, 0, 0.0448), # origin
#         0.198, 0.198, 0.081, # x,y,z size
#         name="base - main section"
#     ),
#     AlignedCuboidCollider(
#         (0, 0, 0.093885), # origin
#         0.159987, 0.159987, 0.015414, # x,y,z size
#         name="base - LED ring"
#     ),
# ])

# LINK_SHOULDER_ARM.add_collider([
#     CuboidCollider(
#         (0.999951-1, -0.0001, 0.057294), # origin
#         0.096747, 0.1148, 0.114589, # x,y,z size
#         0, 0, 0, # x,y,z rotation
#         name="shoulder - main joint"
#     ),
#     CuboidCollider(
#         (0.921189-1, 0.012574, 0.026108), # origin
#         0.060777, 0.120253, 0.053785, # x,y,z size
#         0, 0, 0, # x,y,z rotation
#         name="shoulder - motor box"
#     )
# ])

# LINK_ARM_ELBOW.add_collider([
#     # rotation:
#     # x -> x
#     # y -> -z
#     # z -> y
#     CuboidCollider(
#         (2-2, 0.077342, -0), # origin
#         0.05264, 0.105281, 0.05264,# x,y,z size
#         0, 0, 0, # x,y,z rotation
#         name="arm - connecting box"
#     ),
#     CuboidCollider(
#         (2-2, 0.14469, -0), # origin
#         0.064, 0.06402, 0.101, # x,y,z size
#         0, 0, 0, # x,y,z rotation
#         name="arm - mid box"
#     ),
#     CuboidCollider(
#         (2.00015-2, 0.154575, -0.06035), # origin
#         0.0423, 0.05285, 0.0197, # x,y,z size
#         0, 0, 0, # x,y,z rotation
#         name="arm - side box"
#     ),
#     CuboidCollider(
#         (2.0125-2, 0.2207, -0), # origin
#         0.088, 0.088, 0.101, # x,y,z size
#         0, 0, 0, # x,y,z rotation
#         name="arm - top box"
#     ),
# ])

# LINK_ELBOW_FOREARM.add_collider([
#     # rotation:
#     # x -> -y
#     # y -> -z
#     # z -> x
#     CuboidCollider(
#         (0.037725, -(3.00788-3), -0), # origin
#         0.06005, 0.07777, 0.062, # x,y,z size
#         0, 0, 0, # x,y,z rotation
#         name="elbow"
#     ),
# ])

# LINK_FOREARM_WRIST.add_collider([
#     # rotation:
#     # x -> z
#     # y -> -y
#     # z -> x
#     CuboidCollider(
#         (-0.000218, 0, 4.00341-4), # origin
#         0.057091, 0.06, 0.043388, # x,y,z size
#         0, 0, 0, # x,y,z rotation
#         name="forearm - socket"
#     ),
#     CuboidCollider(
#         (0.0002, 0, 4.048-4), # origin
#         0.04, 0.04, 0.047896, # x,y,z size
#         0, 0, 0, # x,y,z rotation
#         name="forearm - bridge"
#     ),
#     CuboidCollider(
#         (0, 0, 4.13231-4), # origin
#         0.05, 0.06, 0.125736, # x,y,z size
#         0, 0, 0, # x,y,z rotation
#         name="forearm - main"
#     ),
#     CuboidCollider(
#         (0, 0.045, 4.158-4), # origin
#         0.0285, 0.03, 0.0465, # x,y,z size
#         0, 0, 0, # x,y,z rotation
#         name="forearm - side"
#     ),
# ])

# LINK_WRIST_HAND.add_collider([
#     # rotation:
#     # x -> -y
#     # y -> -z
#     # z -> x
#     CuboidCollider(
#         (0, -(4.981-5), 0), # origin
#         0.036, 0.06, 0.03, # x,y,z size
#         0, 0, 0, # x,y,z rotation
#         name="wrist"
#     ),
# ])

# LINK_HAND_GIRPPER.add_collider([
#     # rotation:
#     # x -> z
#     # y -> -y
#     # z -> x
#     CuboidCollider(
#         (-0.00005, 0, 6.0113-6), # origin
#         0.02, 0.02, 0.035, # x,y,z size
#         0, 0, 0, # x,y,z rotation
#         name="hand"
#     ),
# ])

# LINK_GRIPPER.add_collider([
#     # rotation:
#     # x -> z
#     # y -> -y
#     # z -> x
#     CuboidCollider(
#         (-0.00025, 0, 7.0616-7), # origin
#         0.0235, 0.076999, 0.085355, # x,y,z size
#         0, 0, 0, # x,y,z rotation
#         name="gripper - main"
#     ), 
#     CuboidCollider(
#         (0.014912, -0.000219, 7.04175-7), # origin
#         0.006823, 0.082321, 0.024491, # x,y,z size
#         0, 0, 0, # x,y,z rotation
#         name="gripper - upper"
#     ), 
#     CuboidCollider(
#         (-0.0165, 0.0075, 7.04-7), # origin
#         0.009, 0.034, 0.02, # x,y,z size
#         0, 0, 0, # x,y,z rotation
#         name="gripper - lower"
#     ),
# ])


# # and finally construct the arm!
# RM = RoboticManipulator([
#     JOINT_BASE,
#     JOINT_SHOULDER,
#     JOINT_ARM,
#     JOINT_ELBOW,
#     JOINT_FOREARM,
#     JOINT_WRIST,
#     JOINT_HAND,
#     JOINT_GRIPPER,
# ])
