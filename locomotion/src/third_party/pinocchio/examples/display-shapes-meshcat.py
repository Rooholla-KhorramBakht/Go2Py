import sys
import numpy as np
import pinocchio as pin
try:
    import hppfcl
except ImportError:
    print("This example requires hppfcl")
    sys.exit(0)
from pinocchio.visualize import MeshcatVisualizer

model = pin.Model()

geom_model = pin.GeometryModel()
geometries = [
    hppfcl.Capsule(0.1, 0.8),
    hppfcl.Sphere(0.5),
    hppfcl.Box(1, 1, 1),
    hppfcl.Cylinder(0.1, 1.0),
    hppfcl.Cone(0.5, 1.0),
]
for i, geom in enumerate(geometries):
    placement = pin.SE3(np.eye(3), np.array([i, 0, 0]))
    geom_obj = pin.GeometryObject("obj{}".format(i), 0, 0, geom, placement)
    color = np.random.uniform(0, 1, 4)
    color[3] = 1
    geom_obj.meshColor = color
    geom_model.addGeometryObject(geom_obj)

viz = MeshcatVisualizer(model, geom_model, geom_model)

# Initialize the viewer.
try:
    viz.initViewer(open=True)
except ImportError as error:
    print(error)
    sys.exit(0)

try:
    viz.loadViewerModel("shapes")
except AttributeError as error:
    print(error)
    sys.exit(0)

viz.display(np.zeros(0))
input("press enter to continue")
