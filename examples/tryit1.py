import numpy as np
import pandaplotutils.pandactrl as pc
import pandaplotutils.pandageom as pg
import utils.robotmath as rm
import trimesh
import os
from panda3d.core import Filename, TransparencyAttrib

base = pc.World(camp = [1000, -300, 700], lookatp = [0,10,0], w = 1000, h = 1000)
pggen = pg.PandaGeomGen()
pggen.plotAxis(base.render)

this_dir, this_filename = os.path.split(__file__)
objpath = os.path.join(this_dir, "sandpart.stl")
# objtrimesh=trimesh.load_mesh(objpath)
# #
# objmnp = pg.trimesh2Panda(objtrimesh)
objmnp = pg.genObjmnp(objpath, color = (0,0,0,1))
# #
objmnp.reparentTo(base.render)
#
# eggpath = Filename.fromOsSpecific(os.path.join(this_dir, "bzcube.egg"))
# objmnp2 = loader.loadModel(eggpath)
# objmnp2.setScale(5.0)
# objmnp2.setColor(0,0,1, 0.7)
# objmnp2.setTransparency(TransparencyAttrib.MAlpha)
# objmnp2.reparentTo(base.render)

base.run()
