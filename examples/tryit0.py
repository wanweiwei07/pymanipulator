import numpy as np
import pandaplotutils.pandactrl as pc
import pandaplotutils.pandageom as pg
import utils.robotmath as rm

base = pc.World(camp = [1000, -300, 700], lookatp = [0,0,0], w = 1000, h = 1000)
pggen = pg.PandaGeomGen()
pggen.plotAxis(base.render)
pggen.plotBox(base.render, pos = [0,0,0], x=100, y=100, z=50, rgba=[1,1,.3,1])
#
rotmat = rm.rodrigues([1,1,1], 30)
pandarotmat = pg.cvtMat4(rotmat, np.array([0,0,0]))
pggen.plotAxis(base.render, pandamat4=pandarotmat)

base.run()
