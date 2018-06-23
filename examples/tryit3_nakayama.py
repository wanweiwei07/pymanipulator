import os
import database.dbaccess as db
from manipulation.grip.robotiq85 import rtq85nm
from manipulation.grip import freegrip
from pandaplotutils import pandactrl

base = pandactrl.World(camp=[700, 300, 700], lookatp=[0, 0, 100])
this_dir, this_filename = os.path.split(__file__)
print this_dir
objpath = os.path.join(this_dir, "../manipulation/grip/objects", "plane.stl")
# objnp = base.pg.loadstlaspandanp_fn(objpath)
# objnp.reparentTo(base.render)
# base.run()
handpkg = rtq85nm

freegriptst = freegrip.Freegrip(objpath, handpkg, readser=False, torqueresist = 70, dotnormplan=.90, dotnoarmovlp=.95, dotnormpara = -.80)
freegriptst.segShow(base, togglesamples=False, togglenormals=False,
                    togglesamples_ref=False, togglenormals_ref=False,
                    togglesamples_refcls=False, togglenormals_refcls=False)

freegriptst.removeFgrpcc(base)
freegriptst.removeHndcc(base, discretesize=16)

gdb = db.GraspDB()
freegriptst.saveToDB(gdb)
#
data = gdb.loadFreeAirGrip('plane', 'rtq85')
if data:
    freegripid, freegripcontacts, freegripnormals, freegriprotmats, freegripjawwidth = data
    print len(freegripid)
    for i, freegriprotmat in enumerate(freegriprotmats):
        # if i>120 and i-120 < 30:
        rtqhnd = rtq85nm.Rtq85NM(hndcolor=[1, 1, 1, .2])
        rtqhnd.setMat(pandanpmat4=freegriprotmat)
        rtqhnd.setJawwidth(freegripjawwidth[i])
        rtqhnd.reparentTo(base.render)


# def updateshow(task):
#     freegriptst.removeFgrpccShow(base)
#     # freegriptst.removeFgrpccShowLeft(base)
#     # freegriptst.removeHndccShow(base)
#     return task.again
# taskMgr.doMethodLater(.1, updateshow, "tickTask")

base.run()