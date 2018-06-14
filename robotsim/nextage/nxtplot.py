# the file plots in Panda3D

import os

import numpy as np
from manipulation.grip.robotiq85 import rtq85
from panda3d.core import *

import pandaplotutils.pandageom as pandageom
from manipulation.grip.robotiq85 import rtq85nm


def plotstick(pandanp, nxtrobot, rgtrbga=np.array([.5 ,0 ,0 ,0]), lftrgba=np.array([.5 ,0 ,0 ,0])):
    """
    plot the stick model of the nextage robot in panda3d

    :param pandanp: a panda3d nodepath
    :param nxtrobot: the NxtRobot object, see nextage.py
    :param rgtrbga: color of right arm
    :param lftrgba: color of left arm
    :return: null

    author: weiwei
    date: 20161108
    """

    i = 0
    while i != -1:
        pandageom.plotDumbbell(pandanp, spos=nxtrobot.rgtarm[i]['linkpos'], epos=nxtrobot.rgtarm[i]['linkend'],
                               thickness=20, rgba=rgtrbga)
        i = nxtrobot.rgtarm[i]['child']
    i = 0
    while i != -1:
        pandageom.plotDumbbell(pandanp, spos=nxtrobot.lftarm[i]['linkpos'], epos=nxtrobot.lftarm[i]['linkend'],
                               thickness=20, rgba=lftrgba)
        i = nxtrobot.lftarm[i]['child']

def genmnp(nxtrobot, handpkg, jawwidthrgt = None, jawwidthlft = None):
    """
    generate a panda3d nodepath for the nxtrobot
    mnp indicates this function generates a mesh nodepath

    :param nxtrobot: the NxtRobot object, see nextage.py
    :return: a nodepath which is ready to be plotted using plotmesh

    author: weiwei
    date: 20161109
    """

    nxtmnp = NodePath("nxtmnp")
    
    this_dir, this_filename = os.path.split(__file__)

    # mainbody, waist, body, head (neck is not plotted)
    nxtwaist_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_waist.egg"))
    nxtbody_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_body.egg"))
    nxthead_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_head.egg"))

    nxtwaist_model = loader.loadModel(nxtwaist_filepath)
    nxtbody_model = loader.loadModel(nxtbody_filepath)
    nxthead_model = loader.loadModel(nxthead_filepath)

    nxtwaist_nodepath = NodePath("nxtwaist")
    nxtbody_nodepath = NodePath("nxtbody")
    nxthead_nodepath = NodePath("nxthead")

    nxtwaist_model.instanceTo(nxtwaist_nodepath)
    nxtwaist_rotmat = pandageom.cvtMat4(nxtrobot.base['rotmat'])
    nxtwaist_nodepath.setMat(nxtwaist_rotmat)
    nxtbody_model.instanceTo(nxtbody_nodepath)
    nxthead_model.instanceTo(nxthead_nodepath)
    nxthead_nodepath.setH(nxtrobot.initjnts[1])
    nxthead_nodepath.setP(nxtrobot.initjnts[2])
    nxthead_nodepath.setZ(569.5)

    nxtwaist_nodepath.reparentTo(nxtmnp)
    nxtbody_nodepath.reparentTo(nxtwaist_nodepath)
    nxthead_nodepath.reparentTo(nxtwaist_nodepath)

    # rgtarm
    nxtrgtarmlj0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_rgtarm_lj0.egg"))
    nxtrgtarmlj0_model = loader.loadModel(nxtrgtarmlj0_filepath)
    nxtrgtarmlj0_nodepath = NodePath("nxtrgtarmlj0_nodepath")
    nxtrgtarmlj0_model.instanceTo(nxtrgtarmlj0_nodepath)
    nxtrgtarmlj0_rotmat = pandageom.cvtMat4(nxtrobot.rgtarm[1]['rotmat'], nxtrobot.rgtarm[1]['linkpos'])
    nxtrgtarmlj0_nodepath.setMat(nxtrgtarmlj0_rotmat)
    nxtrgtarmlj0_nodepath.reparentTo(nxtmnp)

    nxtrgtarmlj1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_rgtarm_lj1.egg"))
    nxtrgtarmlj1_model = loader.loadModel(nxtrgtarmlj1_filepath)
    nxtrgtarmlj1_nodepath = NodePath("nxtrgtarmlj1_nodepath")
    nxtrgtarmlj1_model.instanceTo(nxtrgtarmlj1_nodepath)
    nxtrgtarmlj1_rotmat = pandageom.cvtMat4(nxtrobot.rgtarm[2]['rotmat'], nxtrobot.rgtarm[2]['linkpos'])
    nxtrgtarmlj1_nodepath.setMat(nxtrgtarmlj1_rotmat)
    nxtrgtarmlj1_nodepath.reparentTo(nxtmnp)

    nxtrgtarmlj2_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_rgtarm_lj2.egg"))
    nxtrgtarmlj2_model = loader.loadModel(nxtrgtarmlj2_filepath)
    nxtrgtarmlj2_nodepath = NodePath("nxtrgtarmlj2_nodepath")
    nxtrgtarmlj2_model.instanceTo(nxtrgtarmlj2_nodepath)
    nxtrgtarmlj2_rotmat = pandageom.cvtMat4(nxtrobot.rgtarm[3]['rotmat'], nxtrobot.rgtarm[3]['linkpos'])
    nxtrgtarmlj2_nodepath.setMat(nxtrgtarmlj2_rotmat)
    nxtrgtarmlj2_nodepath.reparentTo(nxtmnp)

    nxtrgtarmlj3_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_rgtarm_lj3.egg"))
    nxtrgtarmlj3_model = loader.loadModel(nxtrgtarmlj3_filepath)
    nxtrgtarmlj3_nodepath = NodePath("nxtrgtarmlj3_nodepath")
    nxtrgtarmlj3_model.instanceTo(nxtrgtarmlj3_nodepath)
    nxtrgtarmlj3_rotmat = pandageom.cvtMat4(nxtrobot.rgtarm[4]['rotmat'], nxtrobot.rgtarm[4]['linkpos'])
    nxtrgtarmlj3_nodepath.setMat(nxtrgtarmlj3_rotmat)
    nxtrgtarmlj3_nodepath.reparentTo(nxtmnp)

    nxtrgtarmlj4_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_rgtarm_lj4.egg"))
    nxtrgtarmlj4_model = loader.loadModel(nxtrgtarmlj4_filepath)
    nxtrgtarmlj4_nodepath = NodePath("nxtrgtarmlj4_nodepath")
    nxtrgtarmlj4_model.instanceTo(nxtrgtarmlj4_nodepath)
    nxtrgtarmlj4_rotmat = pandageom.cvtMat4(nxtrobot.rgtarm[5]['rotmat'], nxtrobot.rgtarm[5]['linkpos'])
    nxtrgtarmlj4_nodepath.setMat(nxtrgtarmlj4_rotmat)
    nxtrgtarmlj4_nodepath.reparentTo(nxtmnp)

    # lftarm
    nxtlftarmlj0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_lftarm_lj0.egg"))
    nxtlftarmlj0_model = loader.loadModel(nxtlftarmlj0_filepath)
    nxtlftarmlj0_nodepath = NodePath("nxtlftarmlj0_nodepath")
    nxtlftarmlj0_model.instanceTo(nxtlftarmlj0_nodepath)
    nxtlftarmlj0_rotmat = pandageom.cvtMat4(nxtrobot.lftarm[1]['rotmat'], nxtrobot.lftarm[1]['linkpos'])
    nxtlftarmlj0_nodepath.setMat(nxtlftarmlj0_rotmat)
    nxtlftarmlj0_nodepath.reparentTo(nxtmnp)

    nxtlftarmlj1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_lftarm_lj1.egg"))
    nxtlftarmlj1_model = loader.loadModel(nxtlftarmlj1_filepath)
    nxtlftarmlj1_nodepath = NodePath("nxtlftarmlj1_nodepath")
    nxtlftarmlj1_model.instanceTo(nxtlftarmlj1_nodepath)
    nxtlftarmlj1_rotmat = pandageom.cvtMat4(nxtrobot.lftarm[2]['rotmat'], nxtrobot.lftarm[2]['linkpos'])
    nxtlftarmlj1_nodepath.setMat(nxtlftarmlj1_rotmat)
    nxtlftarmlj1_nodepath.reparentTo(nxtmnp)

    nxtlftarmlj2_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_lftarm_lj2.egg"))
    nxtlftarmlj2_model = loader.loadModel(nxtlftarmlj2_filepath)
    nxtlftarmlj2_nodepath = NodePath("nxtlftarmlj2_nodepath")
    nxtlftarmlj2_model.instanceTo(nxtlftarmlj2_nodepath)
    nxtlftarmlj2_rotmat = pandageom.cvtMat4(nxtrobot.lftarm[3]['rotmat'], nxtrobot.lftarm[3]['linkpos'])
    nxtlftarmlj2_nodepath.setMat(nxtlftarmlj2_rotmat)
    nxtlftarmlj2_nodepath.reparentTo(nxtmnp)

    nxtlftarmlj3_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_lftarm_lj3.egg"))
    nxtlftarmlj3_model = loader.loadModel(nxtlftarmlj3_filepath)
    nxtlftarmlj3_nodepath = NodePath("nxtlftarmlj3_nodepath")
    nxtlftarmlj3_model.instanceTo(nxtlftarmlj3_nodepath)
    nxtlftarmlj3_rotmat = pandageom.cvtMat4(nxtrobot.lftarm[4]['rotmat'], nxtrobot.lftarm[4]['linkpos'])
    nxtlftarmlj3_nodepath.setMat(nxtlftarmlj3_rotmat)
    nxtlftarmlj3_nodepath.reparentTo(nxtmnp)

    nxtlftarmlj4_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_lftarm_lj4.egg"))
    nxtlftarmlj4_model = loader.loadModel(nxtlftarmlj4_filepath)
    nxtlftarmlj4_nodepath = NodePath("nxtlftarmlj4_nodepath")
    nxtlftarmlj4_model.instanceTo(nxtlftarmlj4_nodepath)
    nxtlftarmlj4_rotmat = pandageom.cvtMat4(nxtrobot.lftarm[5]['rotmat'], nxtrobot.lftarm[5]['linkpos'])
    nxtlftarmlj4_nodepath.setMat(nxtlftarmlj4_rotmat)
    nxtlftarmlj4_nodepath.reparentTo(nxtmnp)

    # rgthnd
    nxtrgthnd = handpkg.newHand('rgt')
    nxtlftarmlj5_rotmat = pandageom.cvtMat4(nxtrobot.rgtarm[6]['rotmat'], nxtrobot.rgtarm[6]['linkpos'])
    nxtrgthnd.setMat(nxtlftarmlj5_rotmat)
    nxtrgthnd.reparentTo(nxtmnp)
    if jawwidthrgt is not None:
        nxtrgthnd.setJawwidth(jawwidthrgt)

    # lfthnd
    nxtlfthnd = handpkg.newHand('lft')
    nxtlftarmlj5_rotmat = pandageom.cvtMat4(nxtrobot.lftarm[6]['rotmat'], nxtrobot.lftarm[6]['linkpos'])
    nxtlfthnd.setMat(nxtlftarmlj5_rotmat)
    nxtlfthnd.reparentTo(nxtmnp)
    if jawwidthlft is not None:
        nxtlfthnd.setJawwidth(jawwidthlft)

    return nxtmnp

def genmnplist(nxtrobot, handpkg, jawwidthrgt=None, jawwidthlft=None):
    """
    generate a panda3d nodepath for the nxtrobot
    mnp indicates this function generates a mesh nodepath

    [[rightarm mnp list], [leftarm mnp list], [body list]]
    # Right goes first!
    # The order of an arm mnp list is from base to end-effector
    # body list is from base to head

    :param nxtrobot: the NxtRobot object, see nextage.py
    :return:

    author: weiwei
    date: 20161109
    """

    nxtmnp = NodePath("nxtmnp")

    this_dir, this_filename = os.path.split(__file__)

    # mainbody, waist, body, head (neck is not plotted)
    nxtwaist_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_waist.egg"))
    nxtbody_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_body.egg"))
    nxthead_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_head.egg"))

    nxtwaist_model = loader.loadModel(nxtwaist_filepath)
    nxtbody_model = loader.loadModel(nxtbody_filepath)
    nxthead_model = loader.loadModel(nxthead_filepath)

    nxtwaist_nodepath = NodePath("nxtwaist")
    nxtbody_nodepath = NodePath("nxtbody")
    nxthead_nodepath = NodePath("nxthead")

    nxtwaist_model.instanceTo(nxtwaist_nodepath)
    nxtwaist_rotmat = pandageom.cvtMat4(nxtrobot.base['rotmat'])
    nxtwaist_nodepath.setMat(nxtwaist_rotmat)
    nxtbody_model.instanceTo(nxtbody_nodepath)
    nxthead_model.instanceTo(nxthead_nodepath)
    nxthead_nodepath.setH(nxtrobot.initjnts[1])
    nxthead_nodepath.setP(nxtrobot.initjnts[2])
    nxthead_nodepath.setZ(569.5)

    nxtwaist_nodepath.reparentTo(nxtmnp)
    nxtbody_nodepath.reparentTo(nxtwaist_nodepath)
    nxthead_nodepath.reparentTo(nxtwaist_nodepath)

    # rgtarm
    nxtrgtarmlj0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_rgtarm_lj0.egg"))
    nxtrgtarmlj0_model = loader.loadModel(nxtrgtarmlj0_filepath)
    nxtrgtarmlj0_nodepath = NodePath("nxtrgtarmlj0_nodepath")
    nxtrgtarmlj0_model.instanceTo(nxtrgtarmlj0_nodepath)
    nxtrgtarmlj0_rotmat = pandageom.cvtMat4(nxtrobot.rgtarm[1]['rotmat'], nxtrobot.rgtarm[1]['linkpos'])
    nxtrgtarmlj0_nodepath.setMat(nxtrgtarmlj0_rotmat)
    nxtrgtarmlj0_nodepath.reparentTo(nxtmnp)

    nxtrgtarmlj1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_rgtarm_lj1.egg"))
    nxtrgtarmlj1_model = loader.loadModel(nxtrgtarmlj1_filepath)
    nxtrgtarmlj1_nodepath = NodePath("nxtrgtarmlj1_nodepath")
    nxtrgtarmlj1_model.instanceTo(nxtrgtarmlj1_nodepath)
    nxtrgtarmlj1_rotmat = pandageom.cvtMat4(nxtrobot.rgtarm[2]['rotmat'], nxtrobot.rgtarm[2]['linkpos'])
    nxtrgtarmlj1_nodepath.setMat(nxtrgtarmlj1_rotmat)
    nxtrgtarmlj1_nodepath.reparentTo(nxtmnp)

    nxtrgtarmlj2_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_rgtarm_lj2.egg"))
    nxtrgtarmlj2_model = loader.loadModel(nxtrgtarmlj2_filepath)
    nxtrgtarmlj2_nodepath = NodePath("nxtrgtarmlj2_nodepath")
    nxtrgtarmlj2_model.instanceTo(nxtrgtarmlj2_nodepath)
    nxtrgtarmlj2_rotmat = pandageom.cvtMat4(nxtrobot.rgtarm[3]['rotmat'], nxtrobot.rgtarm[3]['linkpos'])
    nxtrgtarmlj2_nodepath.setMat(nxtrgtarmlj2_rotmat)
    nxtrgtarmlj2_nodepath.reparentTo(nxtmnp)

    nxtrgtarmlj3_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_rgtarm_lj3.egg"))
    nxtrgtarmlj3_model = loader.loadModel(nxtrgtarmlj3_filepath)
    nxtrgtarmlj3_nodepath = NodePath("nxtrgtarmlj3_nodepath")
    nxtrgtarmlj3_model.instanceTo(nxtrgtarmlj3_nodepath)
    nxtrgtarmlj3_rotmat = pandageom.cvtMat4(nxtrobot.rgtarm[4]['rotmat'], nxtrobot.rgtarm[4]['linkpos'])
    nxtrgtarmlj3_nodepath.setMat(nxtrgtarmlj3_rotmat)
    nxtrgtarmlj3_nodepath.reparentTo(nxtmnp)

    nxtrgtarmlj4_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_rgtarm_lj4.egg"))
    nxtrgtarmlj4_model = loader.loadModel(nxtrgtarmlj4_filepath)
    nxtrgtarmlj4_nodepath = NodePath("nxtrgtarmlj4_nodepath")
    nxtrgtarmlj4_model.instanceTo(nxtrgtarmlj4_nodepath)
    nxtrgtarmlj4_rotmat = pandageom.cvtMat4(nxtrobot.rgtarm[5]['rotmat'], nxtrobot.rgtarm[5]['linkpos'])
    nxtrgtarmlj4_nodepath.setMat(nxtrgtarmlj4_rotmat)
    nxtrgtarmlj4_nodepath.reparentTo(nxtmnp)

    # lftarm
    nxtlftarmlj0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_lftarm_lj0.egg"))
    nxtlftarmlj0_model = loader.loadModel(nxtlftarmlj0_filepath)
    nxtlftarmlj0_nodepath = NodePath("nxtlftarmlj0_nodepath")
    nxtlftarmlj0_model.instanceTo(nxtlftarmlj0_nodepath)
    nxtlftarmlj0_rotmat = pandageom.cvtMat4(nxtrobot.lftarm[1]['rotmat'], nxtrobot.lftarm[1]['linkpos'])
    nxtlftarmlj0_nodepath.setMat(nxtlftarmlj0_rotmat)
    nxtlftarmlj0_nodepath.reparentTo(nxtmnp)

    nxtlftarmlj1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_lftarm_lj1.egg"))
    nxtlftarmlj1_model = loader.loadModel(nxtlftarmlj1_filepath)
    nxtlftarmlj1_nodepath = NodePath("nxtlftarmlj1_nodepath")
    nxtlftarmlj1_model.instanceTo(nxtlftarmlj1_nodepath)
    nxtlftarmlj1_rotmat = pandageom.cvtMat4(nxtrobot.lftarm[2]['rotmat'], nxtrobot.lftarm[2]['linkpos'])
    nxtlftarmlj1_nodepath.setMat(nxtlftarmlj1_rotmat)
    nxtlftarmlj1_nodepath.reparentTo(nxtmnp)

    nxtlftarmlj2_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_lftarm_lj2.egg"))
    nxtlftarmlj2_model = loader.loadModel(nxtlftarmlj2_filepath)
    nxtlftarmlj2_nodepath = NodePath("nxtlftarmlj2_nodepath")
    nxtlftarmlj2_model.instanceTo(nxtlftarmlj2_nodepath)
    nxtlftarmlj2_rotmat = pandageom.cvtMat4(nxtrobot.lftarm[3]['rotmat'], nxtrobot.lftarm[3]['linkpos'])
    nxtlftarmlj2_nodepath.setMat(nxtlftarmlj2_rotmat)
    nxtlftarmlj2_nodepath.reparentTo(nxtmnp)

    nxtlftarmlj3_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_lftarm_lj3.egg"))
    nxtlftarmlj3_model = loader.loadModel(nxtlftarmlj3_filepath)
    nxtlftarmlj3_nodepath = NodePath("nxtlftarmlj3_nodepath")
    nxtlftarmlj3_model.instanceTo(nxtlftarmlj3_nodepath)
    nxtlftarmlj3_rotmat = pandageom.cvtMat4(nxtrobot.lftarm[4]['rotmat'], nxtrobot.lftarm[4]['linkpos'])
    nxtlftarmlj3_nodepath.setMat(nxtlftarmlj3_rotmat)
    nxtlftarmlj3_nodepath.reparentTo(nxtmnp)

    nxtlftarmlj4_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_lftarm_lj4.egg"))
    nxtlftarmlj4_model = loader.loadModel(nxtlftarmlj4_filepath)
    nxtlftarmlj4_nodepath = NodePath("nxtlftarmlj4_nodepath")
    nxtlftarmlj4_model.instanceTo(nxtlftarmlj4_nodepath)
    nxtlftarmlj4_rotmat = pandageom.cvtMat4(nxtrobot.lftarm[5]['rotmat'], nxtrobot.lftarm[5]['linkpos'])
    nxtlftarmlj4_nodepath.setMat(nxtlftarmlj4_rotmat)
    nxtlftarmlj4_nodepath.reparentTo(nxtmnp)

    # rgthnd
    nxtrgthnd = handpkg.newHand('rgt')
    nxtlftarmlj5_rotmat = pandageom.cvtMat4(nxtrobot.rgtarm[6]['rotmat'], nxtrobot.rgtarm[6]['linkpos'])
    nxtrgthnd.setMat(nxtlftarmlj5_rotmat)
    nxtrgthnd.reparentTo(nxtmnp)
    if jawwidthrgt is not None:
        nxtrgthnd.setJawwidth(jawwidthrgt)

    # lfthnd
    nxtlfthnd = handpkg.newHand('lft')
    nxtlftarmlj5_rotmat = pandageom.cvtMat4(nxtrobot.lftarm[6]['rotmat'], nxtrobot.lftarm[6]['linkpos'])
    nxtlfthnd.setMat(nxtlftarmlj5_rotmat)
    nxtlfthnd.reparentTo(nxtmnp)
    if jawwidthlft is not None:
        nxtlfthnd.setJawwidth(jawwidthlft)

    return [[nxtrgtarmlj0_nodepath, nxtrgtarmlj1_nodepath, nxtrgtarmlj2_nodepath,
             nxtrgtarmlj3_nodepath, nxtrgtarmlj4_nodepath, nxtrgthnd.handnp],
            [nxtlftarmlj0_nodepath, nxtlftarmlj1_nodepath, nxtlftarmlj2_nodepath,
             nxtlftarmlj3_nodepath, nxtlftarmlj4_nodepath, nxtlfthnd.handnp],
            [nxtwaist_nodepath]]

def genmnp_nm(nxtrobot, handpkg, plotcolor=[.5,.5,.5,.3], jawwidthrgt = None, jawwidthlft = None):
    """
    generate a panda3d nodepath for the nxtrobot
    mnp indicates this function generates a mesh nodepath
    nm means no material

    :param nxtrobot: the NxtRobot object, see nextage.py
    :param plotcolor: the color of the model, alpha allowed
    :return: a nodepath which is ready to be plotted using plotmesh

    author: weiwei
    date: 20161109
    """

    nxtmnp = NodePath("nxtmnp")

    this_dir, this_filename = os.path.split(__file__)

    # mainbody, waist, body, head (neck is not plotted)
    nxtwaist_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg/nomat", "nxt_waist_nm.egg"))
    nxtbody_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg/nomat", "nxt_body_nm.egg"))
    nxthead_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg/nomat", "nxt_head_nm.egg"))

    nxtwaist_model = loader.loadModel(nxtwaist_filepath)
    nxtbody_model = loader.loadModel(nxtbody_filepath)
    nxthead_model = loader.loadModel(nxthead_filepath)

    nxtwaist_nodepath = NodePath("nxtwaist_nm")
    nxtbody_nodepath = NodePath("nxtbody_nm")
    nxthead_nodepath = NodePath("nxthead_nm")

    nxtwaist_nodepath.setColor(plotcolor[0],plotcolor[1],plotcolor[2],plotcolor[3])
    nxtwaist_nodepath.setTransparency(TransparencyAttrib.MAlpha)
    nxtbody_nodepath.setColor(plotcolor[0],plotcolor[1],plotcolor[2],plotcolor[3])
    nxtbody_nodepath.setTransparency(TransparencyAttrib.MAlpha)
    nxthead_nodepath.setColor(plotcolor[0],plotcolor[1],plotcolor[2],plotcolor[3])
    nxthead_nodepath.setTransparency(TransparencyAttrib.MAlpha)

    nxtwaist_model.instanceTo(nxtwaist_nodepath)
    nxtwaist_rotmat = pandageom.cvtMat4(nxtrobot.base['rotmat'])
    nxtwaist_nodepath.setMat(nxtwaist_rotmat)
    nxtbody_model.instanceTo(nxtbody_nodepath)
    nxthead_model.instanceTo(nxthead_nodepath)
    nxthead_nodepath.setH(nxtrobot.initjnts[1])
    nxthead_nodepath.setP(nxtrobot.initjnts[2])
    nxthead_nodepath.setZ(569.5)

    nxtwaist_nodepath.reparentTo(nxtmnp)
    nxtbody_nodepath.reparentTo(nxtwaist_nodepath)
    nxthead_nodepath.reparentTo(nxtwaist_nodepath)

    # rgtarm
    nxtrgtarmlj0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg/nomat", "nxt_rgtarm_lj0_nm.egg"))
    nxtrgtarmlj0_model = loader.loadModel(nxtrgtarmlj0_filepath)
    nxtrgtarmlj0_nodepath = NodePath("nxtrgtarmlj0_nodepath")
    nxtrgtarmlj0_model.instanceTo(nxtrgtarmlj0_nodepath)
    nxtrgtarmlj0_rotmat = pandageom.cvtMat4(nxtrobot.rgtarm[1]['rotmat'], nxtrobot.rgtarm[1]['linkpos'])
    nxtrgtarmlj0_nodepath.setMat(nxtrgtarmlj0_rotmat)
    nxtrgtarmlj0_nodepath.reparentTo(nxtmnp)
    nxtrgtarmlj0_nodepath.setColor(plotcolor[0],plotcolor[1],plotcolor[2],plotcolor[3])
    nxtrgtarmlj0_nodepath.setTransparency(TransparencyAttrib.MAlpha)

    nxtrgtarmlj1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg/nomat", "nxt_rgtarm_lj1_nm.egg"))
    nxtrgtarmlj1_model = loader.loadModel(nxtrgtarmlj1_filepath)
    nxtrgtarmlj1_nodepath = NodePath("nxtrgtarmlj1_nodepath")
    nxtrgtarmlj1_model.instanceTo(nxtrgtarmlj1_nodepath)
    nxtrgtarmlj1_rotmat = pandageom.cvtMat4(nxtrobot.rgtarm[2]['rotmat'], nxtrobot.rgtarm[2]['linkpos'])
    nxtrgtarmlj1_nodepath.setMat(nxtrgtarmlj1_rotmat)
    nxtrgtarmlj1_nodepath.reparentTo(nxtmnp)
    nxtrgtarmlj1_nodepath.setColor(plotcolor[0],plotcolor[1],plotcolor[2],plotcolor[3])
    nxtrgtarmlj1_nodepath.setTransparency(TransparencyAttrib.MAlpha)

    nxtrgtarmlj2_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg/nomat", "nxt_rgtarm_lj2_nm.egg"))
    nxtrgtarmlj2_model = loader.loadModel(nxtrgtarmlj2_filepath)
    nxtrgtarmlj2_nodepath = NodePath("nxtrgtarmlj2_nodepath")
    nxtrgtarmlj2_model.instanceTo(nxtrgtarmlj2_nodepath)
    nxtrgtarmlj2_rotmat = pandageom.cvtMat4(nxtrobot.rgtarm[3]['rotmat'], nxtrobot.rgtarm[3]['linkpos'])
    nxtrgtarmlj2_nodepath.setMat(nxtrgtarmlj2_rotmat)
    nxtrgtarmlj2_nodepath.reparentTo(nxtmnp)
    nxtrgtarmlj2_nodepath.setColor(plotcolor[0],plotcolor[1],plotcolor[2],plotcolor[3])
    nxtrgtarmlj2_nodepath.setTransparency(TransparencyAttrib.MAlpha)

    nxtrgtarmlj3_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg/nomat", "nxt_rgtarm_lj3_nm.egg"))
    nxtrgtarmlj3_model = loader.loadModel(nxtrgtarmlj3_filepath)
    nxtrgtarmlj3_nodepath = NodePath("nxtrgtarmlj3_nodepath")
    nxtrgtarmlj3_model.instanceTo(nxtrgtarmlj3_nodepath)
    nxtrgtarmlj3_rotmat = pandageom.cvtMat4(nxtrobot.rgtarm[4]['rotmat'], nxtrobot.rgtarm[4]['linkpos'])
    nxtrgtarmlj3_nodepath.setMat(nxtrgtarmlj3_rotmat)
    nxtrgtarmlj3_nodepath.reparentTo(nxtmnp)
    nxtrgtarmlj3_nodepath.setColor(plotcolor[0],plotcolor[1],plotcolor[2],plotcolor[3])
    nxtrgtarmlj3_nodepath.setTransparency(TransparencyAttrib.MAlpha)

    nxtrgtarmlj4_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg/nomat", "nxt_rgtarm_lj4_nm.egg"))
    nxtrgtarmlj4_model = loader.loadModel(nxtrgtarmlj4_filepath)
    nxtrgtarmlj4_nodepath = NodePath("nxtrgtarmlj4_nodepath")
    nxtrgtarmlj4_model.instanceTo(nxtrgtarmlj4_nodepath)
    nxtrgtarmlj4_rotmat = pandageom.cvtMat4(nxtrobot.rgtarm[5]['rotmat'], nxtrobot.rgtarm[5]['linkpos'])
    nxtrgtarmlj4_nodepath.setMat(nxtrgtarmlj4_rotmat)
    nxtrgtarmlj4_nodepath.reparentTo(nxtmnp)
    nxtrgtarmlj4_nodepath.setColor(plotcolor[0],plotcolor[1],plotcolor[2],plotcolor[3])
    nxtrgtarmlj4_nodepath.setTransparency(TransparencyAttrib.MAlpha)

    # lftarm
    nxtlftarmlj0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg/nomat", "nxt_lftarm_lj0_nm.egg"))
    nxtlftarmlj0_model = loader.loadModel(nxtlftarmlj0_filepath)
    nxtlftarmlj0_nodepath = NodePath("nxtlftarmlj0_nodepath")
    nxtlftarmlj0_model.instanceTo(nxtlftarmlj0_nodepath)
    nxtlftarmlj0_rotmat = pandageom.cvtMat4(nxtrobot.lftarm[1]['rotmat'], nxtrobot.lftarm[1]['linkpos'])
    nxtlftarmlj0_nodepath.setMat(nxtlftarmlj0_rotmat)
    nxtlftarmlj0_nodepath.reparentTo(nxtmnp)
    nxtlftarmlj0_nodepath.setColor(plotcolor[0],plotcolor[1],plotcolor[2],plotcolor[3])
    nxtlftarmlj0_nodepath.setTransparency(TransparencyAttrib.MAlpha)

    nxtlftarmlj1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg/nomat", "nxt_lftarm_lj1_nm.egg"))
    nxtlftarmlj1_model = loader.loadModel(nxtlftarmlj1_filepath)
    nxtlftarmlj1_nodepath = NodePath("nxtlftarmlj1_nodepath")
    nxtlftarmlj1_model.instanceTo(nxtlftarmlj1_nodepath)
    nxtlftarmlj1_rotmat = pandageom.cvtMat4(nxtrobot.lftarm[2]['rotmat'], nxtrobot.lftarm[2]['linkpos'])
    nxtlftarmlj1_nodepath.setMat(nxtlftarmlj1_rotmat)
    nxtlftarmlj1_nodepath.reparentTo(nxtmnp)
    nxtlftarmlj1_nodepath.setColor(plotcolor[0],plotcolor[1],plotcolor[2],plotcolor[3])
    nxtlftarmlj1_nodepath.setTransparency(TransparencyAttrib.MAlpha)

    nxtlftarmlj2_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg/nomat", "nxt_lftarm_lj2_nm.egg"))
    nxtlftarmlj2_model = loader.loadModel(nxtlftarmlj2_filepath)
    nxtlftarmlj2_nodepath = NodePath("nxtlftarmlj2_nodepath")
    nxtlftarmlj2_model.instanceTo(nxtlftarmlj2_nodepath)
    nxtlftarmlj2_rotmat = pandageom.cvtMat4(nxtrobot.lftarm[3]['rotmat'], nxtrobot.lftarm[3]['linkpos'])
    nxtlftarmlj2_nodepath.setMat(nxtlftarmlj2_rotmat)
    nxtlftarmlj2_nodepath.reparentTo(nxtmnp)
    nxtlftarmlj2_nodepath.setColor(plotcolor[0],plotcolor[1],plotcolor[2],plotcolor[3])
    nxtlftarmlj2_nodepath.setTransparency(TransparencyAttrib.MAlpha)

    nxtlftarmlj3_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg/nomat", "nxt_lftarm_lj3_nm.egg"))
    nxtlftarmlj3_model = loader.loadModel(nxtlftarmlj3_filepath)
    nxtlftarmlj3_nodepath = NodePath("nxtlftarmlj3_nodepath")
    nxtlftarmlj3_model.instanceTo(nxtlftarmlj3_nodepath)
    nxtlftarmlj3_rotmat = pandageom.cvtMat4(nxtrobot.lftarm[4]['rotmat'], nxtrobot.lftarm[4]['linkpos'])
    nxtlftarmlj3_nodepath.setMat(nxtlftarmlj3_rotmat)
    nxtlftarmlj3_nodepath.reparentTo(nxtmnp)
    nxtlftarmlj3_nodepath.setColor(plotcolor[0],plotcolor[1],plotcolor[2],plotcolor[3])
    nxtlftarmlj3_nodepath.setTransparency(TransparencyAttrib.MAlpha)

    nxtlftarmlj4_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg/nomat", "nxt_lftarm_lj4_nm.egg"))
    nxtlftarmlj4_model = loader.loadModel(nxtlftarmlj4_filepath)
    nxtlftarmlj4_nodepath = NodePath("nxtlftarmlj4_nodepath")
    nxtlftarmlj4_model.instanceTo(nxtlftarmlj4_nodepath)
    nxtlftarmlj4_rotmat = pandageom.cvtMat4(nxtrobot.lftarm[5]['rotmat'], nxtrobot.lftarm[5]['linkpos'])
    nxtlftarmlj4_nodepath.setMat(nxtlftarmlj4_rotmat)
    nxtlftarmlj4_nodepath.reparentTo(nxtmnp)
    nxtlftarmlj4_nodepath.setColor(plotcolor[0],plotcolor[1],plotcolor[2],plotcolor[3])
    nxtlftarmlj4_nodepath.setTransparency(TransparencyAttrib.MAlpha)

    # rgthnd
    nxtrgthnd = handpkg.newHandNM('rgt', hndcolor=plotcolor)
    nxtlftarmlj5_rotmat = pandageom.cvtMat4(nxtrobot.rgtarm[6]['rotmat'], nxtrobot.rgtarm[6]['linkpos'])
    nxtrgthnd.setMat(nxtlftarmlj5_rotmat)
    nxtrgthnd.reparentTo(nxtmnp)
    if jawwidthrgt is not None:
        nxtrgthnd.setJawwidth(jawwidthrgt)

    # lfthnd
    nxtlfthnd = handpkg.newHandNM('lft', hndcolor=plotcolor)
    nxtlftarmlj5_rotmat = pandageom.cvtMat4(nxtrobot.lftarm[6]['rotmat'], nxtrobot.lftarm[6]['linkpos'])
    nxtlfthnd.setMat(nxtlftarmlj5_rotmat)
    nxtlfthnd.reparentTo(nxtmnp)
    if jawwidthlft is not None:
        nxtlfthnd.setJawwidth(jawwidthlft)

        nxtmnp.setTransparency(TransparencyAttrib.MAlpha)

    return nxtmnp