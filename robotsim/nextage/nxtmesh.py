import os

import numpy as np
from manipulation.grip.robotiq85 import rtq85
from panda3d.core import *
import copy

import pandaplotutils.pandageom as pg

class NxtMesh(object):
    """
    generate nxtmesh

    NOTE: it is unnecessary to attach a mnp to render repeatedly
    once attached, it is always there. update the joint angles will change the attached model directly
    """

    def __init__(self, rgthand = None, lfthand = None):
        """
        load models

        :param rgthand: hand object
        :param lfthand: hand object
               the two hands could be different
        author: weiwei
        date: 20180109
        """

        ##########
        ### load the model files of the robots
        ##########
        this_dir, this_filename = os.path.split(__file__)

        nxtwaist_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_waist.egg"))
        nxtbody_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_body.egg"))
        nxthead_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_head.egg"))

        self.__nxtwaist_nodepath = NodePath("nxtwaist")
        self.__nxtbody_nodepath = NodePath("nxtbody")
        self.__nxthead_nodepath = NodePath("nxthead")

        nxtwaist_model = loader.loadModel(nxtwaist_filepath)
        nxtbody_model = loader.loadModel(nxtbody_filepath)
        nxthead_model = loader.loadModel(nxthead_filepath)

        nxtwaist_model.instanceTo(self.__nxtwaist_nodepath)
        nxtbody_model.instanceTo(self.__nxtbody_nodepath)
        nxthead_model.instanceTo(self.__nxthead_nodepath)

        self.__nxtbody_nodepath.reparentTo(self.__nxtwaist_nodepath)
        self.__nxthead_nodepath.setZ(569.5)
        self.__nxthead_nodepath.reparentTo(self.__nxtwaist_nodepath)

        # rgtarm
        nxtrgtarmlj0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_rgtarm_lj0.egg"))
        nxtrgtarmlj0_model = loader.loadModel(nxtrgtarmlj0_filepath)
        self.__nxtrgtarmlj0_nodepath = NodePath("nxtrgtarmlj0_nodepath")
        nxtrgtarmlj0_model.instanceTo(self.__nxtrgtarmlj0_nodepath)

        nxtrgtarmlj1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_rgtarm_lj1.egg"))
        nxtrgtarmlj1_model = loader.loadModel(nxtrgtarmlj1_filepath)
        self.__nxtrgtarmlj1_nodepath = NodePath("nxtrgtarmlj1_nodepath")
        nxtrgtarmlj1_model.instanceTo(self.__nxtrgtarmlj1_nodepath)

        nxtrgtarmlj2_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_rgtarm_lj2.egg"))
        nxtrgtarmlj2_model = loader.loadModel(nxtrgtarmlj2_filepath)
        self.__nxtrgtarmlj2_nodepath = NodePath("nxtrgtarmlj2_nodepath")
        nxtrgtarmlj2_model.instanceTo(self.__nxtrgtarmlj2_nodepath)

        nxtrgtarmlj3_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_rgtarm_lj3.egg"))
        nxtrgtarmlj3_model = loader.loadModel(nxtrgtarmlj3_filepath)
        self.__nxtrgtarmlj3_nodepath = NodePath("nxtrgtarmlj3_nodepath")
        nxtrgtarmlj3_model.instanceTo(self.__nxtrgtarmlj3_nodepath)

        nxtrgtarmlj4_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_rgtarm_lj4.egg"))
        nxtrgtarmlj4_model = loader.loadModel(nxtrgtarmlj4_filepath)
        self.__nxtrgtarmlj4_nodepath = NodePath("nxtrgtarmlj4_nodepath")
        nxtrgtarmlj4_model.instanceTo(self.__nxtrgtarmlj4_nodepath)

        # lftarm
        nxtlftarmlj0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_lftarm_lj0.egg"))
        nxtlftarmlj0_model = loader.loadModel(nxtlftarmlj0_filepath)
        self.__nxtlftarmlj0_nodepath = NodePath("nxtlftarmlj0_nodepath")
        nxtlftarmlj0_model.instanceTo(self.__nxtlftarmlj0_nodepath)

        nxtlftarmlj1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_lftarm_lj1.egg"))
        nxtlftarmlj1_model = loader.loadModel(nxtlftarmlj1_filepath)
        self.__nxtlftarmlj1_nodepath = NodePath("nxtlftarmlj1_nodepath")
        nxtlftarmlj1_model.instanceTo(self.__nxtlftarmlj1_nodepath)

        nxtlftarmlj2_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_lftarm_lj2.egg"))
        nxtlftarmlj2_model = loader.loadModel(nxtlftarmlj2_filepath)
        self.__nxtlftarmlj2_nodepath = NodePath("nxtlftarmlj2_nodepath")
        nxtlftarmlj2_model.instanceTo(self.__nxtlftarmlj2_nodepath)

        nxtlftarmlj3_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_lftarm_lj3.egg"))
        nxtlftarmlj3_model = loader.loadModel(nxtlftarmlj3_filepath)
        self.__nxtlftarmlj3_nodepath = NodePath("nxtlftarmlj3_nodepath")
        nxtlftarmlj3_model.instanceTo(self.__nxtlftarmlj3_nodepath)

        nxtlftarmlj4_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_lftarm_lj4.egg"))
        nxtlftarmlj4_model = loader.loadModel(nxtlftarmlj4_filepath)
        self.__nxtlftarmlj4_nodepath = NodePath("nxtlftarmlj4_nodepath")
        nxtlftarmlj4_model.instanceTo(self.__nxtlftarmlj4_nodepath)

        # rgthnd
        self.rgthnd = rgthand

        # lfthnd
        self.lfthnd = lfthand

        ##########
        ### load the model files of sticks
        ##########

        self.pggen = pg.PandaGeomGen()

    def gensnp(self, robot, rgtrbga=np.array([.5 ,0 ,0 ,0]), lftrgba=np.array([.5 ,0 ,0 ,0])):
        """
        generate the stick model of the nextage robot in panda3d
        snp means stick nodepath

        :param nxtrobot: the nxtrobot object, see Hrp5robot.py
        :param rgtrbga: color of right arm
        :param lftrgba: color of left arm
        :return: null

        author: weiwei
        date: 20180109
        """

        nxtstick = NodePath("nxtstick")
        i = 0
        while i != -1:
            sticknp = self.pggen.genDumbbell(spos=robot.rgtarm[i]['linkpos'],
                                             epos=robot.rgtarm[i]['linkend'],
                                             thickness=20, rgba=rgtrbga)
            i = robot.rgtarm[i]['child']
            sticknp.reparentTo(nxtstick)
        i = 0
        while i != -1:
            sticknp = self.pggen.genDumbbell(spos=robot.lftarm[i]['linkpos'],
                                             epos=robot.lftarm[i]['linkend'],
                                             thickness=20, rgba=lftrgba)
            i = robot.lftarm[i]['child']
            sticknp.reparentTo(nxtstick)

        return nxtstick

    def genmnp(self, robot, jawwidthrgt = 0, jawwidthlft = 0,
               toggleendcoord = True, togglejntscoord = False, name = 'robotmesh'):
        """
        generate a panda3d nodepath for the robot
        mnp indicates this function generates a mesh nodepath

        :param robot: the robot object, see robot.py
        :return: a nodepath which is ready to be plotted using plotmesh

        author: weiwei
        date: 20180109
        """

        robotmesh_nodepath = NodePath(name)

        identmat4 = Mat4.identMat()
        # body

        nxtwaist_rotmat = pg.cvtMat4(robot.base['rotmat'])
        self.__nxtwaist_nodepath.setMat(nxtwaist_rotmat)

        # rgtarm
        nxtrgtarmlj0_rotmat = pg.cvtMat4(robot.rgtarm[1]['rotmat'], robot.rgtarm[1]['linkpos'])
        self.__nxtrgtarmlj0_nodepath.setMat(nxtrgtarmlj0_rotmat)
        nxtrgtarmlj1_rotmat = pg.cvtMat4(robot.rgtarm[2]['rotmat'], robot.rgtarm[2]['linkpos'])
        self.__nxtrgtarmlj1_nodepath.setMat(nxtrgtarmlj1_rotmat)
        nxtrgtarmlj2_rotmat = pg.cvtMat4(robot.rgtarm[3]['rotmat'], robot.rgtarm[3]['linkpos'])
        self.__nxtrgtarmlj2_nodepath.setMat(nxtrgtarmlj2_rotmat)
        nxtrgtarmlj3_rotmat = pg.cvtMat4(robot.rgtarm[4]['rotmat'], robot.rgtarm[4]['linkpos'])
        self.__nxtrgtarmlj3_nodepath.setMat(nxtrgtarmlj3_rotmat)
        nxtrgtarmlj4_rotmat = pg.cvtMat4(robot.rgtarm[5]['rotmat'], robot.rgtarm[5]['linkpos'])
        self.__nxtrgtarmlj4_nodepath.setMat(nxtrgtarmlj4_rotmat)

        # lftarm
        nxtlftarmlj0_rotmat = pg.cvtMat4(robot.lftarm[1]['rotmat'], robot.lftarm[1]['linkpos'])
        self.__nxtlftarmlj0_nodepath.setMat(nxtlftarmlj0_rotmat)
        nxtlftarmlj1_rotmat = pg.cvtMat4(robot.lftarm[2]['rotmat'], robot.lftarm[2]['linkpos'])
        self.__nxtlftarmlj1_nodepath.setMat(nxtlftarmlj1_rotmat)
        nxtlftarmlj2_rotmat = pg.cvtMat4(robot.lftarm[3]['rotmat'], robot.lftarm[3]['linkpos'])
        self.__nxtlftarmlj2_nodepath.setMat(nxtlftarmlj2_rotmat)
        nxtlftarmlj3_rotmat = pg.cvtMat4(robot.lftarm[4]['rotmat'], robot.lftarm[4]['linkpos'])
        self.__nxtlftarmlj3_nodepath.setMat(nxtlftarmlj3_rotmat)
        nxtlftarmlj4_rotmat = pg.cvtMat4(robot.lftarm[5]['rotmat'], robot.lftarm[5]['linkpos'])
        self.__nxtlftarmlj4_nodepath.setMat(nxtlftarmlj4_rotmat)

        self.__nxtwaist_nodepath.reparentTo(robotmesh_nodepath)
        self.__nxtrgtarmlj0_nodepath.reparentTo(robotmesh_nodepath)
        self.__nxtrgtarmlj1_nodepath.reparentTo(robotmesh_nodepath)
        self.__nxtrgtarmlj2_nodepath.reparentTo(robotmesh_nodepath)
        self.__nxtrgtarmlj3_nodepath.reparentTo(robotmesh_nodepath)
        self.__nxtrgtarmlj4_nodepath.reparentTo(robotmesh_nodepath)
        self.__nxtlftarmlj0_nodepath.reparentTo(robotmesh_nodepath)
        self.__nxtlftarmlj1_nodepath.reparentTo(robotmesh_nodepath)
        self.__nxtlftarmlj2_nodepath.reparentTo(robotmesh_nodepath)
        self.__nxtlftarmlj3_nodepath.reparentTo(robotmesh_nodepath)
        # self.__nxtlftarmlj4_nodepath.reparentTo(robotmesh_nodepath)

        # rgthand
        nxtrgtarmlj5_rotmat = pg.cvtMat4(robot.rgtarm[6]['rotmat'], robot.rgtarm[6]['linkpos'])
        if self.rgthnd is not None:
            self.rgthnd.setMat(nxtrgtarmlj5_rotmat)
            self.rgthnd.setJawwidth(jawwidthrgt)
            self.rgthnd.reparentTo(robotmesh_nodepath)

        # lfthand
        nxtlftarmlj5_rotmat = pg.cvtMat4(robot.lftarm[6]['rotmat'], robot.lftarm[6]['linkpos'])
        if self.lfthnd is not None:
            self.lfthnd.setMat(nxtlftarmlj5_rotmat)
            self.lfthnd.setJawwidth(jawwidthlft)
            self.lfthnd.reparentTo(robotmesh_nodepath)

        # endcoord
        if toggleendcoord:
            self.pggen.plotAxis(robotmesh_nodepath,
                            spos = robot.lftarm[6]['linkend'],
                            pandamat4 = nxtlftarmlj5_rotmat)
            self.pggen.plotAxis(robotmesh_nodepath,
                            spos = robot.rgtarm[6]['linkend'],
                            pandamat4 = nxtrgtarmlj5_rotmat)
        # toggle all coord
        if togglejntscoord:
            for i in range(1, 7):
                self.pggen.plotAxis(robotmesh_nodepath, spos = robot.lftarm[i]['linkpos'],
                                pandamat4 = pg.cvtMat4(robot.lftarm[i-1]['rotmat'],
                                                       robot.lftarm[i]['linkpos']))

        return copy.deepcopy(robotmesh_nodepath)
