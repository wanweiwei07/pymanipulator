import os

import numpy as np
from manipulation.grip.robotiq85 import rtq85
from panda3d.core import *
import copy
import math

import pandaplotutils.pandageom as pg

class NxtBall(object):
    """
    generate nxtballs for quick collision detection

    NOTE: it is unnecessary to attach a mnp to render repeatedly
    once attached, it is always there. update the joint angles will change the attached model directly
    """

    def __init__(self):
        """
        load models

        author: weiwei
        date: 20180110
        """

        # b indicates ball, cn indicates collision node
        # self.__bodybcn = CollisionNode("robotbody")
        # self.__rgtupperbcn = CollisionNode("rgtupperarm")
        # self.__rgtlowerbcn = CollisionNode("rgtlowerbcn")
        # self.__rgthandbcn = CollisionNode("rgthandbcn")
        # self.__lftupperbcn = CollisionNode("lftupperarm")
        # self.__lftlowerbcn = CollisionNode("lftlowerbcn")
        # self.__lfthandbcn = CollisionNode("lfthandbcn")
        # self.__bodybcn = None
        # self.__rgtupperbcn = None
        # self.__rgtlowerbcn = None
        # self.__rgthandbcn = None
        # self.__lftupperbcn = None
        # self.__lftlowerbcn = None
        # self.__lfthandbcn = None
        self.__shownp = None

    def __genbcn_direct(self, linklist, radius = 70.0, name = "autogen"):
        """
        gennerate the ball collision node for a link
        the balls are generated along the direction of the links

        :param linkid:
        :param armid:
        :return:
        """

        radius = float(radius)
        balldist = radius

        tmpcolnode = CollisionNode(name)
        for link in linklist:
            spos = link['linkpos']
            epos = link['linkend']
            linklength = np.linalg.norm(epos - spos)
            linkvec = (epos - spos)/linklength
            nball = int(math.ceil(linklength / balldist))
            for i in range(1, nball):
                pos = spos+linkvec*i*(balldist)
                # if i == nball-1:
                #     pos = spos+linkvec*(i-.4)*(balldist)
                colsphere = CollisionSphere(pos[0], pos[1], pos[2], radius)
                tmpcolnode.addSolid(colsphere)
        return tmpcolnode

    def __genbcn_manhattan(self, linklist, radius = 70.0, name = "autogen"):
        """
        gennerate the ball collision node for a link
        the balls are generated along the manhattan directions between joints

        :param linkid:
        :param armid:
        :return:
        """

        radius = float(radius)
        balldist = radius

        tmpcolnode = CollisionNode(name)
        for link, divdir in linklist:
            spos = link['linkpos']
            epos = link['linkend']
            # linkvec direction
            linkdir = np.dot((epos-spos), divdir)*divdir
            linkdirlength = np.linalg.norm(linkdir)
            linkdirnormalized = linkdir/linkdirlength
            nball = int(math.ceil(linkdirlength / balldist))
            for i in range(0, nball):
                pos = spos+linkdirnormalized*i*(balldist)
                colsphere = CollisionSphere(pos[0], pos[1], pos[2], radius)
                tmpcolnode.addSolid(colsphere)
            # linkvec orthogonal direction
            linkorth = epos-spos-linkdir
            linkorthlength = np.linalg.norm(linkorth)
            linkorthnormalized = linkorth/linkorthlength
            nball = int(math.ceil(linkorthlength / balldist))
            for i in range(0, nball):
                pos = spos+linkdir+linkorthnormalized*i*(balldist)
                colsphere = CollisionSphere(pos[0], pos[1], pos[2], radius)
                tmpcolnode.addSolid(colsphere)
        return tmpcolnode

    def genbcndict(self, robot):
        """
        generate the ball collision nodes of a robot

        :param robot: the ur5sgl object, see ur5sgl.py
        :return: a dictionary {'body': colnode, 'rgtarm': colnode,
                'lftarm': colnode, 'rgthnd': colnode, 'lfthnd': colnode}

        author: weiwei
        date: 20180130
        """

        returnlist = {}

        # body
        link0 = robot.rgtarm[0]
        link1 = robot.lftarm[0]
        bodybcn = self.__genbcn_direct([link0, link1], radius = 30)
        returnlist['body'] = bodybcn

        # rgt hand
        link5r = robot.rgtarm[5]
        divdir5r = robot.rgtarm[5]['rotmat'][:,2]
        link6r = robot.rgtarm[6]
        divdir6r = robot.rgtarm[6]['rotmat'][:,2]
        rgthandbcn = self.__genbcn_manhattan([[link5r, divdir5r], [link6r, divdir6r]], radius = 70)
        returnlist['rgthnd'] = rgthandbcn

        # lft hand
        link5l = robot.lftarm[5]
        divdir5l = robot.lftarm[5]['rotmat'][:,2]
        link6l = robot.lftarm[6]
        divdir6l = robot.lftarm[6]['rotmat'][:,2]
        lfthandbcn = self.__genbcn_manhattan([[link5l, divdir5l], [link6l, divdir6l]], radius = 70)
        returnlist['lfthnd'] = lfthandbcn

        return returnlist

    def showbcn(self, base, bcndict):
        """
        show bcnlist to base

        :param bcndict is in the form of
            {'body': colnode, 'rgtarm': colnode, 'lftarm': colnode,
            'rgthnd': colnode, 'lfthnd': colnode}
        :return: null

        author: weiwei
        date: 20170615
        """

        self.unshowbcn()
        self.__shownp = NodePath("collision balls")
        for key, bcn in bcndict.items():
            tst = self.__shownp.attachNewNode(bcn)
            tst.show()
        self.__shownp.reparentTo(base.render)

    def unshowbcn(self):
        """
        show bcnlist to base

        :param bcnlist is in the form of
            [bodybcn, rgtupperbcn, rgtlowerbcn,rgthandbcn,
                    lftupperbcn, lftlowerbcn, lfthandbcn]
        :return: null

        author: weiwei
        date: 20170615
        """

        if self.__shownp is not None:
            self.__shownp.removeNode()
        self.__shownp = None


if __name__=="__main__":

    # show in panda3d
    from direct.showbase.ShowBase import ShowBase
    from panda3d.core import *
    from panda3d.bullet import BulletSphereShape
    import pandaplotutils.pandageom as pandageom
    import pandaplotutils.pandactrl as pandactrl
    from direct.filter.CommonFilters import CommonFilters
    import nxt
    from manipulation.grip.robotiq85 import rtq85nm
    import nxt
    import nxtmesh

    # loadPrcFileData("", "want-directtools #t")
    # loadPrcFileData("", "want-tk #t")

    base = pandactrl.World()

    nxtrobot = nxt.NxtRobot()
    nxtb = NxtBall()
    bcndict = nxtb.genbcndict(nxtrobot)
    nxtb.showbcn(base, bcndict)
    rgthnd = rtq85.Rtq85()
    lfthnd = rtq85.Rtq85()
    nxtmeshgen = nxtmesh.NxtMesh(rgthand=rgthnd, lfthand = lfthnd)
    nxtmnp = nxtmeshgen.genmnp(nxtrobot)
    nxtmnp.reparentTo(base.render)

    base.run()

