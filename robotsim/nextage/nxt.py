import numpy as np
import exceptions as ep
import utils.robotmath as rm
import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pg
import nxtik

class NxtRobot():
    def __init__(self):
        # initjnts has 16 elements where the first three are the waist yaw, head pitch, and head yaw
        # the following 12 elements are the joint angles of right and left arms
        # the last element is the moving speed
        self.__name = 'nextage'
        self.__initjnts = np.array([0,0,0,-15,0,-143,0,0,0,15,0,-143,0,0,0]);
        self.__rgtarm = self.__initrgtlj()
        self.__lftarm = self.__initlftlj()
        self.__base = self.__rgtarm[0]
        self.__targetjoints = [1,2,3,4,5,6]
        self.goinitpose()

    @property
    def name(self):
        # read-only property
        return self.__name

    @property
    def initjnts(self):
        # read-only property
        return self.__initjnts

    @property
    def initrgtjntsr(self):
        # read-only property
        return np.array([self.__initjnts[i] for i in [0,3,4,5,6,7,8]])

    @property
    def initlftjntsr(self):
        # read-only property
        return  np.array([self.__initjnts[i] for i in [0,9,10,11,12,13,14]])

    @property
    def initrgtjnts(self):
        # read-only property
        return np.array([self.__initjnts[i] for i in [3,4,5,6,7,8]])

    @property
    def initlftjnts(self):
        # read-only property
        return  np.array([self.__initjnts[i] for i in [9,10,11,12,13,14]])

    @property
    def rgtarm(self):
        # read-only property
        return self.__rgtarm

    @property
    def lftarm(self):
        # read-only property
        return self.__lftarm

    @property
    def base(self):
        # read-only property
        return self.__base

    @property
    def targetjoints(self):
        # read-only property
        return self.__targetjoints

    def getinitarmjnts(self, armid = 'rgt'):
        """
        get init arm jnts by specifying armid

        :param armid:
        :return:

        date: 20180602 for ik udpate
        """

        if armid!="rgt" and armid!="lft":
            raise ep.ValueError

        armjnts = self.__initjnts[3:9]
        if armid == "lft":
            armjnts = self.__initjnts[9:]

        return armjnts


    def movewaist(self, rotangle=0):
        """
        rotate the base of the robot

        :param rotangle: in degree
        :return: null

        author: weiwei
        date: 20161107
        """

        # right arm
        self.rgtarm[0]['rotangle'] = rotangle
        self.rgtarm[0]['rotmat'] = rm.rodrigues(self.rgtarm[0]['rotax'], self.rgtarm[0]['rotangle'])
        self.rgtarm[0]['linkend'] = np.squeeze(np.dot(self.rgtarm[0]['rotmat'], self.rgtarm[0]['linkvec'].reshape((-1,))))+self.rgtarm[0]['linkpos']
        self.__updatefk(self.rgtarm)

        # left arm
        self.lftarm[0]['rotangle'] = rotangle
        self.lftarm[0]['rotmat'] = rm.rodrigues(self.lftarm[0]['rotax'], self.lftarm[0]['rotangle'])
        self.lftarm[0]['linkend'] = np.squeeze(np.dot(self.lftarm[0]['rotmat'], self.lftarm[0]['linkvec'].reshape((-1,))))+self.lftarm[0]['linkpos']
        self.__updatefk(self.lftarm)

    # def movearmfk6(self, armjnts, armid="rgt"):
    #     """
    #     move the 6 joints of armlj using forward kinematics
    #
    #     :param armjnts: a 1-by-6 ndarray where each element indicates the angle of a joint (in degree)
    #     :param armid: a string indicating the rgtlj or lftlj robot structure, could "lft" or "rgt"
    #     :return: null
    #
    #     author: weiwei
    #     date: 20161205
    #     """
    #
    #     if armid!="rgt" and armid!="lft":
    #         raise ep.ValueError
    #
    #     armlj = self.rgtarm
    #     if armid == "lft":
    #         armlj = self.lftarm
    #     i = 1
    #     while i!=-1:
    #         armlj[i]['rotangle'] = armjnts[i-1]
    #         i = armlj[i]['child']
    #     self.__updatefk(armlj)

    def movearmfk(self, armjnts, armid="rgt"):
        """
        move the 6 joints of armlj using forward kinematics
        this function will replace movearmfk6

        :param armjnts: a 1-by-6 ndarray where each element indicates the angle of a joint (in degree)
        :param armid: a string indicating the rgtlj or lftlj robot structure, could "lft" or "rgt"
        :return: null

        author: weiwei
        date: 20161205
        """

        if armid!="rgt" and armid!="lft":
            raise ep.ValueError

        armlj = self.rgtarm
        if armid == "lft":
            armlj = self.lftarm

        counter = 0
        for i in self.__targetjoints:
            armlj[i]['rotangle'] = armjnts[counter]
            counter += 1

        self.__updatefk(armlj)

    def movearmfkr(self, armjnts, armid ="rgt"):
        """
        move the 7 joints of armlj using forward kinematics

        :param armjnts: a 1-by-7 ndarray where the 1t ele is waist, the second is the targetjoints list
        :param armid: a string indicating the rgtlj or lftlj robot structure, could "lft" or "rgt"
        :return: null

        author: weiwei
        date: 20161107
        """

        if armid!="rgt" and armid!="lft":
            raise ep.ValueError

        armlj = self.rgtarm
        if armid == "lft":
            armlj = self.lftarm

        counter = 0
        for i in self.__targetjoints:
            armlj[i]['rotangle'] = armjnts[1][counter]
            counter += 1

        self.movewaist(rotangle=armjnts[0])

    # def movejnts15(self, nxjnts):
    #     """
    #     move all joints of the nextage robo
    #
    #     :param nxjnts: the definition as self.initjntss
    #     :return: null
    #
    #     author: weiwei
    #     date: 20161108
    #     """
    #
    #     narmjoints = len(self.__targetjoints)
    #     # right arm
    #     i = 1
    #     while i != -1:
    #         self.rgtarm[i]['rotangle'] = nxjnts[i+2]
    #         i = self.rgtarm[i]['child']
    #     # left arm
    #     i = 1
    #     while i != -1:
    #         self.lftarm[i]['rotangle'] = nxjnts[i+2+narmjoints]
    #         i = self.lftarm[i]['child']
    #
    #     self.movewaist(nxjnts[0])

    def movealljnts(self, nxjnts):
        """
        move all joints of the nextage robo

        :param nxjnts: the definition as self.initjntss
        :return: null

        author: weiwei
        date: 20161108
        """

        narmjoints = len(self.__targetjoints)
        # right arm
        i = 1
        while i != -1:
            self.rgtarm[i]['rotangle'] = nxjnts[i+2]
            i = self.rgtarm[i]['child']
        # left arm
        i = 1
        while i != -1:
            self.lftarm[i]['rotangle'] = nxjnts[i+2+narmjoints]
            i = self.lftarm[i]['child']

        self.movewaist(nxjnts[0])

    def goinitpose(self):
        """
        move the nextage robot to initial pose

        :return: null

        author: weiwei
        date: 20161108
        """

        self.movealljnts(self.initjnts)

    # def getarmjnts6(self, armid="rgt"):
    #     """
    #     get the 6 joints of the specified armid
    #
    #     :param armid:
    #     :return: armjnts: a 1-by-6 numpy ndarray
    #
    #     author: weiwei
    #     date: 20161111, tsukuba
    #     """
    #
    #     if armid!="rgt" and armid!="lft":
    #         raise ep.ValueError
    #
    #     armlj = self.rgtarm
    #     if armid == "lft":
    #         armlj = self.lftarm
    #
    #     armjnts = np.zeros(6)
    #     for i in range(6):
    #         armjnts[i] = armlj[i+1]['rotangle']
    #
    #     return armjnts

    def getarmjnts(self, armid="rgt"):
        """
        get the target joints of the specified armid

        :param armid:
        :return: armjnts: a 1-by-6 numpy ndarray

        author: weiwei
        date: 20161111, tsukuba
        """

        if armid!="rgt" and armid!="lft":
            raise ep.ValueError

        armlj = self.rgtarm
        if armid == "lft":
            armlj = self.lftarm

        armjnts = np.zeros(len(self.__targetjoints))
        counter = 0
        for i in self.__targetjoints:
            armjnts[counter] = armlj[i]['rotangle']
            counter += 1

        return armjnts

    def getjntwaist(self):
        """
        get the rot angle of robot waist

        :return: rot angle in degree

         author: weiwei
         date: 20161216, sapporo
        """

        return self.base['rotangle']

    def chkrng(self, armjnts, armid="rgt"):
        """
        check if the given armjnts is inside the oeprating range of the speificed armid
        this function doesn't check the waist

        :param armjnts: a 1-by-6 numpy ndarray
        :param armid: a string "rgt" or "lft"
        :return: True or False indicating inside the range or not

        author: weiwei
        date: 20161111
        """

        if armid!="rgt" and armid!="lft":
            raise ep.ValueError

        armlj = self.rgtarm
        if armid == "lft":
            armlj = self.lftarm

        counter = 0
        for i in self.__targetjoints:
            if armjnts[counter] < armlj[i]["rngmin"] or armjnts[counter]>armlj[i]["rngmax"]:
                # print "Joint "+ str(i) + " of the " + armid + " arm is out of range"
                # print "Angle is " + str(armjnts[i])
                # print "Range is (" + str(armlj[i+1]["rngmin"]) + ", " + str(armlj[i+1]["rngmax"]) + ")"
                return False
            counter += 1

        return True

    def chkrngdrag(self, armjnts, armid="rgt"):
        """
        check if the given armjnts is inside the oeprating range of the speificed armid
        this function doesn't check the waist
        The joint angles out of range will be pulled back to their maxima

        :param armjnts: a 1-by-6 numpy ndarray indicating the targejoints of a manipulator
        :param armid: a string "rgt" or "lft"
        :return: Two parameters, one is true or false indicating if the joint angles are inside the range or not
                The other is the joint angles after draggin.
                If the joints were not dragged, the same joint angles will be returned

        author: weiwei
        date: 20161205
        """

        if armid!="rgt" and armid!="lft":
            raise ep.ValueError

        armlj = self.rgtarm
        if armid == "lft":
            armlj = self.lftarm

        counter = 0
        bdragged = True
        jntanglesdrag = []
        for i in self.__targetjoints:
            if armjnts[counter] < armlj[i]["rngmin"]:
                # print "Joint "+ str(i) + " of the " + armid + " arm is out of range"
                # print "Angle is " + str(armjnts[counter])
                # print "Range is (" + str(armlj[i]["rngmin"]) + ", " + str(armlj[i]["rngmax"]) + ")"
                bdragged = True
                jntanglesdrag.append(armlj[i]["rngmin"])
            elif armjnts[counter] > armlj[i]["rngmax"]:
                bdragged = True
                jntanglesdrag.append(armlj[i]["rngmax"])
            else:
                bdragged = False
                jntanglesdrag.append(armjnts[counter])

            counter += 1

        return bdragged, jntanglesdrag

    def __initrgtlj(self):
        '''
        Init the structure of hiro's rgt arm links and joints

        ## output
        rgtlj:
            a list of dictionaries with each dictionary holding name, mother, child,
        linkpos (link position), linkvec (link vector), rotmat (orientation), rotax (rotation axis of a joint),
        rotangle (rotation angle of the joint around rotax)
        linkend (the end position of a link) is computed using rotmat, rotax, linkpos, and linklen
        lj means link and joint, the joint attached to the link is at the linkend

        ## note
        x is facing forward, y is facing left, z is facing upward
        each element of rgtlj is a dictionary
        rgtlj[i]['linkpos'] indicates the position of a link
        rgtlj[i]['linkvec'] indicates the vector of a link that points from start to end
        rgtlj[i]['rotmat'] indicates the frame of this link
        rgtlj[i]['rotax'] indicates the rotation axis of the link
        rgtlj[i]['rotangle'] indicates the rotation angle of the link around the rotax
        rgtlj[i]['linkend'] indicates the end position of the link (passively computed)

        ## more note:
        rgtlj[1]['linkpos'] is the position of the first joint
        rgtlj[i]['linkend'] is the same as rgtlj[i+1]['linkpos'],
        I am keeping this value for the eef (end-effector)

        ## even more note:
        joint is attached to the linkpos of a link
        for the first link, the joint is fixed and the rotax = 0,0,0

        ## inherentR is only available at the first link

        author: weiwei
        date: 20160615
        '''

        # create a arm with six joints
        rgtlj = [dict() for i in range(7)]
        rngsafemargin = 5

        # the 0th link and joint
        rgtlj[0]['name'] = 'link0'
        rgtlj[0]['mother'] = -1
        rgtlj[0]['child'] = 1
        rgtlj[0]['linkpos'] = np.array([0,0,0])
        rgtlj[0]['linkvec'] = np.array([0,-145,370.296])
        rgtlj[0]['rotax'] = np.array([0,0,1])
        rgtlj[0]['rotangle'] = 0
        rgtlj[0]['rotmat'] = np.eye(3)
        rgtlj[0]['linkend'] = np.dot(rgtlj[0]['rotmat'], rgtlj[0]['linkvec'])+rgtlj[0]['linkpos']

        # the 1st joint and link
        rgtlj[1]['name'] = 'link1'
        rgtlj[1]['mother'] = 0
        rgtlj[1]['child'] = 2
        rgtlj[1]['linkpos'] = rgtlj[0]['linkend']
        rgtlj[1]['linkvec'] = np.array([0,-95,0])
        rgtlj[1]['rotax'] = np.array([0,0,1])
        rgtlj[1]['rotangle'] = 0
        rgtlj[1]['inherentR'] = rm.rodrigues([1,0,0], 15)
        rgtlj[1]['rotmat'] = np.dot(np.dot(rgtlj[0]['rotmat'], rgtlj[1]['inherentR']), \
                                    rm.rodrigues(rgtlj[1]['rotax'], rgtlj[1]['rotangle']))
        rgtlj[1]['linkend'] = np.dot(rgtlj[1]['rotmat'], rgtlj[1]['linkvec'])+rgtlj[1]['linkpos']
        rgtlj[1]['rngmin'] = -(88-rngsafemargin)
        rgtlj[1]['rngmax'] = +(88-rngsafemargin)

        # the 2nd joint and link
        rgtlj[2]['name'] = 'link2'
        rgtlj[2]['mother'] = 1
        rgtlj[2]['child'] = 3
        rgtlj[2]['linkpos'] = rgtlj[1]['linkend']
        rgtlj[2]['linkvec'] = np.array([0,0,-250])
        rgtlj[2]['rotax'] = np.array([0,1,0])
        rgtlj[2]['rotangle'] = 0
        rgtlj[2]['rotmat'] = np.dot(rgtlj[1]['rotmat'], rm.rodrigues(rgtlj[2]['rotax'], rgtlj[2]['rotangle']))
        rgtlj[2]['linkend'] = np.dot(rgtlj[2]['rotmat'], rgtlj[2]['linkvec'])+rgtlj[2]['linkpos']
        rgtlj[2]['rngmin'] = -(140-rngsafemargin)
        rgtlj[2]['rngmax'] = +(60-rngsafemargin)

        # the 3rd joint and link
        rgtlj[3]['name'] = 'link3'
        rgtlj[3]['mother'] = 2
        rgtlj[3]['child'] = 4
        rgtlj[3]['linkpos'] = rgtlj[2]['linkend']
        rgtlj[3]['linkvec'] = np.array([-30,0,-235])
        rgtlj[3]['rotax'] = np.array([0,1,0])
        rgtlj[3]['rotangle'] = 0
        rgtlj[3]['rotmat'] = np.dot(rgtlj[2]['rotmat'], rm.rodrigues(rgtlj[3]['rotax'], rgtlj[3]['rotangle']))
        rgtlj[3]['linkend'] = np.dot(rgtlj[3]['rotmat'], rgtlj[3]['linkvec'])+rgtlj[3]['linkpos']
        rgtlj[3]['rngmin'] = -(158-rngsafemargin)
        rgtlj[3]['rngmax'] = +(0-rngsafemargin)

        # the 4th joint and link
        rgtlj[4]['name'] = 'link4'
        rgtlj[4]['mother'] = 3
        rgtlj[4]['child'] = 5
        rgtlj[4]['linkpos'] = rgtlj[3]['linkend']
        rgtlj[4]['linkvec'] = np.array([0,0,0])
        rgtlj[4]['rotax'] = np.array([0,0,1])
        rgtlj[4]['rotangle'] = 0
        rgtlj[4]['rotmat'] = np.dot(rgtlj[3]['rotmat'], rm.rodrigues(rgtlj[4]['rotax'], rgtlj[4]['rotangle']))
        rgtlj[4]['linkend'] = np.dot(rgtlj[4]['rotmat'], rgtlj[4]['linkvec'])+rgtlj[4]['linkpos']
        rgtlj[4]['rngmin'] = -(165-rngsafemargin)
        rgtlj[4]['rngmax'] = +(105-rngsafemargin)

        # the 5th joint and link
        rgtlj[5]['name'] = 'link5'
        rgtlj[5]['mother'] = 4
        rgtlj[5]['child'] = 6
        rgtlj[5]['linkpos'] = rgtlj[4]['linkend']
        # the reason 100 is used: -40 is where the ee starts, -60 is where the rqt85 hand locates
        rgtlj[5]['linkvec'] = np.array([-100,0,-90])
        rgtlj[5]['rotax'] = np.array([0,1,0])
        rgtlj[5]['rotangle'] = 0
        rgtlj[5]['rotmat'] = np.dot(rgtlj[4]['rotmat'], rm.rodrigues(rgtlj[5]['rotax'], rgtlj[5]['rotangle']))
        rgtlj[5]['linkend'] = np.dot(rgtlj[5]['rotmat'], rgtlj[5]['linkvec'])+rgtlj[5]['linkpos']
        rgtlj[5]['rngmin'] = -(100-rngsafemargin)
        rgtlj[5]['rngmax'] = +(100-rngsafemargin)

        # the 6th joint and link
        rgtlj[6]['name'] = 'link6'
        rgtlj[6]['mother'] = 5
        rgtlj[6]['child'] = -1
        rgtlj[6]['linkpos'] = rgtlj[5]['linkend']
        # the reason 130 is used: the fgrcenter is defined somewhere inside the finger pads
        rgtlj[6]['linkvec'] = np.array([-145,0,0])
        rgtlj[6]['rotax'] = np.array([1,0,0])
        rgtlj[6]['rotangle'] = 0
        rgtlj[6]['rotmat'] = np.dot(rgtlj[5]['rotmat'], rm.rodrigues(rgtlj[6]['rotax'], rgtlj[6]['rotangle']))
        rgtlj[6]['linkend'] = np.dot(rgtlj[6]['rotmat'], rgtlj[6]['linkvec'])+rgtlj[6]['linkpos']
        rgtlj[6]['rngmin'] = -(163-rngsafemargin)
        rgtlj[6]['rngmax'] = +(163-rngsafemargin)

        return rgtlj

    def __initlftlj(self):
        '''
        Init the structure of hiro's lft arm links and joints, everything is the same as initrgtlj

        author: weiwei
        date: 20161107
        '''

        lftlj = [dict() for i in range(7)]
        rngsafemargin = 5

        lftlj[0]['name'] = 'link0'
        lftlj[0]['mother'] = -1
        lftlj[0]['child'] = 1
        lftlj[0]['linkpos'] = np.array([0,0,0])
        lftlj[0]['linkvec'] = np.array([0,145,370.296])
        lftlj[0]['rotax'] = np.array([0,0,1])
        lftlj[0]['rotangle'] = 0
        lftlj[0]['rotmat'] = np.eye(3)
        lftlj[0]['linkend'] = np.dot(lftlj[0]['rotmat'], lftlj[0]['linkvec'])+lftlj[0]['linkpos']

        lftlj[1]['name'] = 'link1'
        lftlj[1]['mother'] = 0
        lftlj[1]['child'] = 2
        lftlj[1]['linkpos'] = lftlj[0]['linkend']
        lftlj[1]['linkvec'] = np.array([0,95,0])
        lftlj[1]['rotax'] = np.array([0,0,1])
        lftlj[1]['rotangle'] = 0
        lftlj[1]['inherentR'] = rm.rodrigues([1,0,0],-15)
        lftlj[1]['rotmat'] = np.dot(np.dot(lftlj[0]['rotmat'], lftlj[1]['inherentR']), \
                                    rm.rodrigues(lftlj[1]['rotax'], lftlj[1]['rotangle']))
        lftlj[1]['linkend'] = np.dot(lftlj[1]['rotmat'], lftlj[1]['linkvec'])+lftlj[1]['linkpos']
        lftlj[1]['rngmin'] = -(88-rngsafemargin)
        lftlj[1]['rngmax'] = +(88-rngsafemargin)

        lftlj[2]['name'] = 'link2'
        lftlj[2]['mother'] = 1
        lftlj[2]['child'] = 3
        lftlj[2]['linkpos'] = lftlj[1]['linkend']
        lftlj[2]['linkvec'] = np.array([0,0,-250])
        lftlj[2]['rotax'] = np.array([0,1,0])
        lftlj[2]['rotangle'] = 0
        lftlj[2]['rotmat'] = np.dot(lftlj[1]['rotmat'], rm.rodrigues(lftlj[2]['rotax'], lftlj[2]['rotangle']))
        lftlj[2]['linkend'] = np.dot(lftlj[2]['rotmat'], lftlj[2]['linkvec'])+lftlj[2]['linkpos']
        lftlj[2]['rngmin'] = -(140-rngsafemargin)
        lftlj[2]['rngmax'] = +(60-rngsafemargin)


        lftlj[3]['name'] = 'link3'
        lftlj[3]['mother'] = 2
        lftlj[3]['child'] = 4
        lftlj[3]['linkpos'] = lftlj[2]['linkend']
        lftlj[3]['linkvec'] = np.array([-30,0,-235])
        lftlj[3]['rotax'] = np.array([0,1,0])
        lftlj[3]['rotangle'] = 0
        lftlj[3]['rotmat'] = np.dot(lftlj[2]['rotmat'], rm.rodrigues(lftlj[3]['rotax'], lftlj[3]['rotangle']))
        lftlj[3]['linkend'] = np.dot(lftlj[3]['rotmat'], lftlj[3]['linkvec'])+lftlj[3]['linkpos']
        lftlj[3]['rngmin'] = -(158-rngsafemargin)
        lftlj[3]['rngmax'] = +(0-rngsafemargin)

        lftlj[4]['name'] = 'link4'
        lftlj[4]['mother'] = 3
        lftlj[4]['child'] = 5
        lftlj[4]['linkpos'] = lftlj[3]['linkend']
        lftlj[4]['linkvec'] = np.array([0,0,0])
        lftlj[4]['rotax'] = np.array([0,0,1])
        lftlj[4]['rotangle'] = 0
        lftlj[4]['rotmat'] = np.dot(lftlj[3]['rotmat'], rm.rodrigues(lftlj[4]['rotax'], lftlj[4]['rotangle']))
        lftlj[4]['linkend'] = np.dot(lftlj[4]['rotmat'], lftlj[4]['linkvec'])+lftlj[4]['linkpos']
        lftlj[4]['rngmin'] = -(105-rngsafemargin)
        lftlj[4]['rngmax'] = +(165-rngsafemargin)

        lftlj[5]['name'] = 'link5'
        lftlj[5]['mother'] = 4
        lftlj[5]['child'] = 6
        lftlj[5]['linkpos'] = lftlj[4]['linkend']
        lftlj[5]['linkvec'] = np.array([-100,0,-90])
        lftlj[5]['rotax'] = np.array([0,1,0])
        lftlj[5]['rotangle'] = 0
        lftlj[5]['rotmat'] = np.dot(lftlj[4]['rotmat'], rm.rodrigues(lftlj[5]['rotax'], lftlj[5]['rotangle']))
        lftlj[5]['linkend'] = np.dot(lftlj[5]['rotmat'], lftlj[5]['linkvec'])+lftlj[5]['linkpos']
        lftlj[5]['rngmin'] = -(100-rngsafemargin)
        lftlj[5]['rngmax'] = +(100-rngsafemargin)

        lftlj[6]['name'] = 'link6'
        lftlj[6]['mother'] = 5
        lftlj[6]['child'] = -1
        lftlj[6]['linkpos'] = lftlj[5]['linkend']
        lftlj[6]['linkvec'] = np.array([-145,0,0])
        lftlj[6]['rotax'] = np.array([1,0,0])
        lftlj[6]['rotangle'] = 0
        lftlj[6]['rotmat'] = np.dot(lftlj[5]['rotmat'], rm.rodrigues(lftlj[6]['rotax'], lftlj[6]['rotangle']))
        lftlj[6]['linkend'] = np.dot(lftlj[6]['rotmat'], lftlj[6]['linkvec'])+lftlj[6]['linkpos']
        lftlj[6]['rngmin'] = -(163-rngsafemargin)
        lftlj[6]['rngmax'] = +(163-rngsafemargin)

        return lftlj

    def __updatefk(self, armlj):
        """
        Update the structure of hiro's arm links and joints (single)
        Note that this function should not be called explicitly
        It is called automatically by functions like movexxx

        :param armlj: the rgtlj or lftlj robot structure
        :return: null

        author: weiwei
        date: 20160615
        """

        i = 1
        while i!=-1:
            j = armlj[i]['mother']
            armlj[i]['linkpos'] = armlj[j]['linkend']
            if i==1:
                armlj[i]['rotmat'] = np.dot(np.dot(armlj[j]['rotmat'],armlj[i]['inherentR']), rm.rodrigues(armlj[i]['rotax'], armlj[i]['rotangle']))
            else:
                armlj[i]['rotmat'] = np.dot(armlj[j]['rotmat'], rm.rodrigues(armlj[i]['rotax'], armlj[i]['rotangle']))
            armlj[i]['linkend'] = np.squeeze(np.dot(armlj[i]['rotmat'], armlj[i]['linkvec'].reshape((-1,1))).reshape((1,-1)))+armlj[i]['linkpos']
            i = armlj[i]['child']
        return armlj

    def numik(self, objpos, objrot, armid="rgt"):
        return nxtik.numik(self, objpos, objrot, armid)

    def numikr(self, objpos, objrot, armid="rgt"):
        return nxtik.numikr(self, objpos, objrot, armid)

if __name__=="__main__":
    import os
    from direct.actor.Actor import Actor
    from panda3d.core import *
    import pandaplotutils.pandageom as pandageom
    import pandaplotutils.pandactrl as pandactrl

    from manipulation.grip.robotiq85 import rtq85
    from nxtmesh import NxtMesh

    base = pandactrl.World(camp=[1400,1200,0], lookatp=[0,0,0])

    nxtrobot = NxtRobot()
    nxtrobot.goinitpose()
    # nxtrobot.movewaist(0)
    nxtrobot.movearmfk([-1.4,4.1,-92.6,154.5,-87.6,-124.5])
    nxtrobot.movearmfk([20,0,-143.2,0,0,0], armid='lft')

    import nxtik
    objpos = np.array([200,0,300])
    # objrot = np.array([[0,1,0],[1,0,0],[0,0,-1]])
    objrot = np.array([[0,1,0],[0,0,-1],[-1,0,0]])
    # plot goal axis
    # base.pggen.plotAxis(nodepath = base.render, spos = objpos, pandamat4 = pg.cvtMat4(objrot))

    lfthand = rtq85.Rtq85()
    rgthand = rtq85.Rtq85()

    nmg = NxtMesh(lfthand=lfthand, rgthand=rgthand)
    nxtmesh = nmg.genmnp(nxtrobot, togglejntscoord=False)
    # nxtmesh = nmg.gensnp(nxtrobot)
    nxtmesh.reparentTo(base.render)

    # this_dir, this_filename = os.path.split(__file__)
    # objpath = os.path.join(os.path.split(this_dir)[0], "../manipulation/grip", "objects", "tool.stl")
    # objmnp = pg.genObjmnp(objpath, color=Vec4(.7, .7, 0, 1))
    # objmnp.reparentTo(base.render)

    base.pggen.plotAxis(base.render)

    base.run()