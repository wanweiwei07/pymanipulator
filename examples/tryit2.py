import motionplanning.collisioncheckerball as cdck
import pandaplotutils.pandactrl as pandactrl
from manipulation.grip.robotiq85 import rtq85nm
from motionplanning.rrt import ddrrtconnect as ddrrtc
# from motionplanning.rrt import ddrrtconnect_rtree as ddrrtc
from robotsim.nextage import nxt
from robotsim.nextage import nxtmesh
from robotsim.nextage import nxtball

def iscollidedfunc(point, obstaclelist = [], robot = None, cdchecker = None):
    """
    check if a specific configuration is in collision

    :param point:
    :param robot the object defined in robotsim/robot
    :param cdchecker: a collisionchecker object
    :return:

    author: weiwei
    date: 20180109
    """

    robot.movearmfk(point)
    isselfcollided = cdchecker.isCollided(robot, obstaclelist)
    robot.goinitpose()

    if isselfcollided:
        return True
    else:
        return False

if __name__ == '__main__':
    from manipulation.grip.robotiq85 import rtq85
    import motionplanning.smoother as sm

    base = pandactrl.World()

    robot = nxt.NxtRobot()
    rgthnd = rtq85.Rtq85()
    lfthnd = rtq85.Rtq85()
    robotmesh = nxtmesh.NxtMesh(rgthand=rgthnd, lfthand=lfthnd)
    robotball = nxtball.NxtBall()
    cdchecker = cdck.CollisionCheckerBall(robotball)

    start = [50.0,0.0,-143.0,0.0,0.0,0.0]
    goal = [-45.0,0.0,-143.0,0.0,0.0,0.0]
    # plot init and goal
    robot.movearmfk(armjnts = start, armid = 'rgt')
    rmi = robotmesh.genmnp(robot)
    rmi.reparentTo(base.render)
    robot.movearmfk(armjnts = goal, armid = 'rgt')
    rmg = robotmesh.genmnp(robot)
    rmg.reparentTo(base.render)

    jointlimits = [[robot.rgtarm[1]['rngmin'], robot.rgtarm[1]['rngmax']],
                   [robot.rgtarm[2]['rngmin'], robot.rgtarm[2]['rngmax']],
                   [robot.rgtarm[3]['rngmin'], robot.rgtarm[3]['rngmax']],
                   [robot.rgtarm[4]['rngmin'], robot.rgtarm[4]['rngmax']],
                   [robot.rgtarm[5]['rngmin'], robot.rgtarm[5]['rngmax']],
                   [robot.rgtarm[6]['rngmin'], robot.rgtarm[6]['rngmax']]]
    import os
    from panda3d.core import *
    import pandaplotutils.pandageom as pg
    obsrotmat4 = Mat4(1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,400,-300,200,1.0)
    this_dir, this_filename = os.path.split(__file__)
    objpath = os.path.join(os.path.split(this_dir)[0], "manipulation/grip", "objects", "bunnysim.stl")
    objmnp = pg.genObjmnp(objpath, color=Vec4(.7, .7, 0, 1))
    objmnp.setMat(obsrotmat4)
    objmnp.setScale(3.0)
    objmnp.reparentTo(base.render)
    # base.run()
    #
    planner = ddrrtc.DDRRTConnect(start=start, goal=goal, iscollidedfunc = iscollidedfunc,
                              cdchecker = cdchecker, jointlimits = jointlimits, starttreesamplerate=30,
                            expanddis = 10, robot = robot)

    import time
    tic = time.clock()
    [path, sampledpoints] = planner.planning(obstaclelist = [objmnp])
    toc = time.clock()
    print toc-tic
    smoother = sm.Smoother()
    path = smoother.pathsmoothing(path, planner, 200)

    # if you want to show all samples
    sampledpointsindex = [0]
    robotonscreen = [None]
    pathindex = [0]

    def updateshow(path, pathindex, sampledpoints, sampledpointsindex,
                   robot, robotmesh, task):
        if sampledpointsindex[0] <= len(sampledpoints) - 1:
            if robotonscreen[0] is not None:
                robotonscreen[0].removeNode()
            robot.movearmfk(sampledpoints[sampledpointsindex[0]][0], armid='rgt')
            robotonscreen[0] = robotmesh.genmnp(robot)
            robotonscreen[0].reparentTo(base.render)
            robot.goinitpose()
            sampledpointsindex[0] += 1
            return task.again
        else:
            if robotonscreen[0] is not None:
                robotonscreen[0].removeNode()
            if pathindex[0] <= len(path) - 1:
                robot.movearmfk(path[pathindex[0]], armid='rgt')
                robotonscreen[0] = robotmesh.genmnp(robot)
                robotonscreen[0].reparentTo(base.render)
                pathindex[0] += 1
            else:
                pathindex[0] = 0
            time.sleep(.07)
            return task.again

    taskMgr.add(updateshow, "updateshow",
                extraArgs=[path, pathindex, sampledpoints, sampledpointsindex,
                           robot, robotmesh],
                appendTask=True)
    base.run()