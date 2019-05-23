'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''

from keyframes import leftBellyToStand
from pid import PIDAgent


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE

        # TODO: Add comments

        for n, name in enumerate(keyframes[0]):

            target_joints[name] = 0

            time = perception.game_state.time

            moveidx = next((i for i, value in enumerate(keyframes[1][n]) if value > time), len(keyframes[1][n]))

            if moveidx < len(keyframes[1][n]):
                movetime = keyframes[1][n][moveidx - 1]

                it = (time if moveidx == 0 else time - movetime) / (
                    keyframes[1][n][moveidx] if moveidx == 0 else keyframes[1][n][moveidx] - movetime)

                p0 = (0.0 if moveidx == 0 else keyframes[2][n][moveidx - 1][0])

                p1 = (0.0 if moveidx == 0 else keyframes[2][n][moveidx - 1][0] + keyframes[2][n][moveidx - 1][2][2])

                p2 = keyframes[2][n][moveidx][0] + keyframes[2][n][moveidx][1][2]

                p3 = keyframes[2][n][moveidx][0]

                target_joints[name] = ((1 - it) ** 3) * p0 + 3 * ((1 - it) ** 2) * it * p1 + 3 * (1 - it) * (
                        it ** 2) * p2 + (it ** 3) * p3

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = leftBellyToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
