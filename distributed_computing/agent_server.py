'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

import SimpleXMLRPCServer
# add PYTHONPATH
import os
import sys

sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent
import threading


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE

        return self.perception.joint[joint_name]
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE

        self.target_joints[joint_name] = angle


    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE

        print(self.posture)
        self.recognize_posture(self.perception)
        return self.posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE

        self.keyframes = keyframes

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE

        self.forward_kinematics(self.perception.joint)
        return self.transforms[name]

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE

        self.set_transforms(self, effector_name, transform)

if __name__ == '__main__':
    # Only works after learning the postures with the ipython notebook

    agent = ServerAgent()

    agent_server = SimpleXMLRPCServer.SimpleXMLRPCServer(('localhost', 9000))

    agent_server.register_function(agent.get_angle, 'get_angle')
    agent_server.register_function(agent.set_angle, 'set_angle')
    agent_server.register_function(agent.get_posture, 'get_posture')
    agent_server.register_function(agent.execute_keyframes, 'execute_keyframes')
    agent_server.register_function(agent.get_transform, 'get_transform')
    agent_server.register_function(agent.set_transform, 'set_transform')

    agent_server_thread = threading.Thread(target=agent_server.serve_forever())
    agent_server_thread.start()

    agent.run()
