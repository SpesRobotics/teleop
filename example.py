from teleop import Teleop


def main():
    def callback(pose, message):
        print(pose)
        print(message)

    teleop = Teleop()
    # teleop.set_pose(np.eye(4))
    teleop.subscribe(callback)
    teleop.run()
