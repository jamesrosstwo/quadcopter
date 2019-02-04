from hmc5883l import hmc5883l
import time


class Gyroscope:
    def __init__(pin):
        self.hmc = hmc5883l(pin)
        self.hmc.setContinuousMode()
        self.setDeclination(-10.51, 0)
        self.rotation = {}

    def read():
        axes = self.hmc.getAxes()
        self.rotation = {"x": axes[0], "y": axes[1], "z": axes[2]}
        return self.rotation


if __name__ == "__main__":
    gyro = hmc5883l(1)
    gyro.setContinuousMode()
    gyro.setDeclination(-10.51,0)
