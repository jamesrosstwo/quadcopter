from hmc5883l import hmc5883l
import time


class Gyroscope:
    def __init__(self):
        self.hmc = hmc5883l(1)
        self.hmc.setContinuousMode()
        self.hmc.setDeclination(-10.51, 0)
        self.rotation_x = 0
        self.rotation_y = 0
        self.rotation_z = 0

    def read(self):
        axes = self.hmc.getAxes()
        self.rotation_x = axes[0]
        self.rotation_y = axes[1]
        self.rotation_z = axes[2]
        return self.rotation_x, self.rotation_y, self.rotation_z

if __name__ == "__main__":
    gyro = Gyroscope()
    while True:
        print(gyro.read())
        time.sleep(0.5)
