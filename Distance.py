from networktables import NetworkTables
NetworkTables.initialize(server='roborio-6445-frc.local')
sd = NetworkTables.getTable('SmartDashboard')
# Find Pixels using bounding box
# then use F = (P*D)/W (Or height)
# With the focal point then use D' = (W*F)/P Units are in inches
# Then find offset from center of focal point


class Vision:
    def __init__(self):
        # init vars
        self.a = "Var"
    def process(self, source0):
        self.a = source0
        # just a place holder self.a = source0 will cause a crash
        # Create pipeline

if __name__ == "__main__":
    ip = Vision()
    while (True):
        ip.process(cv2.VideoCapture(0))
        if not sd.isconnected():
            sd = NetworkTables.getTable('SmartDashboard')
            NetworkTables.initialize(server='roborio-6445-frc.local')

