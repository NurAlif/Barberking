class Vector2:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Vector2yaw:
    def __init__(self, x=0.0, y=0.000, yaw=0.000):
        self.x = x
        self.y = y
        self.yaw = yaw
    
    @staticmethod
    def multiply(a, b):
        return Vector2yaw(a.x * b.x, a.y * b.y, a.yaw * b.yaw)

    @staticmethod
    def add(a, b):
        temp = Vector2yaw(0,0,0)
        temp.x = a.x + b.x
        temp.y = a.y + b.y
        temp.yaw = a.yaw + b.yaw
        return temp
    
    def set(self, target):
        self.x = target.x
        self.y = target.y
        self.yaw = target.yaw

    def stepToTarget(self, target, step):
        selfx = self.x
        selfy = self.y
        selfyaw = self.yaw
        targetx = target.x
        targety = target.y
        targetyaw = target.yaw
        if(selfx < targetx): 
            self.x += step.x
            if selfx > targetx: self.x = targetx
        elif(selfx > targetx): 
            self.x -= step.x
            if selfx < targetx: self.x = targetx

        if(selfy < targety): 
            self.y += step.y
            if selfy > targety: self.y = targety
        elif(selfy > targety): 
            self.y -= step.y
            if selfy < targety: self.y = targety

        if(selfyaw < targetyaw): 
            self.yaw += step.yaw
            if selfyaw > targetyaw: self.yaw = targetyaw
        elif(selfyaw > targetyaw): 
            self.yaw -= step.yaw
            if selfyaw < targetyaw: self.yaw = targetyaw


CONTROL_MODE_HEADLESS = 0
CONTROL_MODE_YAWMODE = 1

class Walking:
    def __init__(self):
        global CONTROL_MODE_YAWMODE
        global CONTROL_MODE_HEADLESS
        self.control = None # held controller socket id
        self.turn_mode = CONTROL_MODE_YAWMODE
        self.max_speed = 40
        self.stationary_offset = Vector2yaw()
        self.feed_rate = 10
        self.step = Vector2yaw(0.001,0.001,0.001)
        self.vectorMultiplier = Vector2yaw(0.02, 0.02, 0.02)
        self.vectorCurrent = Vector2yaw()
        self.vectorTarget = Vector2yaw() # normalized

    def setTarget(self, newTarget): # normalized input
        self.vectorTarget = Vector2yaw.add(Vector2yaw.multiply(newTarget, self.vectorMultiplier), self.stationary_offset)

    def stepToTargetVel(self):
        self.vectorCurrent.stepToTarget(self.vectorTarget, self.step)
        # self.vectorCurrent.set(self.vectorTarget)
    
    def setWalkingOffset(self):
        self.stationary_offset.set(self.vectorCurrent)

    def setWalkingConf(self, confDict):
        confName = confDict[0]
        confValue = confDict[1]
        if confName == 'max_speed': self.max_speed = confValue
        elif confName == 'stationary_offset':
            self.stationary_offset.x = confValue[0]
            self.stationary_offset.y = confValue[1]
            self.stationary_offset.yaw = confValue[2]
        elif confName == 'feed_rate': self.feed_rate = confValue
        elif confName == 'step':
            if(confValue[0] == "xy"):
                self.step.x = confValue[1]
                self.step.y = confValue[1]
            elif(confValue[0] == "yaw"):
                self.step.yaw = confValue[1]
        elif confName == 'multipler':
            if(confValue[0] == "xy"):
                self.step.x = confValue[1]
                self.step.y = confValue[1]
            elif(confValue[0] == "yaw"):
                self.step.yaw = confValue[1]
        elif confName == 'turn_mode': self.turn_mode = confValue
        elif confName == 'offset':
            if confValue[0] == "x": self.stationary_offset.x = confValue[1]
            elif confValue[0] == "y": self.stationary_offset.y = confValue[1]
            elif confValue[0] == "yaw": self.stationary_offset.yaw = confValue[1]

    def getWalkingConf(self):
        offset = self.stationary_offset
        multiplier = self.vectorMultiplier
        step = self.step
        confDict = {
            'control': self.control,
            'max_speed': self.max_speed,
            'turn_mode': self.turn_mode,
            'stationary_offset': [offset.x, offset.y, offset.yaw],
            'feed_rate': self.feed_rate,
            'step':[step.x, step.y, step.yaw],
            'multiplier': [multiplier.x, multiplier.y, multiplier.yaw]
        }
        if(self.control == None):
            confDict['control'] = -1
        return confDict
    def getWalkingCurrent(self):
        current = self.vectorCurrent
        array = [current.x, current.y, current.yaw]
        return array