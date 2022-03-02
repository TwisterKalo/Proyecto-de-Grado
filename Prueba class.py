class Robot:
    def introduce_self(self):
        print("Mi nombre es " + self.name)
        
r1 = Robot()
r1.name = "Tom"
r1.introduce_self()