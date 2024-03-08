class super:
    def __init__(self):
        # print("super init")
        self.name = "super"

class Device(super):
    def print (self):
        print("Device")
        print(super().name)
        print(self.name)

if __name__ == "__main__":
    d = Device()
    d.print()
    print(d.name)
    # print(super().name)
    # print(Device().name)