from math import pi

class Circle:
    def __init__(self, radius):
        self.__radius = radius #radius is a private attribute
    
    @property
    def radius(self):
        return self.__radius
    
    @radius.setter
    def radius(self, radius):
        if(radius > 0):
            self.__radius = radius
        else:
            print("Invalid radius")
    
    @property
    def area(self):
        return pi*self.__radius*self.__radius
    
    def circumference(self):
        return 2*pi*self.__radius

circle1 = Circle(5)
# print(circle1.__radius) such printing will throw an error as that attribute can only be accessed inside the class
print(circle1.radius)
print(circle1.area)
print(circle1.circumference())

circle1.radius = -2
circle1.radius = 10

# print(circle1.__radius) will also throw an error
print(circle1.radius)
print(circle1.area)
print(circle1.circumference())