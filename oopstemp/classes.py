#This program outlines the basics of classes in python
class esb: #definition of a class
    n_classroom = 0 #class variable - common to all object instances of the class

    def __init__(self, name, number): #initialisation function, which creates instance variables and assigns values to them
        self.name = name
        self.number = number
        esb.n_classroom += 1

class person:
    def __init__(self, name, dob, country):
        self.name = name
        self.dob = dob
        self.country = country
    def tell_age(self): #instance method, can be used with the instance to retrieve properties of the instance
        print(str(self.name + " was born on " + self.dob))

#Example usage
sanjeev = person("Sanjeev", "26/07/2005", "India")
sanjeev.tell_age()
room1 = esb(number=242, name='hell')
room2 = esb(number=127, name='heaven')

print(room2.name)
print(esb.n_classroom)
