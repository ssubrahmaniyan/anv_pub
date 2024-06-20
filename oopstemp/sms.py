#This is to implement a student management system using object oriented programming, using most of the concepts

class student:
    total_students = 0

    def __init__(self, first, last, age):
        self.first = first
        self.last = last
        self.age = age
        self.grades = {}
        student.total_students+=1
    
    def add_grade(self, subject, grade): #instance method, operates on one instance of the class
        self.grades[subject] = grade

    def average_grade(self):
        sum = 0
        cnt = 0
        for key in self.grades.keys():
            sum += self.grades[key]
            cnt += 1
        return sum/cnt
    
    @classmethod
    def from_string(cls, student_str): # class method, generally used when an instance is created or an operation is performed on the class as a whole
        first, last, age = student_str.split(" ")
        return cls(first, last, age)
    
    @staticmethod
    def is_adult(age): #regular methods, only defined inside the class because they have some functional relationship with the class or object of it
        if age >= 18:
            return True
        return False

#example use cases    
stud1 = student("Sanjeev", "Subrahmaniyan", 18)
stud2 = student.from_string("Hello Bello 17")
stud2.add_grade("Physics", 10)
stud2.add_grade("Programming", 7)
stud1.add_grade("Physics", 10)
stud1.add_grade("Programming", 10)

print(student.is_adult(stud1.age))
print(stud1.average_grade())
print(stud2.average_grade())
