#another example of classes, with implementation of encapsulation to create public, protected and private attributes
class Employee:
    bonus_amount = 0.1

    def __init__(self, name, employee_id, salary):
        self.__name = name #private attribute, starting with double _, can only be accessed by the base class
        self.__salary = salary
        self._emp_id = employee_id #protected attribute, starting with single _, can be accessed by a derived class
    
    def get_name(self):
        print(self.__name)

    def get_employee_id(self):
        print(self._emp_id)
    
    def get_salary(self):
        print(self.__salary)
    
    def set_salary(self, new_salary):
        if new_salary >= self.__salary:
            self.__salary = new_salary
        else:
            print("Invalid assignment of salary!")
    
    def calculate_bonus(self):
        print("Bonus is", self.__salary*self.bonus_amount)
    
class Manager(Employee):
    def __init__(self, name, employee_id, salary, department):
        super().__init__(name, employee_id, salary)
        self._department = department
    
    def get_department(self):
        print(self._department)
    
    def set_department(self, new_department):
        self._department = new_department
        print("Reassignment successful!")

class Director(Manager):
    def __init__(self, name, employee_id, salary, department, budget):
        super().__init__(name, employee_id, salary, department)
        self.__budget = budget
    
    def get_budget(self):
        print(self.__budget)
    
    def set_budget(self, new_budget):
        if(new_budget > 0):
            self.__budget = new_budget
    
    def show_use_of_private(self):
        print(self.__name)
    
    def show_use_of_protected(self):
        print(self._emp_id)

emp1 = Manager("Sanjeev", 1234, 5000000, "EE")
emp1.set_salary(90000)
emp1.get_name()
emp1.calculate_bonus()

emp2 = Director("Subrahmaniyan", 12345, 6789023, "EE", 1234313123)
emp2.get_name()
# emp2.show_use_of_private() calling this will throw an error because it cannot be accessed
emp2.show_use_of_protected() #does not throw error because it can access the variable, being a derived class