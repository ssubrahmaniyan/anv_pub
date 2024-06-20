#A library management system implementation, illustrating the use of inheritance and polymorphism
class LibraryItem: #base class, the class on which other class are derived from
    def __init__(self, title, author, year):
        self.title = title
        self.author = author
        self.year = year
        self.available = True

    def disp_details(self):#polymorphic function, takes different forms depending on the object with which it is called
        if(isinstance(self, Book)):
            print("Title:", self.title, "Author:", self.author, "Year:", self.year, "Genre:", self.genre)
        elif(isinstance(self, Magazine)):
            print("Title:", self.title, "Author:", self.author, "Year:", self.year, "Issue#", self.issue_number)
        elif(isinstance(self, DVD)):
            print("Title:", self.title, "Author:", self.author, "Year:", self.year, "Director:", self.director, "Duration:", self.duration)
    
    def checkout_item(self):
        if(self.available):
            self.available = False
            print(self.title, "due in 30 days. Thank you!")
        else:
            print("Item unavailable for checkout")
    
    def return_item(self):
            self.available = True
            print("Thank you, please take a look at the other books!")

class Book(LibraryItem): #example of a derived class. These classes have access to all the public and protected attributes and methods of the base class
    def __init__(self, title, author, year, genre):
        super().__init__(title, author, year) #a super() method works by calling the init function of the higher class on which this child class is based on
        self.genre = genre

class Magazine(LibraryItem):
    def __init__(self, title, author, year, issue_number):
        super().__init__(title, author, year)
        self.issue_number = issue_number
    
class DVD(LibraryItem):
    def __init__(self, title, author, year, director, duration):
        super().__init__(title, author, year)
        self.director = director
        self.duration = duration


# Example Usage:
book = Book("The Great Gatsby", "F. Scott Fitzgerald", 1925, "Fiction")
magazine = Magazine("National Geographic", "Various", 2021, 12)
dvd = DVD("Inception", "Christopher Nolan", 2010, "Christopher Nolan", "148 minutes")

book.disp_details() 
magazine.disp_details()  
dvd.disp_details()  

book.checkout_item()
book.checkout_item()
book.return_item()
