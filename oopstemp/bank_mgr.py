class BankAccount:
    def __init__(self, name, upfront):
        self.__balance = upfront
        self.name = name
    
    @property
    def balance(self):
        return self.__balance
    
    @balance.setter
    def balance(self, amount):
        raise AttributeError("Direct balance setting is not allowed. Use deposit or withdraw methods.")
    
    def deposit(self, amount):
        if amount > 0:
            self.__balance += amount
            print("Deposit successful")
        else:
            print("Deposit amount must be positive")
    
    def withdraw(self, amount):
        if 0 < amount <= self.__balance:
            self.__balance -= amount
            print("Withdrawal successful")
        else:
            print("Insufficient balance for withdrawal or invalid amount")

acc1 = BankAccount("Sanjeev", 10000)
print(acc1.balance)  # Should print 10000
acc2 = BankAccount("Demo", 1000)
print(acc1.balance)  # Should print 10000 (No change in acc1)
acc1.deposit(5000)
print(acc1.balance)  # Should print 15000
acc2.deposit(15000)
print(acc2.balance)  # Should print 16000
acc1.withdraw(20000)  # Should print "Insufficient balance for withdrawal"
print(acc1.balance)  # Should print 15000 (No change due to failed withdrawal)
acc2.withdraw(10000)  # Should print "Withdrawal successful"
print(acc1.balance)  # Should print 15000
print(acc2.balance)  # Should print 6000