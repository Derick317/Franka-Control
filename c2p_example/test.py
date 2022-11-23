class Test:
    data = 1
    def __init__(self):
        self.data = 2

    def print(self):
        print(Test.data, self.data)

if __name__ == '__main__':
    test = Test()
    test.print()