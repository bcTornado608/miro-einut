from state_machine import *

counter = 0

def func1():
    print('func1')
    global counter
    counter += 1
    if counter < 5:
        return False
    else:
        return True

def func2():
    print('func2')
    return True

def func3():
    print('func3')
    return True


if __name__ == '__main__':
    seq = Sequential([func1, func2, func3])
    while not seq():
        pass