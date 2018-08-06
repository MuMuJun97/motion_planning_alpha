import sys
import os


def terminal(*commands):
    responses = [os.system(command) for command in commands]
    return responses


def run(mode):
    commands = './run.sh'
    return terminal(commands)

def end():
    if input('Close all terminals (y or n)?').lower() == 'y' :
        terminal('pkill gnome-terminal')
        return

if __name__ == '__main__' :
    mode = 'standard'
    if len(sys.argv) > 1 :
        mode = sys.argv[1]
    print(run(mode))
    end()