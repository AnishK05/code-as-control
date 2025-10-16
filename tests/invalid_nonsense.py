import os
import sys

def run(group):
    # This has forbidden operations
    print("Hello world!")  # I/O not allowed
    os.system("ls")  # System calls not allowed
    eval("1+1")  # eval not allowed
    
    with open("file.txt", "w") as f:  # File I/O not allowed
        f.write("test")

