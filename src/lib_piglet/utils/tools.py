import sys

def eprint(*args, **kwargs):
    print("[ERROR] ",*args, file=sys.stderr, **kwargs)