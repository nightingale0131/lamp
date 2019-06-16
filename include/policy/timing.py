"""
Source:
https://stackoverflow.com/questions/1557571/how-do-i-get-time-of-a-python-programs-execution/12344609#12344609

Written by: Paul McGuire
Slight modifications by: Florence

Timing module to time program runtime
"""

#!/usr/bin/python
import atexit
from time import clock

def secondsToStr(t):
    return "%d:%02d:%02d.%06d" % \
        reduce(lambda ll,b : divmod(ll[0],b) + ll[1:], [(t*1000,),1000,60,60])

line = "="*40
def log(s, elapsed=True):
    print line
    current = clock()
    print secondsToStr(current), '-', s
    if elapsed:
        print "Elapsed time:", secondsToStr(current-start)
    print line
    print

def endlog():
    log("End Program", True)

def now():
    return secondsToStr(clock())

start = clock()
atexit.register(endlog)
log("Start Program")
