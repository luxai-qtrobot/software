#!/usr/bin/env python
import concurrent.futures
import asyncio
import time
import rospy
import random

from qt_robot_interface.srv import *
from qt_gspeech_app.srv import *


class Synchronizer():
    """
    A simple concurrent tasks synchornizer
    """

    def __init__(self, max_workers=5):
        self.loop = asyncio.get_event_loop()
        self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=max_workers)


    def __worker(self, *args):
        delay_exe = args[0][0] 
        func = args[0][1]
        time.sleep(delay_exe)
        return func()

    async def __non_blocking(self, tasks):
        fs = []
        for task in tasks:
            fs.append(self.loop.run_in_executor(self.executor, self.__worker, task))        
        done, pending = await asyncio.wait(fs=fs, return_when=asyncio.ALL_COMPLETED)        
        results = [task.result() for task in done]        
        return results 


    def sync(self, tasks):
        """
        call this function with multiple tasks to run concurrently.
        tasks is a list of (delay, lamda function) tuple. for exmaple: 
        tasks = [ (0, lambda: print("hello")), (3, lambda: print("world")), ...] 

        returns a list of each lamda function return value   
        """
        results = self.loop.run_until_complete(self.__non_blocking(tasks))        
        return results



if __name__ == '__main__':

    rospy.init_node('test_vosk')
    rospy.loginfo("test_vosk started!")

    # define a ros service
    talk = rospy.ServiceProxy('/qt_robot/behavior/talkText', behavior_talk_text)
    recognize = rospy.ServiceProxy('/qt_robot/speech/recognize', speech_recognize)

    # block/wait for ros service
    rospy.wait_for_service('/qt_robot/speech/say')
    rospy.wait_for_service('/qt_robot/behavior/talkText')


    numbers = ["zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine", "ten", "eleven", "twelve", "thirteen", "fourteen", "fifteen", "sixteen", "seventeen", "eighteen", "nineteen", "twenty", "twenty one", "twenty two", "twenty three", "twenty four", "twenty five", "twenty six", "twenty seven", "twenty eight", "twenty nine", "thirty", "thirty one", "thirty two", "thirty three", "thirty four", "thirty five", "thirty six", "thirty seven", "thirty eight", "thirty nine", "forty", "forty one", "forty two", "forty three", "forty four", "forty five", "forty six", "forty seven", "forty eight", "forty nine", "fifty", "fifty one", "fifty two", "fifty three", "fifty four", "fifty five", "fifty six", "fifty seven", "fifty eight", "fifty nine", "sixty", "sixty one", "sixty two", "sixty three", "sixty four", "sixty five", "sixty six", "sixty seven", "sixty eight", "sixty nine", "seventy", "seventy one", "seventy two", "seventy three", "seventy four", "seventy five", "seventy six", "seventy seven", "seventy eight", "seventy nine", "eighty", "eighty one", "eighty two", "eighty three", "eighty four", "eighty five", "eighty six", "eighty seven", "eighty eight", "eighty nine", "ninety", "ninety one", "ninety two", "ninety three", "ninety four", "ninety five", "ninety six", "ninety seven", "ninety eight", "ninety nine", "hundred"]
    words = ["claim", "accurate", "mend", "fire", "greet", "cook", "buzz", "soap", "quizzical", "support", "exist", "degree", "general", "cherries", "concentrate", "soda", "probable", "rifle", "examine", "faded", "blush", "even", "legs", "spark", "yummy", "glass", "smart", "growth", "nice", "receipt", "stereotyped", "lace", "treatment", "wandering", "mother", "dusty", "good", "rest", "grouchy", "error", "hammer", "advice", "unique", "room", "knee", "flippant", "blood", "shivering", "wiggly", "tiny", "lean", "airport", "lunch", "identify", "thunder", "tooth", "deceive", "seed", "sick", "end", "fine", "telephone", "group", "left", "bushes", "jealous", "taboo", "gullible", "tremble", "safe", "milky", "cheap", "owe", "immense", "plough", "woman", "decide", "start", "ugly", "maddening", "matter", "flavor", "skin", "nine", "retire", "cattle", "earthquake", "vigorous", "jewel", "pretty"]
    digits = ["11", "12", "13", "14", "15", "16", "17", "18", "19", "20", "21", "22", "23", "24", "25", "26", "27", "28", "29", "30", "31", "32", "33", "34", "35", "36", "37", "38", "39", "40", "41", "42", "43", "44", "45", "46", "47", "48", "49", "50", "51", "52", "53", "54", "55", "56", "57", "58", "59", "60", "61", "62", "63", "64", "65", "66", "67", "68", "69", "70", "71", "72", "73", "74", "75", "76", "77", "78", "79", "80", "81", "82", "83", "84", "85", "86", "87", "88", "89", "90", "91", "92", "93", "94", "95", "96", "97", "98", "99"]

    #talk("Please repeat the words after me")

    bs = Synchronizer()
    correct_count = 0
    error_count = 0
    vocabs = words
    try:
        for w in vocabs:
            # w = random.choice(words[0:20])
            w = w.lower()
            print(f"{error_count+correct_count}: {w}")

            results = bs.sync([
                (0, lambda: recognize("en-US", vocabs, 10)),
                (2, lambda: talk(w))
            ])

            try:
                transcript = results[0].transcript
            except:
                transcript = results[1].transcript

            transcript = transcript.lower()
            print(f"got: {transcript}")
            if transcript.replace(".", "") == w:
                correct_count = correct_count + 1
            else:
                error_count = error_count + 1               
    except:
        pass

    #rospy.loginfo("finsihed!")
    print("")
    print(f"Total:   {error_count + correct_count}")
    print(f"Correct: {correct_count}")
    print(f"Wrong:   {error_count}")
    print("WER:      %.2f " % (error_count/(error_count + correct_count)*100))
