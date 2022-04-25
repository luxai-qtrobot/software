#!/usr/bin/env python
import concurrent.futures
import asyncio
import time
import rospy
import random

from qt_robot_interface.srv import *
from qt_vosk_app.srv import *

german_words = [
["GERFND1_0001", "hallo"],
["GERFND1_0051", "hi"],
["GERFND1_0003", "danke"],
["GERFND1_0006", "entschuldigung"],
["GERFND1_0004", "bitte"],
["GERFND1_0044", "prost"],
["GERFND1_0062", "deutschland"],
["GERFND1_0078", "deutsch"],
["GERFND1_0023", "ja"],
["GERFND1_0024", "nein"],
["GERFND1_0026", "eins"],
["GERFND1_0027", "zwei"],
["GERFND1_0746", "drei"],
["GERFND1_0028", "dies"],
["GERFND1_0029", "das"],
["GERFND1_0088", "schön"],
["GERFND1_0031", "lecker"],
["GERFND1_1043", "was"],
["GERFND1_1044", "wann"],
["GERFND1_1112", "willkommen"],
["GERFND1_0050", "tschüss"],
["GERFND1_0034", "hier"],
["GERFND1_0035", "dort"],
["GERFND1_0148", "interessant"],
["GERFND1_0177", "möglich"],
["GERFND1_0105", "richtig"],
["GERFND1_0216", "wunderbar"],
["GERFND1_0123", "berühmt"],
["GERFND1_1059", "großartig"],
["GERFND1_0093", "beschäftigt"],
["GERFND1_0184", "fertig"],
["GERFND1_0032", "später"],
["GERFND1_0033", "jetzt"],
["GERFND1_0726", "ich"],
["GERFND1_0727", "du"],
["GERFND1_0907", "nichts"],
["GERFND1_0902", "mehr"],
["GERFND1_0901", "vielleicht"],
["GERFND1_0163", "natürlich"],
["GERFND1_0038", "hilfe"],
["GERFND1_1152", "polizei"],
["GERFND1_0784", "wohin"],
["GERFND1_0370", "geldautomat"],
["GERFND1_0806", "heute"],
["GERFND1_0780", "links"],
["GERFND1_0781", "rechts"],
["GERFND1_0509", "reservierung"],
["GERFND1_0099", "geschlossen"],
["GERFND1_0429", "fußball"],
["GERFND1_0940", "bier"],
["GERFND1_0030", "wasser"],
["GERFND1_0367", "ankunft"],
["GERFND1_0138", "glücklich"],
["GERFND1_0354", "straße"],
["GERFND1_0145", "wichtig"]]

german_phrases = [
["GERFND1_0001", "hallo"],
["GERFND1_0051", "hi"],
["GERFND1_0003", "danke"],
["GERFND1_0006", "entschuldigung"],
["GERFND1_0004", "bitte"],
["GERFND1_0044", "prost"],
["GERFND1_0002", "auf wiedersehen"],
["GERFND1_0009", "wie viel kostet es?"],
["GERFND1_0047", "die rechnung bitte"],
["GERFND1_0008", "wo ist das badezimmer?"],
["GERFND1_0062", "deutschland"],
["GERFND1_0078", "deutsch"],
["GERFND1_0023", "ja"],
["GERFND1_0024", "nein"],
["GERFND1_0026", "eins"],
["GERFND1_0027", "zwei"],
["GERFND1_0746", "drei"],
["GERFND1_0028", "dies"],
["GERFND1_0029", "das"],
["GERFND1_0213", "sehr gut"],
["GERFND1_0088", "schön"],
["GERFND1_0031", "lecker"],
["GERFND1_0039", "ich mag es"],
["GERFND1_0041", "das ist gut"],
["GERFND1_0005", "entschuldigen sie"],
["GERFND1_0007", "wo ist?"],
["GERFND1_1043", "was?"],
["GERFND1_1044", "wann?"],
["GERFND1_0871", "ein bisschen"],
["GERFND1_1112", "willkommen"],
["GERFND1_0011", "guten morgen"],
["GERFND1_0014", "gute nacht"],
["GERFND1_0050", "tschüss"],
["GERFND1_0054", "bis später"],
["GERFND1_1108", "vielen dank"],
["GERFND1_0037", "kein problem"],
["GERFND1_1083", "ich liebe dich"],
["GERFND1_1066", "glückwunsch"],
["GERFND1_0042", "was empfehlen sie?"],
["GERFND1_0045", "kann ich ein foto machen?"],
["GERFND1_1131", "was ist das?"],
["GERFND1_0048", "woher kommen sie?"],
["GERFND1_0049", "ich bin aus"],
["GERFND1_0015", "wie gehts?"],
["GERFND1_0052", "wie geht es ihnen?"],
["GERFND1_0016", "mir geht es gut"],
["GERFND1_0056", "wie heißen sie?"],
["GERFND1_0057", "mein name ist"],
["GERFND1_0017", "schön, sie zu treffen"],
["GERFND1_0034", "hier"],
["GERFND1_0035", "dort"],
["GERFND1_0148", "interessant"],
["GERFND1_0177", "möglich"],
["GERFND1_0105", "richtig"],
["GERFND1_0216", "wunderbar"],
["GERFND1_0127", "der lieblings"],
["GERFND1_0123", "berühmt"],
["GERFND1_1059", "großartig"],
["GERFND1_0093", "beschäftigt"],
["GERFND1_0184", "fertig"],
["GERFND1_0032", "später"],
["GERFND1_0033", "jetzt"],
["GERFND1_0726", "ich"],
["GERFND1_0727", "du"],
["GERFND1_0907", "nichts"],
["GERFND1_0902", "mehr"],
["GERFND1_0901", "vielleicht"],
["GERFND1_0163", "natürlich"],
["GERFND1_0018", "ich verstehe"],
["GERFND1_0019", "ich verstehe nicht"],
["GERFND1_0021", "wiederholen sie bitte"],
["GERFND1_0010", "sprechen sie englisch?"],
["GERFND1_1077", "ich spreche ein bisschen deutsch"],
["GERFND1_1073", "wie sagt man. in deutsch?"],
["GERFND1_0022", "was bedeutet?"],
["GERFND1_0046", "gibt es hier einen internetanschluss?"],
["GERFND1_1132", "können sie ein gutes restaurant in der nähe empfehlen?"],
["GERFND1_0038", "hilfe"],
["GERFND1_1152", "polizei"],
["GERFND1_0799", "ich habe mich verlaufen"],
["GERFND1_0784", "wohin?"],
["GERFND1_0370", "geldautomat"],
["GERFND1_0806", "heute"],
["GERFND1_0808", "heute abend"],
["GERFND1_0780", "links"],
["GERFND1_0781", "rechts"],
["GERFND1_0509", "reservierung"],
["GERFND1_0099", "geschlossen"],
["GERFND1_0429", "fußball"],
["GERFND1_0940", "bier"],
["GERFND1_0030", "wasser"],
["GERFND1_0367", "ankunft"],
["GERFND1_0138", "glücklich"],
["GERFND1_0354", "straße"],
["GERFND1_1172", "gibt es hier?"],
["GERFND1_1177", "bekomme ich einen rabatt?"],
["GERFND1_0145", "wichtig"],
["GERFND1_1049", "wie viele?"],
["GERFND1_1282", "fischers fritze fischt frische fische, frische fische fischt fischers fritze"] ]


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
    talk = rospy.ServiceProxy('/qt_robot/behavior/talkAudio', behavior_talk_audio)
    recognize = rospy.ServiceProxy('/qt_robot/speech/recognize', speech_recognize)

    # block/wait for ros service
    rospy.wait_for_service('/qt_robot/speech/say')
    rospy.wait_for_service('/qt_robot/behavior/talkText')


    options = []
    for w_pair in german_words:
        options.append(w_pair[1])

    bs = Synchronizer()
    correct_count = 0
    error_count = 0
    # preload german model
    recognize("de_DE", [], 2)
    try:
        for w_pair in german_words:            
            audio = w_pair[0]
            w = w_pair[1]
            print(f"{error_count+correct_count}: {w}")

            results = bs.sync([
                (0, lambda: recognize("de_DE", options, 10)),
                (1, lambda: talk("german/" + audio, ""))
            ])

            try:
                transcript = results[0].transcript
            except:
                transcript = results[1].transcript

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
