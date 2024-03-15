#!/usr/bin/env python3

# -*- coding: utf-8 -*-
import sys, traceback
import csv
import numpy as np
import rospy

class Imu:
    def __init__(self, name):
        self.name = name
        self.q_indexes = [-1,-1,-1,-1]

    def __repr__(self):
        return self.__str__()
    def __str__(self):
        return f"{self.name}, ({self.q_indexes})"

class Reader:
    def __init__(self, FILENAME, period = 0.01, repeat = True, artificial_time = True):
        rospy.init_node("sto_dumper")
        self.rate = rospy.Rate(1/ period) # in seconds
        self.FILENAME= FILENAME
        self.repeat = repeat
        self.labels = None
        self.artificial_time = artificial_time
        self.t0 = rospy.Time().now().to_sec()
        self.t = self.t0
        self.period = period
        self.imu_list = []
        self.imus = []

    def set_imu_names(self):
        for label in self.labels:
            label_components = label.split("_")
            label_prefix = label_components[0]
            label_restffix= label_components[1:]
            if "q1" in label_restffix:
                ##idk how to do it better
                self.imu_list.append(Imu(label.split("_q1")[0]))
        print(self.imu_list)

    def gen_capture_lists(self):
        for imu in self.imu_list:
            for i, label in enumerate(self.labels):
                if imu.name in label and "_q" in label:
                    if "_q1" in label:
                        imu.q_indexes[0] = i
                    if "_q2" in label:
                        imu.q_indexes[1] = i
                    if "_q3" in label:
                        imu.q_indexes[2] = i
                    if "_q4" in label:
                        imu.q_indexes[3] = i

    def get_qs(self,line):
        pass

    def gen(self):
        with open(self.FILENAME) as stofile:
            line = csv.reader(stofile,delimiter="\t")
            next(line) ## trying to skip the header
            next(line) ## trying to skip the header
            next(line) ## trying to skip the header
            next(line) ## trying to skip the header
            
            self.labels = next(line) ## this line has the actual labels, if you want them
            self.set_imu_names()
            self.gen_capture_lists()
            for imu in self.imu_list:
                print(imu)

            exit()
            while not rospy.is_shutdown():

                for a in line:
                    #print("a:%s"%a)
                    if rospy.is_shutdown():
                        break
                    if self.repeat:
                        ## need to use actual time, or it will break when i loop
                        if self.artificial_time:
                            self.t += self.period 
                        else:
                            self.t = rospy.Time().now().to_sec()
                        a[0]=str(self.t)
                    # like use as a generator?
                    yield ["{:+.5f}".format(float(i)) for i in a]
                if self.repeat:
                    stofile.seek(5)
                else:
                    break

    #bytesToSend         = str.encode(msgFromClient)
    def loopsend(self):
        #try:
            for i,msg in enumerate(self.gen()):
                ## sends tfs
                print(msg)
                if rospy.is_shutdown():
                    break
                self.rate.sleep()
            rospy.loginfo("finished!")
        #except:
        #    traceback.print_exc(file=sys.stdout)



if __name__ == "__main__":
    try:
        A = Reader("test.sto")
        A.loopsend()
    except rospy.ROSInterruptException:
        pass
