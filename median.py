#!/usr/bin/env python3
import statistics

class median():

    def __init__(self,size):
        self.size = size
        self.values = []
        self.counter = 1

    def new_value(self,value):

        if len(self.values) == self.size:
            self.values.pop(len(self.values)-self.size)
            self.new_value(value)        
        else:
            self.values.append(value)
        
        self.counter +=1
    
    def get_median(self):
        return statistics.median(self.values)

    def clear_values(self):
        self.values = []




