#!/usr/bin/env python


class avg:

    def __init__(self,_MESS):
        self.MESS = _MESS
        self.avg_counter = -1
        self.avg_sum = 0.0
        self.avg_direction_angle = 0.0
        self.avg_ok = False

        self.avg_values = []

        for i in range(self.MESS):
            self.avg_values[i] = 0.0


    def new_value(self,value):

        if self.avg_counter < self.MESS:
            self.avg_counter+=1
            self.avg_values[self.avg_counter] = value
            self.avg_sum += value
        
        else:
            self.avg_sum +=self.avg_values[0]

            new_values = []

            for i in range(self.MESS):
                new_values[i] = 0

            for j in range(self.MESS):
                new_values[i-1]=self.avg_values[i]

            new_values[self.MESS-1]=value

            for k in range(self.MESS):
                self.avg_values[i] = new_values[i]
            
            self.avg_sum+=value

    def avg(self):
        if(self.avg_counter != -1):
            return self.avg_sum/(self.avg_counter+1)
        else:
            return 0

    def flush_array(self):
        self.avg_counter = -1
        self.avg_sum = 0.0
