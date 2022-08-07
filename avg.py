#!/usr/bin/env python3
class avg:

    def __init__(self,_MESS):
        self.MESS = _MESS
        self.avg_counter = -1
        self.avg_sum = 0.0
        self.avg_direction_angle = 0.0
        self.avg_ok = False

        self.avg_values = []

        for i in range(self.MESS):
            self.avg_values.append(0)


    def new_value(self,value):
        self.avg_sum = 0

        if self.avg_counter+1 < self.MESS:
            self.avg_counter+=1
            self.avg_sum += value
        
        else:
            self.avg_sum +=self.avg_values[0]

            new_values = []
            for i in range(self.MESS):
                new_values.append(0)
            for j in range(1,self.MESS):
                new_values[j-1]=self.avg_values[j]

            new_values[self.MESS-1]=value

            for k in range(self.MESS):
                self.avg_values[k] = new_values[k]
            
            self.avg_sum+=value

    def avg(self):
        if(self.avg_counter != -1):
            return self.avg_sum/(self.avg_counter+1)
        else:
            return 0

    def flush_array(self):
        self.avg_counter = -1
        self.avg_sum = 0.0
