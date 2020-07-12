import numpy as np

class data_parser:
    
    def __init__(self, data_file_name, verbose=1):
        data_file = open(data_file_name,"r+")
        data = data_file.readlines()
        data_file.close()
        if verbose == 1:
            print('Total number of data lines read: ' + str(len(data)))
            print('First few lines: ')
            print(data[0:10])
        
        # break data up
        self.break_into_seperate_signals(data)
        self.clean_control_data()
        self.clean_wheel_data()
        self.clean_range_data()
        self.correspond_move_and_sense()
        
        
        
        
    def break_into_seperate_signals(self, data):
        data_clean = [line.split('\n')[0] for line in data if len(line.split('\n')[0]) > 0]
        self._data_control = [line for line in data_clean if 'Control' in line]
        self._data_wheel = [line for line in data_clean if 'LDir' in line]
        self._data_sense = [line for line in data_clean if 'S1' in line]
        
        
    
    def clean_control_data(self):
        control_time = [int(line.split(' : ')[0]) for line in self._data_control]
        control_value = [line.split(' : Control ')[-1] for line in self._data_control]
        # controls come in way more often than movement or sensing data 
        # take away duplicates 
        index = 1
        while index < len(control_time):
            if control_time[index] == control_time[index-1]:
                control_time.pop(index)
                control_value.pop(index)
            else:
                index += 1      
        self._controls = {
            'time': np.array(control_time),
            'values': np.array(control_value)
        }
        
        
        
    def clean_wheel_data(self):
        wheel_time = [int(line.split(' : ')[0]) for line in self._data_wheel]
        wheel_lcnt = [int(line.split(' | ')[1].split(': ')[-1]) for line in self._data_wheel]
        wheel_rcnt = [int(line.split(' | ')[-1].split(': ')[-1]) for line in self._data_wheel]
        # remove sensings where neither wheel turned
#         index = 0
#         while index < len(wheel_time):
#             if wheel_lcnt[index] == 0 and wheel_rcnt[index] == 0:
#                 wheel_lcnt.pop(index)
#                 wheel_rcnt.pop(index)
#                 wheel_time.pop(index)
#             else:
#                 index += 1
        self._wheels = {
            'time': np.array(wheel_time),
            'l_cnt': np.array(wheel_lcnt),
            'r_cnt': np.array(wheel_rcnt)*-1 # flip the sign of the right wheel because it makes sense
        }
        
        
    
    def clean_range_data(self):
        range_time = [int(line.split(' : ')[0]) for line in self._data_sense]
        range_front = [float(line.split(' | ')[0].split(': ')[-1].split(' cm')[0]) for line in self._data_sense]
        range_left = [float(line.split(' | ')[1].split(': ')[-1].split(' cm')[0]) for line in self._data_sense]
        range_back = [float(line.split(' | ')[2].split(': ')[-1].split(' cm')[0]) for line in self._data_sense]
        range_right = [float(line.split(' | ')[3].split(': ')[-1].split(' cm')[0]) for line in self._data_sense]
        # turn 0 valued ranges into np.nan values
        for i in range(len(range_front)):
            if range_front[i] == 0.0:
                range_front[i] = np.nan
            if range_left[i] == 0.0:
                range_left[i] = np.nan
            if range_back[i] == 0.0:
                range_back[i] = np.nan
            if range_right[i] == 0.0:
                range_right[i] = np.nan
        self._ranges = {
            'time': np.array(range_time),
            'front': np.array(range_front),
            'left': np.array(range_left),
            'back': np.array(range_back),
            'right': np.array(range_right)
        }
        
        
        
    def correspond_move_and_sense(self):
        sense_move_vals = {}
        for key in self._ranges.keys():
            sense_move_vals[key] = []
        for i,time_wheel in enumerate(self._wheels['time']):
            sense_time_index = np.argmin(np.abs(self._ranges['time'] - time_wheel))
            for key in self._ranges.keys():
                sense_move_vals[key].append(self._ranges[key][sense_time_index])
        for key in sense_move_vals.keys():
            sense_move_vals[key] = np.array(sense_move_vals[key])
        self._sense_move_vals = sense_move_vals
    
    
    def get_sense_data(self):
        return self._sense_move_vals.copy()
    
    def get_move_data(self):
        return self._wheels.copy()