#!/usr/bin/python3

class brobot_hand_logic():
    def __init__(self):
        self.hand_store = [None, None, None]  
        self.action_index= 0
        self.hand_action = 0

    def add_hand_data(self, hand_type)->int:
        self.hand_store[self.action_index] = hand_type
        self.action_index += 1
        if (self.action_index >= 3):
            self.action_index = 0

        if all(element == self.hand_store[0] for element in self.hand_store):
            self.hand_action = self.hand_store[0]
        else:
            self.hand_action = 0
        return self.hand_action
            
    def do_something():
        pass




