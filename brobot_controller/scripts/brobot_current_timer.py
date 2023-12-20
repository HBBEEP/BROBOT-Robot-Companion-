#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from brobot_interfaces.srv import CurrentTimeService
import datetime



class brobot_current_timer(Node):
    def __init__(self):
        super().__init__('current_time_service')
        self.timer_service = self.create_service( CurrentTimeService, '/current_time_service', self.current_time)

    def get_current_time(self)->str:
        current_time = datetime.datetime.now()
        formatted_time = current_time.strftime("the current time is %I o'clock %M minute")
        formatted_time_with_words = ' '.join(self.number_to_words(word) if word.isdigit() else word for word in formatted_time.split())
        return formatted_time_with_words

    def number_to_words(self, number)->str:
        words_mapping = {
            '0': '',
            '1': 'one',
            '2': 'two',
            '3': 'three',
            '4': 'four',
            '5': 'five',
            '6': 'six',
            '7': 'seven',
            '8': 'eight',
            '9': 'nine',
            '10': 'ten',
            '11': 'eleven',
            '12': 'twelve',
            '13': 'thirteen',
            '14': 'fourteen',
            '15': 'fifteen',
            '16': 'sixteen',
            '17': 'seventeen',
            '18': 'eighteen',
            '19': 'nineteen',
            '20': 'twenty',
            '21': 'twenty-one',
            '22': 'twenty-two',
            '23': 'twenty-three',
            '24': 'twenty-four'
        }


        return ' '.join(words_mapping[digit] for digit in str(number))
    
    def current_time(self, request, response):
        response = CurrentTimeService.Response()
        response.message = str(self.get_current_time())
        return response


def main(args=None):
    rclpy.init(args=args)
    node = brobot_current_timer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
