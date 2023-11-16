import random
import string
import lorem

class SimSensor():

    def __init__(self):

        self.topicMap = {
        
            'A' : self.generate_float_data,
            'B' : self.generate_word_data
        }

    def generate_float_data(self):
        return ':'.join(map(str, [random.uniform(0.0, 1.0) for _ in range(5)]))

    def generate_word_data(self):
        sentance = lorem.sentence()
        return sentance

    def generate_data_for_topic(self,):
        topics = list(self.topicMap.keys())
        topic_id = random.choice(topics)
        return topic_id + '/' + self.topicMap[topic_id]() #execute function
 

