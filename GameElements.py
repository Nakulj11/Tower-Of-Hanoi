class Disk(object):

    def __init__(self, name, lowHsv, highHsv):
        self.name = name
        self.lowHsv = lowHsv
        self.highHsv = highHsv

class Tower(object):
    def __init__(self, name, dictionary, id):
        self.name = name
        self.dictionary = dictionary
        self.id = id
