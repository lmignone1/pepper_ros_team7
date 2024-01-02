import json, os, sys
import logging
from rasa_sdk import Tracker

FILE_NAME = "database.json"
TRANSFORM = {'upperColour': 'upper_color', 'lowerColour': 'lower_color', 'kindOfPeople': 'gender', 'hatSlot': 'hat', 'bagSlot': 'bag'}
SHOP_DICT = {'walmart': 'roi1', 'starbucks': 'roi2'}
COLOURS = ['black', 'blue', 'brown', 'gray', 'green', 'orange', 'pink', 'purple', 'red', 'white', 'yellow']

class ReaderJson():

    def __init__(self):
        path = os.path.join(os.path.dirname(__file__), "../", FILE_NAME)

        try:
            with open(path, "r") as f:
                file = json.load(f)
                self._people = file["people"]
                logging.info('File loaded')
        except:
            logging.error('File not found')
            sys.exit(1)
        
    
    def search_on_database(self, constraints : dict):

        if len(constraints) != 0:
            # upperColour -> list
            # lowerColour -> list
            # duration -> tuple(seconds, comparative)
            # kindOfPeople -> string
            # hatSlot -> bool
            # bagSlot -> bool
            # place ->  walmart or starbucks

            constraints, comparative, mall = self._transformation(constraints)

            logging.debug(f'CONSTRAINTS IN JSON:\t{constraints}')

            # upper_color -> list
            # lower_color -> list
            # *_persistence_time -> seconds or -1 (just exist)
            # *_passages ->  None
            # gender -> string
            # hat -> bool
            # bag -> bool
            
            search_dict = dict()
            for k, v in constraints.items():
                if v is None:   # *_passages
                    continue

                if k not in search_dict.keys():
                    search_dict[k] = set()
                
                
                if isinstance(v, list):
                    s : set = search_dict[k]

                    for p in self._people:
                        if p[k] in v:
                            s.add(p['id'])
                
                if isinstance(v, str) or isinstance(v, bool):
                    s : set = search_dict[k]

                    for p in self._people:
                        if p[k] == v:
                            s.add(p['id'])
                    continue

                if isinstance(v, int):
                    s : set = search_dict[k]

                    if not mall:
                        
                        val = 0 if v == -1 else v
                
                        for p in self._people:
                            if self._check(comparative, val, p[k]):
                                s.add(p['id'])
                    else:
                        
                        for p in self._people:
                            if self._check(comparative, v, p['roi1_persistence_time'], p['roi2_persistence_time']):
                                s.add(p['id'])
            
            try:
                intersection = set.intersection(*search_dict.values()) # passa i values come elementi separati
                logging.debug(f'INTERSECTION KEYS:\t{search_dict.keys()}')
                logging.info('Intersection done in json')
            except TypeError:
                intersection = set()    # no intersection
                logging.error('No intersection in json')

            logging.debug(f'Intersection people:\t{intersection}')
            return len(intersection), [p for p in self._people if p['id'] in intersection] 
        else:
            logging.info('No constraints in json')
            return len(self._people), self._people

        
    def _check(self, comparative, const, value1, value2 = None):
        if value2 is None:
            return comparative(value1, const)
        else:
            value = value1 + value2
            return comparative(value, const)


    def _comparative_translation(self, comparative = None):
        if comparative is not None:

            if comparative == 'equal':
                comparative = lambda a, b: a == b
            elif comparative == 'less':
                comparative = lambda a, b: a <= b
            else: 
                comparative = lambda a, b: a >= b
        
        else:
            comparative = lambda a, b: a != b
        
        return comparative


    def _transformation(self, constraints : dict):
        t_constraints = dict()

        for key in constraints.keys():
            if key in TRANSFORM.keys():
                t_constraints[TRANSFORM[key]] = constraints[key]
        
        comparative = None
        mall = False

        if 'place' in constraints.keys() and 'duration' not in constraints.keys():  # just place exist
            
            shop = SHOP_DICT[constraints['place']]
            t_constraints[shop + '_persistence_time'] = -1
            t_constraints[shop + '_passages'] = None
            comparative = self._comparative_translation()

        elif 'place' in constraints.keys() and 'duration' in constraints.keys():    # place e time here

            shop = SHOP_DICT[constraints['place']]
            t_constraints[shop + '_persistence_time'] = constraints['duration'][0]
            t_constraints[shop + '_passages'] = None

            comparative = self._comparative_translation(constraints['duration'][1])
            
        
        elif 'place' not in constraints.keys() and 'duration' in constraints.keys(): # time in mall = (roi1 + roi2)
            
            t_constraints['roi1_persistence_time'] = constraints['duration'][0]
            t_constraints['roi1_passages'] = None

            # for shop in shop_dict.values():
            #     json_dict[shop + '_persistence_time'] = constraints['duration'][0]
            #     json_dict[shop + '_passages'] = None
            
            comparative = self._comparative_translation(constraints['duration'][1])
            mall = True

        # else -> no constraints  
        logging.info('Transformed constraints done')

        return t_constraints, comparative, mall

    def make_constraints(self, tracker: Tracker):
        constraints = dict()
        
        # upperColour -> list
        # lowerColour -> list
        # duration -> tuple(seconds, comparative)
        # kindOfPeople -> string
        # hatSlot -> bool
        # bagSlot -> bool
        # place -> walmart or starbucks 

        attributes = ("upperColour", "lowerColour", "duration")
        conditions = ("not_upperSlot", "not_lowerSlot", "compareSlot")

        for a, c in zip(attributes, conditions):
            attr = tracker.get_slot(a)

            if attr is not None and a == 'duration':
                constraints[a] = (attr, tracker.get_slot(c))
                continue
            
            if attr is not None and attr != "free":
                cond = tracker.get_slot(c)
                
                if cond == True:
                    colours = COLOURS.copy()
                    colours.pop(colours.index(attr))
                    constraints[a] = colours
                else:
                    constraints[a] = [attr]

        attributes = ('kindOfPeople', 'hatSlot', 'bagSlot', 'place')      

        for a in attributes:
            attr = tracker.get_slot(a)

            if attr is not None:

                if a == 'kindOfPeople' and attr != 'A':

                    constraints[a] = 'male' if attr == 'M' else 'female'

                elif a == 'place' and attr != 'mall':
                    
                    constraints[a] = attr 

                elif (a == 'hatSlot' or a == 'bagSlot') and attr != 'both':

                    constraints[a] = True if attr == 'with' else False  
        
        logging.debug(f'CONSTRAINTS CLASS ACTION:\t{constraints}')
        logging.info('Constraints done')

        return constraints













