import string
from numpy import argmin
import logging


NOT_REMOVE_PUNCTUATION = ("""'""", '-')
DISTANCE_TH1 = 2
DISTANCE_TH2 = 4
DRESS = ('suit', 'dresses','dress','tracksuit','tailleur','smoking','work overalls', 'uniform', 'tuxedo')
SHOP_DICT = {'roi1' : 'walmart', 'roi2' : 'starbucks'}

def cleaner(stringa : str):
    punct = [char for char in string.punctuation]
    for i in NOT_REMOVE_PUNCTUATION:
        punct.pop(punct.index(i)) 
    table = str.maketrans(dict.fromkeys(punct))
    return stringa.translate(table)

def not_replacement(msg : list, entities : list):
    j = 0
    n = len(entities)   
    k = len(msg)

    for i in range(k):
        if j == n:
            break
        
        if entities[j] == msg[i]:
            j += 1
        elif entities[j] in msg[i]:
            msg[i] = entities[j]
            j += 1
        elif "'" in msg[i]:
            index = msg[i].index("'")
            if entities[j].startswith(msg[i][:index]):
                msg[i] = entities[j]
                j+=1
        elif "'" in entities[j]:
            index = entities[j].index("'")
            if msg[i].startswith(entities[j][:index]):
                msg[i] = entities[j]
                j+=1
        elif entities[j].endswith(' t'):
            if msg[i] == 't': 
                msg[i-1] = msg[i-1] + ' t'
                msg.pop(i)
                j+=1
                k-=1
    return msg

def minimum_distance(msg : list, entities : list, entity : str, mode = 'not_colour'):
    distances = list()

    entity = entity.split()[0]
    logging.debug("entity", entity)

    for i, en in enumerate(msg):    # se ci sono 2 enitita uguali (ad esempio compare 2 volte t-shirt), in questo modo viene preso sempre l ultima entita
        if en == entity:
            entity_index = i
    
    if mode == 'colour':
        
        for col in entities:
            col_index = msg.index(col)
            logging.debug("colour %s",col)
            logging.debug("col_index: %s", col_index)
            d = entity_index - col_index
            logging.debug("d: %s", d)
            distances.append((d, col)) if d >= 0 else None
            logging.debug("distance: %s", distances)
            msg[col_index] = ''
        logging.debug("distances: %s", distances)

        try:
            min_distance_index = argmin([tup[0] for tup in distances])
            return distances[min_distance_index]
        except ValueError:
            return None
   
    else:

        for en in entities:
            en_index = msg.index(en)
            d = entity_index - en_index
            distances.append(d) if d >= 0 else None
            msg[en_index] = ''
    
        try:
            return min(distances)
        except ValueError:
            return None
        

def helper_for_hat_bag(message_split : list, entities : list, not_entities : list, slot : str):
    if not entities:
        return {slot: None}   # caso: no entities
    
    if not_entities:
        message_split = not_replacement(message_split, not_entities)
        min_distance = minimum_distance(message_split, not_entities, entities[-1])

        if min_distance is not None and min_distance <= DISTANCE_TH2:
            return {slot: 'without'}
        else:
            return {slot: 'with'}
    else:
        return {slot: 'with'}



def latest_tracker_utter(events):
    for event in reversed(events):
        if event.get("event") == "bot":
            return event.get("text")
        

def print_person(person, utterance):
    if person['roi1_passages'] == 1:
        word = "time"
    else:
        word = "times"

    if person['roi2_passages'] == 1:
        word2 = "time"
    else:
        word2 = "times"

    if person['roi1_persistence_time'] > 0 and person['roi2_persistence_time'] > 0:
        utterance += f"has passed through {SHOP_DICT['roi1']} {person['roi1_passages']} " +word + f" and has spent a total of {person['roi1_persistence_time']} seconds there, "
        utterance += f"and has passed through {SHOP_DICT['roi2']} {person['roi2_passages']} " +word2 + f" and has spent a total of {person['roi2_persistence_time']} seconds there."
    else:
        if person['roi1_persistence_time'] > 0:
            utterance += f"has passed through {SHOP_DICT['roi1']} {person['roi1_passages']} " +word + f" and has spent a total of {person['roi1_persistence_time']} seconds there."
        elif person['roi2_persistence_time'] > 0:
            utterance += f"has passed through {SHOP_DICT['roi2']} {person['roi2_passages']} " +word2 + f" and has spent a total of {person['roi2_persistence_time']} seconds there."
    
    return utterance
            
