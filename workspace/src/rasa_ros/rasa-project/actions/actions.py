from typing import Dict, List, Text, Any

from rasa_sdk import Tracker, Action, FormValidationAction
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.forms import ValidationAction

from rasa_sdk.types import DomainDict
from .utils import * 
from .readerJson import *

import inflect
from rasa_sdk.events import SlotSet
import logging


#################### OUT FORM ####################

class ValidateCustomSlotMappings(ValidationAction):

    async def extract_upperColour(
        self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict
    ) -> Dict[Text, Any]:
        
        loop = tracker.active_loop
        if len(loop) != 0:
            return None
        
        loop = tracker.active_loop
        logging.debug("##################################################")
        logging.debug("tracker.active_loop['name']: %s", loop)
        logging.debug("##################################################")
    
        logging.debug("STO ESEGUENDO extract_upperColour NON PENSATO PER IL FORM")

        intent = tracker.get_intent_of_latest_message()
        message_split = cleaner(tracker.latest_message["text"]).split()
        colours = list(tracker.get_latest_entity_values("colour"))
        logging.debug("latest_colours: %s", ', '.join(colours))
        upper_entities = list(tracker.get_latest_entity_values("upper_body")) 
        

        if(intent == "ask_count" or intent == "ask_location"):
            if(len(upper_entities) == 0):
                return {"upperColour": None}
        
            upper = upper_entities[-1]
            
            logging.debug("latest_upper_body: %s", upper)
            logging.debug("words: %s", ', '.join(message_split))
           
            res = minimum_distance(message_split, colours, upper, mode='colour')
            
            if res is not None and res[0] <= DISTANCE_TH1:
                if upper in DRESS:
                    logging.debug("I have found an upperColour and a lowerColour and it is %s", res[1].lower())
                    return{"upperColour": res[1].lower(), "lowerColour": res[1].lower()}
                else:
                    logging.debug("I have found an upperColour and it is %s", res[1].lower())
                    return {"upperColour": res[1].lower()}
            else:
                return {"upperColour": None}
    




    async def extract_lowerColour(
        self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict
    ) -> Dict[Text, Any]:
        
        loop = tracker.active_loop
        if len(loop) != 0:
            return None
        
        logging.debug("STO ESEGUENDO extract_lowerColour NON PENSATO PER IL FORM")

        intent = tracker.get_intent_of_latest_message()
        message_split = cleaner(tracker.latest_message["text"]).split()
        colours = list(tracker.get_latest_entity_values("colour"))
        logging.debug("latest_colours: %s", ', '.join(colours))
        lower_enitities = list(tracker.get_latest_entity_values("lower_body"))
        

        if(intent == "ask_count" or intent == "ask_location"):
            if(len(lower_enitities) == 0):
                return {"lowerColour": None}
            
            lower = lower_enitities[-1]
        
            logging.debug("latest_lower_body: %s", lower)
            logging.debug("words: %s", ', '.join(message_split))

            res = minimum_distance(message_split, colours, lower, mode='colour')
            logging.debug("minimum distance %s", res)
         
            if res is not None and res[0] <= DISTANCE_TH1:
                logging.debug("I have found a lowerColour and it is %s", res[1].lower())
                return {"lowerColour": res[1].lower()}
            else:
                return {"lowerColour": None}
        
            




    async def extract_kindOfPeople(
        self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict
    ) -> Dict[Text, Any]:

        loop = tracker.active_loop
        if len(loop) != 0:
            return None
        
        logging.debug("STO ESEGUENDO extract_kindOfPeeple NON PENSATO PER IL FORM")
               
        intent = tracker.get_intent_of_latest_message()
        male_entities = list(tracker.get_latest_entity_values("male"))
        female_entities = list(tracker.get_latest_entity_values("female"))
        
        logging.debug("latest_male_entities:\t %s", male_entities)
        logging.debug("latest_female_entities:\t %s", female_entities)
        
        if(intent == "ask_count" or intent == "ask_location"):
            if (len(male_entities) > 0 and len(female_entities)== 0):
                return {"kindOfPeople": "M"}
            elif (len(male_entities) == 0 and len(female_entities) > 0):
                return {"kindOfPeople": "F"}
            else:
                return {"kindOfPeople": None}




    async def extract_hatSlot(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict) -> Dict[Text, Any]:

        loop = tracker.active_loop
        if len(loop) != 0:
            return None
       
        logging.debug("STO ESEGUENDO extract_hatSlot NON PENSATO PER IL FORM")

        intent = tracker.get_intent_of_latest_message()
        message_split = cleaner(tracker.latest_message["text"]).split()
        hat_entities = list(tracker.get_latest_entity_values("hat"))
        not_entities = list(tracker.get_latest_entity_values("not"))

        logging.debug('hat_entities:\t %s', hat_entities)
        logging.debug('not_entities:\t %s', not_entities)

        if(intent == "ask_count" or intent == "ask_location"):
            return helper_for_hat_bag(message_split, hat_entities, not_entities, "hatSlot")


    async def extract_bagSlot(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict) -> Dict[Text, Any]:

        loop = tracker.active_loop
        if len(loop) != 0:
            return None
        
        logging.debug("STO ESEGUENDO extract_bagSlot NON PENSATO PER IL FORM")

        intent = tracker.get_intent_of_latest_message()
        message_split = cleaner(tracker.latest_message["text"]).split()
        bag_entities = list(tracker.get_latest_entity_values("bag"))
        not_entities = list(tracker.get_latest_entity_values("not"))

        logging.debug('bag_entities:\t %s', bag_entities)
        logging.debug('not_entities:\t %s', not_entities)

        if(intent == "ask_count" or intent == "ask_location"):
            return helper_for_hat_bag(message_split, bag_entities, not_entities, "bagSlot")

        



    async def extract_place(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict) -> Dict[Text, Any]:
        # Logica: ci troviamo all'esterno dei form;
        # se l'utente specifica solo un negozio e non nomina il mall --> mettiamo il negozio in place
        # se l'utente specifica solo un mall e non nomina alcun negozio --> mettiamo il mall in place
        # in tutti gli altri casi --> lasciamo None in place
        # OSS: il metodo che dovrà fare l'action sul json, se vede che place==None --> assume che place=mall
        
        loop = tracker.active_loop
        if len(loop) != 0:
            return None
        
        logging.debug("STO ESEGUENDO extract_place NON PENSATO PER IL FORM")

        intent = tracker.get_intent_of_latest_message()
        mall_entities = list(set(tracker.get_latest_entity_values("mall")))
        shop_entities = list(set(tracker.get_latest_entity_values("shop")))
        
        logging.debug('latest_mall_entities:\t %s', mall_entities)
        logging.debug('latest_shop_entities:\t %s', shop_entities)
        
        if intent == 'ask_location':
            return {"place": None}

        if intent == "ask_count":
            if(len(shop_entities) == 1 and len(mall_entities) == 0):
                return {"place": shop_entities[0].lower()}
            elif(len(shop_entities) == 0 and len(mall_entities) == 1):
                return {"place": "mall"}
            else:
                return {"place": None}
        




    async def extract_compareSlot(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict) -> Dict[Text, Any]:

        loop = tracker.active_loop
        if len(loop) != 0:
            return None
        
        logging.debug("STO ESEGUENDO extract_compareSlot NON PENSATO PER IL FORM")

        intent = tracker.get_intent_of_latest_message()
        message = tracker.latest_message["text"]
        message_clean = cleaner(message)
        
        less_entities = list(tracker.get_latest_entity_values("less"))
        more_entities = list(tracker.get_latest_entity_values("more"))

        try:
            duration_dict = [x for x in tracker.latest_message["entities"] if x["entity"] == "duration"][0]
            duration_text = duration_dict["text"]
            duration_value = duration_dict['value']
        except IndexError:
            duration_dict = duration_text = None

        logging.debug('less_entities:\t %s', less_entities)
        logging.debug('more_entities:\t %s', more_entities)
        logging.debug('duration:\t %s', duration_text)

        if intent == 'ask_location':
            return {"compareSlot": None}

        if intent == "ask_count":
            if duration_text is not None:
                if len(less_entities) > 0 and len(more_entities) > 0: 
                    return None
                elif len(less_entities) > 0 and len(more_entities) == 0:
                    comparative = less_entities[-1]
                    entity = 'less'
                elif len(less_entities) == 0 and len(more_entities) > 0:
                    comparative = more_entities[-1]
                    entity = 'more'
                else:
                    return {"compareSlot": "equal"} # non ci sono less e more allora è un equal
                
                message = message.replace(duration_text, " " +str(duration_value) +" ")   # sostituisco il testo con il valore numerico
                message = message.split()
                logging.debug("message: %s", message)
                logging.debug("duration_value: %s", duration_value)
                logging.debug("duration text %s", duration_text)
                message_clean = message_clean.split()   # per l'estrazione di more/less uso il messaggio ripulito per assicurarmi che l'eventuale presenza di punteggiatura adiacente a more/less non dia fastidio
                diff = message.index(str(duration_value)) - message_clean.index(comparative)    # distanza tra tempo (value) e more/less
                
                return {"compareSlot": entity} if diff <= DISTANCE_TH1 and diff >= 0 else None
            
            else:

                return {"compareSlot": None}

            
    




    async def extract_duration(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict) -> Dict[Text, Any]:   

        loop = tracker.active_loop
        if len(loop) != 0:
            return None
        
        logging.debug("STO ESEGUENDO extract_duration NON PENSATO PER IL FORM")
        intent = tracker.get_intent_of_latest_message()

        if intent == 'ask_count':
            try:
                duration_dict = [x for x in tracker.latest_message["entities"] if x["entity"] == "duration"][0]
                duration_seconds = duration_dict["additional_info"]["normalized"]["value"]
                return {"duration": duration_seconds} 
            except IndexError:
                return {"duration": None}
        
        if intent == 'ask_location':
            return {"duration": None}
        
    




    async def extract_not_upperSlot(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict) -> Dict[Text, Any]:

        loop = tracker.active_loop
        if len(loop) != 0:
            return None
        
        logging.debug("STO ESEGUENDO extract_not_upperSlot NON PENSATO PER IL FORM")

        intent = tracker.get_intent_of_latest_message()
        message_split = cleaner(tracker.latest_message["text"]).split()
        not_entities = list(tracker.get_latest_entity_values("not"))
        upper_entities = list(tracker.get_latest_entity_values("upper_body"))

        logging.debug('not_entities:\t %s', not_entities)
        logging.debug('upper_entities:\t %s', upper_entities)

        if(intent == "ask_count" or intent == "ask_location"):
            
            if not upper_entities:
                return {"not_upperSlot": False}
            
            else:
                upper = upper_entities[-1]

                if not not_entities:
                    if upper in DRESS:
                        return {"not_upperSlot": False, "not_lowerSlot": False}
                    else:
                        return {"not_upperSlot": False} 
                
                message_split = not_replacement(message_split, not_entities) 
                min_distance = minimum_distance(message_split, not_entities, upper)

                logging.debug('min_distance_not_upper:\t %s', min_distance)

                if min_distance is not None and min_distance <= DISTANCE_TH2:
                    if upper in DRESS:
                        return {"not_upperSlot": True, "not_lowerSlot": True}
                    else:
                        return {"not_upperSlot": True}
                
                return {"not_upperSlot": False} # if mins_distance is None or min_distance > DISTANCE_TH_NOT (nessun not si riferisce all upper)

           



    async def extract_not_lowerSlot(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: DomainDict) -> Dict[Text, Any]:

        loop = tracker.active_loop
        if len(loop) != 0:
            return None

        logging.debug("STO ESEGUENDO extract_not_lowerSlot NON PENSATO PER IL FORM")

        intent = tracker.get_intent_of_latest_message()
        message_split = cleaner(tracker.latest_message["text"]).split()
        not_entities = list(tracker.get_latest_entity_values("not"))
        lower_entities = list(tracker.get_latest_entity_values("lower_body"))

        logging.debug('not_entities:\t %s', not_entities)
        logging.debug('lower_entities:\t %s', lower_entities)

        if (intent == "ask_count" or intent == "ask_location"):
            if not lower_entities:
                return {"not_lowerSlot": False}
            
            else:
            
                if not not_entities:
                    return {"not_lowerSlot": False}
                
                message_split = not_replacement(message_split, not_entities)
                min_distance = minimum_distance(message_split, not_entities, lower_entities[-1])
                
                logging.debug('min_distance_not_lower:\t %s', min_distance)

                if min_distance is not None and min_distance <= DISTANCE_TH2:
                    return {"not_lowerSlot": True}
                else:
                    return {"not_lowerSlot": False}
    


# -----------------------------------------------------------------------------------------------------------------------------------------



class ValidateFatherForm(FormValidationAction):

    _form_upperColour = ""
    _form_lowerColour = ""
    _form_kindOfPeople = ""
    _form_hatSlot = ""
    _form_bagSlot = ""
    _attention_intents = ""


    def name(self) -> Text:
        return "validate_father_form"


    async def extract_upperColour(
        self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict
    ) -> Dict[Text, Any]: 
        
        # QUESTION: "What is the main colour of the upper clothes?"

        logging.debug("STO ESEGUENDO extract_upperColour PENSATO PER IL FORM")

        logging.debug("UPPER_COLOR: %s", tracker.get_slot("upperColour"))
        logging.debug("LOWER_COLOR: %s", tracker.get_slot("lowerColour"))
        logging.debug("KIND_OF_POPLE: %s", tracker.get_slot("kindOfPeople"))
        logging.debug("HAT_SLOT: %s", tracker.get_slot("hatSlot"))
        logging.debug("BAG_SLOT: %s", tracker.get_slot("bagSlot"))
        logging.debug("PLACE: %s", tracker.get_slot("place"))
        logging.debug("NOT_UPPER_SLOT: %s", tracker.get_slot("not_upperSlot"))
        logging.debug("NOT_LOWER_SLOT: %s", tracker.get_slot("not_lowerSlot"))

        intent_of_last_user_message = tracker.get_intent_of_latest_message()
        latest_message = (tracker.latest_message["text"])
        logging.debug("%s",latest_message)
        logging.debug("intent_of_last_user_message: %s", intent_of_last_user_message)

        latest_utter = latest_tracker_utter(tracker.events)
        
        if self._form_upperColour in latest_utter :

            if intent_of_last_user_message == "unknown":
                logging.debug("Metto Free")
                return {"upperColour": "free"} # qualsiasi colore
            
            elif intent_of_last_user_message == "inform" or intent_of_last_user_message in self._attention_intents:

                message_split = cleaner(tracker.latest_message["text"]).split()
                message_plus = ["_", "_"]
                message_split = message_plus + message_split + message_plus
                logging.debug("message_split: %s", message_split)

                latest_colours = list(tracker.get_latest_entity_values("colour"))

                if(len(latest_colours) == 0):
                    return {"upperColour": None}

                latest_lowers = list(tracker.get_latest_entity_values("lower_body"))
                latest_hats = list(tracker.get_latest_entity_values("hat"))
                latest_bags = list(tracker.get_latest_entity_values("bag"))

                good_candidates = []
                for col in latest_colours:
                    col_ind = message_split.index(col)
                    cond_lowers = message_split[col_ind+1] in latest_lowers or message_split[col_ind+2] in latest_lowers
                    cond_hats = message_split[col_ind+1] in latest_hats or message_split[col_ind+2] in latest_hats
                    cond_bags = message_split[col_ind+1] in latest_bags or message_split[col_ind+2] in latest_bags
                    if not cond_lowers and not cond_hats and not cond_bags:
                        good_candidates.append([col, col_ind])

                if len(good_candidates) == 1:
                    logging.debug("colore buono per UPPER: " +good_candidates[0][0])
                    lc_message_split = [word.lower() for word in message_split]
                    if lc_message_split[good_candidates[0][1]-1] == "but":
                        logging.debug("good_candidates: %s", good_candidates)
                        logging.debug("good_candidates[0][1]-1: ", good_candidates[0][1]-1)
                        return {"upperColour": good_candidates[0][0].lower(), "not_upperSlot": True}

                    not_entities = list(tracker.get_latest_entity_values("not"))
                    message_split = not_replacement(message_split, not_entities)
                    min_distance = minimum_distance(message_split, not_entities, good_candidates[0][0])
                    

                    if min_distance is not None and min_distance <= DISTANCE_TH1:
                        return {"upperColour": good_candidates[0][0].lower(), "not_upperSlot": True}
                    else:
                        return {"upperColour": good_candidates[0][0].lower(), "not_upperSlot": False}

                return {"upperColour": None}       
            
            else:
                return {"upperColour": None}
        





    async def extract_lowerColour(
        self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict
    ) -> Dict[Text, Any]: 
        
        # QUESTION: "What is the main colour of the lower clothes?"

        logging.debug("STO ESEGUENDO extract_lowerColour PENSATO PER IL FORM")


        intent_of_last_user_message = tracker.get_intent_of_latest_message()
        latest_message = (tracker.latest_message["text"])
        logging.debug("%s", latest_message)
        logging.debug("intent_of_last_user_message: %s", intent_of_last_user_message)


        latest_utter = latest_tracker_utter(tracker.events)
        
        if self._form_lowerColour in latest_utter :

            if intent_of_last_user_message == "unknown":
                logging.debug("Metto Free")
                return {"lowerColour": "free"} # qualsiasi colore
            
            elif intent_of_last_user_message == "inform" or intent_of_last_user_message in self._attention_intents:

                message_split = cleaner(tracker.latest_message["text"]).split()
                message_plus = ["_", "_"]
                message_split = message_plus + message_split + message_plus
                logging.debug("message_split: %s", message_split)

                latest_colours = list(tracker.get_latest_entity_values("colour"))

                if(len(latest_colours) == 0):
                    return {"lowerColour": None}
                
                latest_uppers = list(tracker.get_latest_entity_values("upper_body"))
                latest_hats = list(tracker.get_latest_entity_values("hat"))
                latest_bags = list(tracker.get_latest_entity_values("bag"))

                good_candidates = []
                for col in latest_colours:
                    col_ind = message_split.index(col)
                    cond_hats = message_split[col_ind+1] in latest_hats or message_split[col_ind+2] in latest_hats
                    cond_bags = message_split[col_ind+1] in latest_bags or message_split[col_ind+2] in latest_bags
                    cond_uppers = message_split[col_ind+1] in latest_uppers or message_split[col_ind+2] in latest_uppers
                    if not cond_uppers and not cond_hats and not cond_bags:
                         good_candidates.append([col, col_ind])

                if len(good_candidates) == 1:
                    logging.debug("colore buono per LOWER: " +good_candidates[0][0])
                    lc_message_split = [word.lower() for word in message_split]
                    if lc_message_split[good_candidates[0][1]-1] == "but":
                        logging.debug("good_candidates: %s", good_candidates)
                        logging.debug("good_candidates[0][1]-1: ", good_candidates[0][1]-1)
                        return {"lowerColour": good_candidates[0][0].lower(), "not_lowerSlot": True}

                    not_entities = list(tracker.get_latest_entity_values("not"))
                    message_split = not_replacement(message_split, not_entities)
                    min_distance = minimum_distance(message_split, not_entities, good_candidates[0][0])
                    

                    if min_distance is not None and min_distance <= DISTANCE_TH1:
                        return {"lowerColour": good_candidates[0][0].lower(), "not_lowerSlot": True}
                    else:
                        return {"lowerColour": good_candidates[0][0].lower(), "not_lowerSlot": False}

                return {"lowerColour": None}

            else:
                return {"lowerColour": None}





    async def extract_kindOfPeople(
        self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict
    ) -> Dict[Text, Any]: 
        
        # "Are you looking for somebody who is male or female?"

        logging.debug("STO ESEGUENDO extract_kindOfPeeple PENSATO PER IL FORM")

        intent_of_last_user_message = tracker.get_intent_of_latest_message()
        latest_message = (tracker.latest_message["text"])
        logging.debug("%s", latest_message)
        logging.debug("intent_of_last_user_message: %s", intent_of_last_user_message)

        latest_utter = latest_tracker_utter(tracker.events)
        
        if self._form_kindOfPeople in latest_utter :
            if intent_of_last_user_message == "unknown":
                logging.debug("Metto Asessuato")
                return {"kindOfPeople": "A"}
            elif intent_of_last_user_message == "inform" or intent_of_last_user_message in self._attention_intents:
                latest_people = list(tracker.get_latest_entity_values("people"))
                latest_males = list(tracker.get_latest_entity_values("male"))
                latest_females = list(tracker.get_latest_entity_values("female"))

                if len(latest_people) > 0 and len(latest_males) == 0 and len(latest_females) == 0:
                    logging.debug("Metto Altro (genere non specificato)")
                    return {"kindOfPeople": "A"}
                elif len(latest_males) > 0 and len(latest_females) == 0:
                    logging.debug("Metto Maschio")
                    return {"kindOfPeople": "M"}
                elif len(latest_females) > 0 and len(latest_males) == 0:
                    logging.debug("Metto Femmina")
                    return {"kindOfPeople": "F"}
                elif len(latest_males) > 0 and len(latest_females) > 0:
                    logging.debug("Metto Altro (genere non specificato)")
                    return {"kindOfPeople": "A"}
                else:
                    return {"kindOfPeople": None}
            else:
                return {"kindOfPeople": None}



        

    async def extract_hatSlot(
        self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict
    ) -> Dict[Text, Any]: 
        
        # QUESTION: Are you looking for someone with a hat or without?"

        logging.debug("STO ESEGUENDO extract_hatSlot PENSATO PER IL FORM")

        intent_of_last_user_message = tracker.get_intent_of_latest_message()

        latest_utter = latest_tracker_utter(tracker.events)
        
        if self._form_hatSlot in latest_utter :

            if intent_of_last_user_message == "unknown":
                return {"hatSlot": "both"}
            
            elif intent_of_last_user_message == "inform" or intent_of_last_user_message in self._attention_intents:
                # se rileviamo cappello e un not vicino --> allora mettiamo without
                # se rileviamo cappello e nessun not in vicinanza --> allora mettiamo with
                # in tutti gli altri casi --> lasciamo None

                message_split = cleaner(tracker.latest_message["text"]).split()
                hat_entities = list(tracker.get_latest_entity_values("hat"))
                not_entities = list(tracker.get_latest_entity_values("not"))

                lc_message_split = [word.lower() for word in message_split]
                if "with" in lc_message_split:
                    return {"hatSlot": 'with'}   # caso presenza with anche se non c'è alcuna entità cappello/borsa
                if "without" in lc_message_split:
                    return {"hatSlot": 'without'} 

                return helper_for_hat_bag(message_split, hat_entities, not_entities, "hatSlot")

            else:
                return {"hatSlot": None}



    async def extract_bagSlot(
        self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict
    ) -> Dict[Text, Any]: 
        
        # QUESTION: "Are you looking for someone with a bag or without?"

        logging.debug("STO ESEGUENDO extract_bagSlot PENSATO PER IL FORM")

        intent_of_last_user_message = tracker.get_intent_of_latest_message()

        latest_utter = latest_tracker_utter(tracker.events)
        
        if self._form_bagSlot in latest_utter :

            if intent_of_last_user_message == "unknown":
                return {"bagSlot": "both"}
            
            elif intent_of_last_user_message == "inform" or intent_of_last_user_message in self._attention_intents:

                # se rileviamo borsa e un not vicino --> allora mettiamo without
                # se rileviamo borsa e nessun not in vicinanza --> allora mettiamo with
                # in tutti gli altri casi --> lasciamo None

                message_split = cleaner(tracker.latest_message["text"]).split()
                bag_entities = list(tracker.get_latest_entity_values("bag"))
                not_entities = list(tracker.get_latest_entity_values("not"))

                lc_message_split = [word.lower() for word in message_split]
                if "with" in lc_message_split:
                    return {"bagSlot": 'with'}   # caso presenza with anche se non c'è alcuna entità cappello/borsa
                if "without" in lc_message_split:
                    return {"bagSlot": 'without'} 

                return helper_for_hat_bag(message_split, bag_entities, not_entities, "bagSlot")

            else:
                return {"bagSlot": None}

    
    async def extract_place(
        self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict
    ) -> Dict[Text, Any]: 
        
        # "Do you want to consider the entire mall, walmart or starbucks?"

        logging.debug("STO ESEGUENDO extract_place PENSATO PER IL FORM")

        intent_of_last_user_message = tracker.get_intent_of_latest_message()

        latest_utter = latest_tracker_utter(tracker.events)

        if self._form_place in latest_utter :
            if intent_of_last_user_message == "unknown":
                return {"place": "mall"}
            
            elif intent_of_last_user_message == "inform" or intent_of_last_user_message in self._attention_intents:        

                # latest_message = cleaner(tracker.latest_message["text"])  non viene usato quindi commentato
                latest_malls = list(tracker.get_latest_entity_values("mall"))
                latest_shops = list(tracker.get_latest_entity_values("shop"))
                if(len(latest_shops) == 1 and len(latest_malls) == 0):
                    return {"place": latest_shops[0].lower()}
                elif(len(latest_shops) == 0 and len(latest_malls) == 1):
                    return {"place": "mall"}
                else:
                    return {"place": None}





# -----------------------------------------------------------------------------------------------------------------------------------------



class ValidateLocationForm(ValidateFatherForm):

    _form_upperColour = "upper clothes"
    _form_lowerColour = "lower clothes"
    _form_kindOfPeople = "who is male or female"
    _form_hatSlot = "with a hat or without"
    _form_bagSlot = "with a bag or without"
    _form_place = "---"
    _attention_intents = ["ask_location"]

    def name(self) -> Text:
        return "validate_location_form"





# -----------------------------------------------------------------------------------------------------------------------------------------

# i metodi di ValidateCountForm funzionano solo quando siamo all'interno del form count. Quindi l aggiunta di ask_location in self._attention_intents non crea problemi per il location form

class ValidateCountForm(ValidateFatherForm):
    
    _form_upperColour = "the upper clothing"
    _form_lowerColour = "the lower clothing"
    _form_kindOfPeople = "people in general"
    _form_hatSlot = "with hat, without hat, or both"
    _form_bagSlot = "with bag, without bag, or both"
    _form_place = "entire mall, walmart or starbucks"
    _attention_intents = ["ask_count", "ask_location"]          # abbiamo aggiunto ask location perchè possibile che l'utente nel count form si riferisca ad un count intent usando 'i m looking for' (ask_location)

    def name(self) -> Text:
        return "validate_count_form"



# -----------------------------------------------------------------------------------------------------------------------------------------



class ActionConfirmationCount(Action):

    def name(self) -> Text:
        return "action_ask_confirmation_count"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> Dict[Text, Any]:

        upperColour = tracker.get_slot("upperColour")
        lowerColour = tracker.get_slot("lowerColour")
        kindOfPeople = tracker.get_slot("kindOfPeople")
        hatSlot = tracker.get_slot("hatSlot")
        bagSlot = tracker.get_slot("bagSlot")
        place = tracker.get_slot("place")
        not_lowerSlot = tracker.get_slot("not_lowerSlot")
        not_upperSlot = tracker.get_slot("not_upperSlot")
        duration = tracker.get_slot("duration")
        compareSlot = tracker.get_slot("compareSlot")
        
        utterance = "So, if I understand well, you want to count "

        if kindOfPeople == "M":
            utterance += "males "
        elif kindOfPeople == "F":
            utterance += "females "
        else:
            utterance += "people "
        

        if upperColour is None:
            pass
        elif upperColour == "free":
            utterance += "with upper clothes of any colour, "
        else:
            if not_upperSlot == False: # not_upperSlot non può essere None
                utterance += "with " +upperColour +" upper clothes, "
            else:
                utterance += "without " +upperColour +" upper clothes, "
        

        if lowerColour is None:
            pass
        elif lowerColour == "free":
            utterance += "with lower clothes of any colour, "
        else:
            if not_lowerSlot == False: # not_lowerSlot non può essere None
                utterance += "with " +lowerColour +" lower clothes, "
            else:
                utterance += "without " +lowerColour +" lower clothes, "
        
        if hatSlot == "with":
            utterance += "with a hat, "
        elif hatSlot == "without":
            utterance += "without a hat, "
          
        
        if bagSlot == "with":
            utterance += "with a bag, "
        elif bagSlot == "without":
            utterance += "without a bag, "
 
        
        if duration is None:
            pass
        else:
            utterance += "who have been "
        

        if place == "mall" or place is None:
            utterance += "in the mall"
        elif place == "walmart":
            utterance += "in walmart"
        elif place == "starbucks":
            utterance += "in starbucks"
        

        if compareSlot == "more":
            utterance += " for more than "
        elif compareSlot == "less":
            utterance += " for less than "
        elif compareSlot == "equal":
            utterance += " for "
        else:
            utterance += "."
        
        
        if duration is not None:
            duration_seconds = int(duration)
            if duration < 60:
                utterance += str(duration_seconds) +" seconds."
            else:
                duration_minutes = int(duration_seconds / 60) # approximate to integer to avoid long results like 3.56583333333
                utterance += str(duration_minutes) +" minutes."
        
        utterance += " Is it right?"
        logging.debug(utterance)
        dispatcher.utter_message(utterance)
        return []
    

# -----------------------------------------------------------------------------------------------------------------------------------------


class ActionConfirmationLocation(Action):

    def name(self) -> Text:
        return "action_ask_confirmation_location"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> Dict[Text, Any]:

        upperColour = tracker.get_slot("upperColour")
        lowerColour = tracker.get_slot("lowerColour")
        kindOfPeople = tracker.get_slot("kindOfPeople")
        hatSlot = tracker.get_slot("hatSlot")
        bagSlot = tracker.get_slot("bagSlot")
        place = tracker.get_slot("place")
        not_lowerSlot = tracker.get_slot("not_lowerSlot")
        not_upperSlot = tracker.get_slot("not_upperSlot")

        utterance = "So, if I understand well, you want to localize "

        if kindOfPeople == "M":
            utterance += "a male "
        elif kindOfPeople == "F":
            utterance += "a female "
        elif kindOfPeople == "A":
            utterance += "a person "

        # hatSlot non può essere None
        if hatSlot == "with":
            utterance += "with a hat, "
        elif hatSlot == "without":
            utterance += "without a hat, "
        elif hatSlot == "both":
            utterance += ""
        
        # bagSlot non può essere None
        if bagSlot == "with":
            utterance += "with a bag, "
        elif bagSlot == "without":
            utterance += "without a bag, "
        elif bagSlot == "both":
            utterance += ""
        
        # upperColour non può essere None
        if upperColour == "free":
            utterance += "with upper clothes of any colour, "
        else:
            if not_upperSlot == False: # not_upperSlot non può essere None
                utterance += "with " +upperColour +" upper clothes, "
            else:
                utterance += "without " +upperColour +" upper clothes, "
        
        # lowerColour non può essere None
        if lowerColour == "free":
            utterance += "with lower clothes of any colour."
        else:
            if not_lowerSlot == False: # not_lowerSlot non può essere None
                utterance += "with " +lowerColour +" lower clothes."
            else:
                utterance += "without " +lowerColour +" lower clothes."
    
        utterance += " Is it right?"
        logging.debug(utterance)
        dispatcher.utter_message(utterance)
        return []
        

# -----------------------------------------------------------------------------------------------------------------------------------------


class ActionCount(Action):

    def name(self) -> Text:
        return 'action_count_answer'
    
    def run(self, dispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        db = ReaderJson()
        constraints = db.make_constraints(tracker)
        number_of_people, _ = db.search_on_database(constraints)
        
        if number_of_people == 0:
            utterance = "I have not found anyone that matches your counting request"
        elif number_of_people == 1:
            utterance = "I have found one person that matches your counting request"
        else:
            utterance = f"I have found {number_of_people} people that match your counting request"
        
        dispatcher.utter_message(text = utterance)
        
        return list()





class ActionLocation(Action):

    def name(self) -> Text:
        return 'action_location_answer'
    
    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
            
            db = ReaderJson()
            constraints = db.make_constraints(tracker)
            number_of_people, people = db.search_on_database(constraints)
            
            if number_of_people == 0:
                utterance = "I have not found anyone that matches your localization request"
            elif number_of_people == 1:
                utterance = "I have found one person that matches your localization request. Such person "
                
                person = people[0]

                utterance = print_person(person, utterance)

                # if person['roi1_passages'] == 1:
                #     word = "time"
                # else:
                #     word = "times"
                
                # if person['roi2_passages'] == 1:
                #     word2 = "time"
                # else:
                #     word2 = "times"

                # if person['roi1_persistence_time'] > 0 and person['roi2_persistence_time'] > 0:
                #     utterance += f"has passed through {SHOP_DICT['roi1']} {person['roi1_passages']} " +word + f" and has spent a total of {person['roi1_persistence_time']} seconds there, "
                #     utterance += f"and has passed through {SHOP_DICT['roi2']} {person['roi2_passages']} " +word2 + f" and has spent a total of {person['roi2_persistence_time']} seconds there."
                # else:
                #     if person['roi1_persistence_time'] > 0:
                #         utterance += f"has passed through {SHOP_DICT['roi1']} {person['roi1_passages']} " +word + f" and has spent a total of {person['roi1_persistence_time']} seconds there."
                #     elif person['roi2_persistence_time'] > 0:
                #         utterance += f"has passed through {SHOP_DICT['roi2']} {person['roi2_passages']} " +word2 + f" and has spent a total of {person['roi2_persistence_time']} seconds there."

            else:
                
                utterance = f"I have found {number_of_people} people that match your localization request.\n"

                p = inflect.engine()
                for i, person in enumerate(people):
                    utterance += f"The {p.number_to_words(p.ordinal(i+1))} person "

                    utterance = print_person(person, utterance)
                    
                    # if person['roi1_passages'] == 1:
                    #     word = "time"
                    # else:
                    #     word = "times"
                    
                    # if person['roi2_passages'] == 1:
                    #     word2 = "time"
                    # else:
                    #     word2 = "times"
                    
                    # if person['roi1_persistence_time'] > 0 and person['roi2_persistence_time'] > 0:
                    #     utterance += f"has passed through {SHOP_DICT['roi1']} {person['roi1_passages']} " +word + f" and has spent a total of {person['roi1_persistence_time']} seconds there, "
                    #     utterance += f"and has passed through {SHOP_DICT['roi2']} {person['roi2_passages']} " +word2 + f" and has spent a total of {person['roi2_persistence_time']} seconds there."
                    # else:
                    #     if person['roi1_persistence_time'] > 0:
                    #         utterance += f"has passed through {SHOP_DICT['roi1']} {person['roi1_passages']} " +word + f" and has spent a total of {person['roi1_persistence_time']} seconds there."
                    #     elif person['roi2_persistence_time'] > 0:
                    #         utterance += f"has passed through {SHOP_DICT['roi2']} {person['roi2_passages']} " +word2 + f" and has spent a total of {person['roi2_persistence_time']} seconds there."

                    utterance += "\n"
            
            dispatcher.utter_message(text = utterance)
            
            return list()






