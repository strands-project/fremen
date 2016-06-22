import json
from frongo.fremen_interface import *

def get_field(item, key):
    fields = key.split('.')
    value=item
    for i in fields:
        value = value[i]
    return value

class TModels(object):

    def __init__(self, name, data_field='data', data_type='boolean', data_conf='', 
                 timestamp_field='_meta.inserted_at', timestamp_type='datetime',
                 query='{}', db='message_store', collection='message_store'):
        self.name=name
        self.db=db
        self.collection=collection
        self.query=query
        self.data_field=data_field
        self.data_type=data_type
        self.data_conf=data_conf
        self.timestamp_field = timestamp_field
        self.timestamp_type = timestamp_type
        self.order = -1
        self.epochs=[]
        self.states=[]
        self._set_data_configuration()
        self._fremen = fremen_interface()

    def _set_props_from_dict(self, data):
        for i in data.keys():
            if i in dir(self):
                self.__setattr__(i,data[i])

        self._set_data_configuration()

    def _set_data_configuration(self):
        if self.data_conf != '':
            self._dconf=json.loads(self.data_conf)
        else:
            self._dconf=self.data_conf

    def _add_entry(self, entry):            
        if self.timestamp_type == 'datetime':
            epoch = int(get_field(entry, self.timestamp_field).strftime('%s'))
        else:
            epoch = int(get_field(entry, self.timestamp_field))
        state=self._decode_state(entry)

        self.states.append(state)
        self.epochs.append(epoch)

    def _decode_state(self, entry):
        if self.data_type == 'boolean':
            if not self.data_conf:
                state = get_field(entry, self.data_field)
            else:
                a = get_field(entry, self.data_field)
                if a in self._dconf["True"]:
                    state=True
                if a in self._dconf["False"]:
                    state=False
        else:
            state = get_field(entry, self.data_field)
        return state
        
    def _create_fremen_models(self):
        self.order = self._fremen.create_fremen_model(self.name, self.epochs, self.states, self.data_type)


    def _predict_outcome(self, epochs, order=-1):
        if order < 0:
            order= self.order
        probs=self._fremen.predict_outcome(epochs, self.name, order)
        return probs

    def __repr__(self):
        a = dir(self)
        b =[]
        s = ''
        for i in a:
            if not i.startswith('_'):
                b.append(str(i))
      
        for i in b:
            if type(self.__getattribute__(i)) is not list:
                s = s + str(i) +': ' + str(self.__getattribute__(i)) + '\n'
            else:
                s = s + str(i) +': [list with ' + str(len(self.__getattribute__(i))) +' ' + str(type(self.__getattribute__(i)[0])) + ' elements]\n'

        return s
        