import json
import yaml
import datetime
from frongo.fremen_interface import *

def get_field(item, key):
    fields = key.split('.')
    value=item
    for i in fields:
        value = value[i]
    return value

class TModels(object):

    def __init__(self, name, data_field='data', model_type='standard', 
                 data_type='boolean', data_conf='', timestamp_field='_meta.inserted_at',
                 timestamp_type='datetime', query='{}', db='message_store', collection='message_store'):
        self.name=name
        self.model_type=model_type
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
        self.anomalyTimes=[]
        self.anomalyValues=[]

        self._set_data_configuration()
        self._fremen = fremen_interface()

        if self.data_type == 'boolean':
            self.min_value=False
            self.max_value=True
        self.unknown=True

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

    def _add_state(self, epoch, state):
        self.unknown=False
        self.states.append(state)
        self.epochs.append(epoch)
        

    def _add_entry(self, entry):
        self.unknown=False
        if self.timestamp_type == 'datetime':
            epoch = int(get_field(entry, self.timestamp_field).strftime('%s'))
        else:
            epoch = int(get_field(entry, self.timestamp_field))
        state=self._decode_state(entry)

        if state != 'none':
            self._set_min_max_values(state, epoch)
                
            self.states.append(state)
            self.epochs.append(epoch)

    def _set_min_max_values(self, state, epoch):
        if hasattr(self, 'min_epoch'):
            if epoch< self.min_epoch:
                self.min_epoch=epoch
                self.from_date=datetime.datetime.fromtimestamp(epoch).strftime('%Y-%m-%d %H:%M:%S')
        else:
            self.min_epoch=epoch
            self.from_date=datetime.datetime.fromtimestamp(epoch).strftime('%Y-%m-%d %H:%M:%S')
            
        if hasattr(self, 'max_epoch'):
            if epoch > self.max_epoch:
                self.max_epoch=epoch
                self.to_date=datetime.datetime.fromtimestamp(epoch).strftime('%Y-%m-%d %H:%M:%S')
        else:
            self.max_epoch=epoch
            self.to_date=datetime.datetime.fromtimestamp(epoch).strftime('%Y-%m-%d %H:%M:%S')

        if self.data_type != 'boolean':
            if hasattr(self, 'min_value'):
                if epoch< self.min_value:
                    self.min_value=state
            else:
                self.min_value=state
                
            if hasattr(self, 'max_value'):
                if epoch > self.max_value:
                    self.max_value=state
        else:
            self.max_value=state            

    def _decode_state(self, entry):
        state = 'none'
        if self.data_type == 'boolean':
            if not self.data_conf:
                state = get_field(entry, self.data_field)
            else:
                a = get_field(entry, self.data_field)
                if a in self._dconf["True"]:
                    state=True
                if a in self._dconf["False"]:
                    state=False
        elif self.data_type == 'float':
            if not self.data_conf:
                state = get_field(entry, self.data_field)
            else:
                a = float(get_field(entry, self.data_field))
                if self._dconf.has_key('max'):
                    v_max=float(self._dconf["max"])
                else:
                    v_max=1.0
                if self._dconf.has_key('min'):
                    v_min=float(self._dconf["min"])
                else:
                    v_min=0.0
                state = float((a-v_min)/(v_max-v_min))

        else:
            state = float(get_field(entry, self.data_field))
            
        return state
        
    def _create_fremen_models(self):
        if not self.unknown:
            self.order = self._fremen.create_fremen_model(self.name, self.epochs, self.states, self.data_type)
        else:
            self.order = 0

    def _detect_anomalies(self, order, confidence):
        if not self.unknown:
            self.anomalyTimes, self.anomalyValues = self._fremen.detect_anomalies(self.name, self.epochs, self.states, order, confidence)
        return self.anomalyTimes, self.anomalyValues
        


    def _predict_outcome(self, epochs, order=-1):
        if not self.unknown:
            if order < 0:
                order= self.order
            probs=self._fremen.predict_outcome(epochs, self.name, order)
            return probs
        else:
            probs=[]
            for i in epochs:
                probs.append(0.5)
            return probs

    
    def _predict_entropy(self, epochs, order=-1):
        if not self.unknown:
            if order < 0:
                order= self.order
            probs=self._fremen.predict_entropy(epochs, self.name, order)
            print probs
            return probs
        else:
            probs=[]
            for i in epochs:
                probs.append(0.5)
            return probs


    def _set_unknown(self, status):
        self.unknown=status

    def _get_states(self, epochs):
        ret_epochs=[]
        ret_states=[]
        if len(epochs) < 2:
            lowr=self.min_epoch
            highr=self.max_epoch
        else:
            lowr=epochs[0]
            highr=epochs[1]
        
        for i in range(len(self.epochs)):
            if lowr <= self.epochs[i] <= highr:
                ret_epochs.append(self.epochs[i])
                ret_states.append(self.states[i])
                
        #print "GET STATES: ", len(ret_epochs), len(ret_states)
        return ret_epochs, ret_states


    def _get_info(self):
        a = dir(self)
        b =[]
        s = {}
        for i in a:
            if not i.startswith('_'):
                b.append(str(i))
      
        for i in b:
            if type(self.__getattribute__(i)) is not list:
                s[str(i)] =  self.__getattribute__(i)
            else:
                if not self.unknown:
                    if len(self.__getattribute__(i)) > 0:
                        st= '[list with ' + str(len(self.__getattribute__(i))) +' ' + str(type(self.__getattribute__(i)[0])) + ' elements]'
                    else:
                        st= '[list with 0 elements]'

                    s[str(i)] = st
                           
        out=yaml.safe_dump(s,default_flow_style=False)
        return out

    def _dump_to_file(self, filename):
        fh = open(filename, "w")
        fh.close

        for i in range(len(self.epochs)):
            fh = open(filename, "a")
            s_output = "%d; %d\n" %(self.epochs[i], self.states[i])
            fh.write(s_output)
        fh.close

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
                if len(self.__getattribute__(i)) > 0:
                    s = s + str(i) +': [list with ' + str(len(self.__getattribute__(i))) +' ' + str(type(self.__getattribute__(i)[0])) + ' elements]\n'
                else:
                    s = s + str(i) +': [list with ' + str(len(self.__getattribute__(i))) +' ' + str(0) + ' elements]\n'
        return s
        