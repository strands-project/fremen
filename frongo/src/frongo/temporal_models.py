
class TModels(object):

    def __init__(self, name, data_field='data', data_type='boolean', 
                 timestamp_field='_meta.inserted_at', timestamp_type='datetime',
                 query='{}', db='message_store', collection='message_store'):
        self.name=name
        self.data_field=data_field
        self.timestamp_field = timestamp_field
        self.timestamp_type = timestamp_type
        self.query=query
        self.db=db
        self.collection=collection
        self.epochs=[]
        self.states=[]

    def _set_props_from_dict(self, data):
        if data.has_key('name'):
            self.name=data['name']
        if data.has_key('query'):
            self.query=data['query']
        if data.has_key('db'):
            self.db=data['db']
        if data.has_key('collection'):
            self.collection=data['collection']
        if data.has_key('timestamp_field'):
            self.timestamp_field=data['timestamp_field']
        if data.has_key('timestamp_type'):
            self.timestamp_type=data['timestamp_type']
        if data.has_key('data_field'):
            self.data_field=data['data_field']
        if data.has_key('data_type'):
            self.data_type=data['data_type']
    
    def __repr__(self):
        a = dir(self)
        b =[]
        s = ''
        for i in a:
            if not i.startswith('_'):
                b.append(str(i))
      
        for i in b:
            s = s + str(i) +': ' + str(self.__getattribute__(i)) + '\n'
            #print i, self.__getattribute__(i)
        return s
        