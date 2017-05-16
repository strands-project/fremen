#!/usr/bin/env python

import rospy
import yaml
import json
import pymongo
import argparse
import numpy as np

import std_msgs

from threading import Lock

from std_srvs.srv import Trigger
from frongo.temporal_models import *
from frongo.graph_models import *
from frongo.srv import PredictState
from frongo.srv import PredictStateOrder
from frongo.srv import GraphModel
from frongo.srv import GetInfo
from frongo.srv import AddModel
from frongo.srv import DetectAnomalies



def get_field(entry, field_name):
    la=field_name.split('.')
    val=entry
    for i in la:
        val=val[i]
        
    return val


def load_yaml(filename):
    data=[]
    rospy.loginfo("loading %s"%filename)
    with open(filename, 'r') as f:
        datum = yaml.load(f)
        if not isinstance(datum, list):
            data.append(datum)
        else:
            data=datum
        return data


class frongo(object):

    def __init__(self, data) :
        self.is_fremen_active=False
        self.models=[]
        rospy.on_shutdown(self._on_node_shutdown)
        host = rospy.get_param("mongodb_host")
        port = rospy.get_param("mongodb_port")
        self.srv_lock=Lock()
        
        rospy.loginfo("Setting up Mongo client on %s:%d" %(host,port))
        self.mongo_client = pymongo.MongoClient(host, port)

        if data:
            self.create_models(data)
            for i in self.models:
                print i

        # Subscribe to fremen server start topic
        rospy.Subscriber('/fremenserver_start', std_msgs.msg.Bool, self.fremen_restart_cb)
        rospy.loginfo("... Done")

        rospy.sleep(3)

        #Advertise Service
        self.get_states_srv=rospy.Service('/frongo/get_states', PredictState, self.get_states_cb)
        self.predict_ent_srv=rospy.Service('/frongo/get_entropies', PredictState, self.predict_entropy_cb)
        self.predict_ent_ord_srv=rospy.Service('/frongo/get_entropies_with_order', PredictStateOrder, self.predict_entropy_order_cb)
        self.predict_srv=rospy.Service('/frongo/predict_models', PredictState, self.predict_cb)
        self.predict_ord_srv=rospy.Service('/frongo/predict_models_with_order', PredictStateOrder, self.predict_order_cb)
        self.graph_build_srv=rospy.Service('/frongo/graph_model_build', GraphModel, self.graph_model_build_cb)
        self.new_model_srv=rospy.Service('/frongo/add_model_defs', AddModel, self.add_model_cb)
        self.info_srv=rospy.Service('/frongo/get_models', GetInfo, self.get_model_info_cb)
        self.rebuild_srv=rospy.Service('/frongo/rebuild_all_models', Trigger, self.rebuild_all_models_cb)
        self.detect_annomalies=rospy.Service('/frongo/detect_anomalies', DetectAnomalies, self.detect_anomalies_cb)       
        
        #self.graph_model_construction()
        rospy.loginfo("All Done ...")
        rospy.spin()



    def get_states_cb(self, req):
        with self.srv_lock:
            if len(req.epochs) < 2:
                rospy.logwarn("Size of epochs requested is less than two. Returning all epochs")
            
            for i in self.models:
                if i.name == req.model_name:
                    epochs, predictions = i._get_states(req.epochs)
       
        return epochs, predictions
        


    def get_model_info_cb(self, req):
        with self.srv_lock:
            names=[]
            info=[]
            for i in self.models:
                names.append(i.name)
                info.append(i._get_info())

        return names, info        


    def add_model_cb(self, req):
        with self.srv_lock:
            data=[]
            print req.model_def
            datum = yaml.load(req.model_def)
            print datum
            if not isinstance(datum, list):
                data.append(datum)
            else:
                data=datum
                
            self.create_models(data)
    
            if self.is_fremen_active:
                for i in self.models:
                    print i
                    i._create_fremen_models()
        return True



    def fremen_restart_cb(self, msg):
        """    
         This function creates the models when the fremenserver is started
        """
        if msg.data:
            rospy.logwarn("FREMENSERVER restart detected will generate new models now")
            with self.srv_lock:
                for i in self.models:
                    i._create_fremen_models()
                    print i.name, i.order
                self.is_fremen_active=True
            #self.create_models()


    def rebuild_all_models_cb(self, req):
        """    
         This function creates the models when the service is called
        """
        with self.srv_lock:
            resp=False
            if self.is_fremen_active:
                for i in self.models:
                    i._create_fremen_models()
                    print i.name, i.order
                resp=True
                str_msg="All Done"
            else:
                resp=False
                str_msg="No fremenserver"
        
        return resp, str_msg

    def predict_cb(self, req):
        with self.srv_lock:
            epochs =[]
            for i in req.epochs:
                epochs.append(i)
    
            model_found=False
            for i in self.models:
                if i.name == req.model_name:
                    predictions = i._predict_outcome(epochs)
                    model_found=True
                    
        if not model_found:
            rospy.logerr("Frongo: Model %s Not Found" %req.model_name)
            return epochs, [-1] * len(epochs)
       
        return epochs, predictions

    def predict_entropy_cb(self, req):
        with self.srv_lock:
            epochs =[]
            for i in req.epochs:
                epochs.append(i)

            model_found=False    
            for i in self.models:
                if i.name == req.model_name:
                    predictions = i._predict_entropy(epochs)
                    model_found=True

        if not model_found:
            rospy.logerr("Frongo: Model %s Not Found" %req.model_name)
            return epochs, [-1] * len(epochs)
       
        return epochs, predictions


    def predict_order_cb(self, req):
        with self.srv_lock:
            epochs =[]
            for i in req.epochs:
                epochs.append(i)

            model_found=False    
            for i in self.models:
                if i.name == req.model_name:
                    predictions = i._predict_outcome(epochs, order=req.order)
                    model_found=True

        if not model_found:
            rospy.logerr("Frongo: Model %s Not Found" %req.model_name)
            return epochs, [-1] * len(epochs)
       
        return epochs, predictions

    def predict_entropy_order_cb(self, req):
        with self.srv_lock:
            epochs =[]
            for i in req.epochs:
                epochs.append(i)
    
            model_found=False
            for i in self.models:
                if i.name == req.model_name:
                    predictions = i._predict_entropy(epochs, order=req.order)
                    model_found=True

        if not model_found:
            rospy.logerr("Frongo: Model %s Not Found" %req.model_name)
            return epochs,  [-1] * len(epochs)
            
        return epochs, predictions


    def create_models(self, data):
        rospy.loginfo("Creating Temporal Models:")
        rospy.loginfo("%s" %data)
        print data
        for i in data:
            #print i
            val=TModels(i['model']['name'])
            val._set_props_from_dict(i['model'])
            self.models.append(val)

        for i in self.models:
            print "-------------------------------------"
            print i
            if i.model_type != 'events':
                self.set_model_states(i)
            else:
                self.set_event_states(i)


    def set_event_states(self, model):
        db=self.mongo_client[model.db]
        collection=db[model.collection]
        query = json.loads(model.query)

        #print "++++++++++++++++"
        #print query
        #print model.timestamp_field
        
        oldest = collection.find(query).sort(model.timestamp_field,1).limit(1)
        youngest = collection.find(query).sort(model.timestamp_field, -1 ).limit(1)
        
        
        oldest_t=get_field(oldest[0], model.timestamp_field) 
        youngest_t= get_field(youngest[0], model.timestamp_field)
        
        if model.timestamp_type == 'datetime' :
            oldest_t= int(oldest_t.strftime("%s"))
            youngest_t= int(youngest_t.strftime("%s"))
            
        if model._dconf.has_key("sampling"):
            sampling=model._dconf['sampling']
        else:
            sampling=7200
        
        
        #print oldest_t, youngest_t, sampling
        #print youngest_t-oldest_t,  (youngest_t-oldest_t)/sampling
        #print "++++++++++++++++"
        
        #time0= oldest_t
        available = collection.find(query).sort(model.timestamp_field,1)


        model._add_state(oldest_t, 1.0)
        last_inserted_time=oldest_t

        epoch_events=[]        
        
        for i in available:
            epoch=get_field(i, model.timestamp_field)
            
            if model.timestamp_type == 'datetime' :
                epoch=int(epoch.strftime("%s"))
            epoch_events.append(epoch)


        n_epochs=0
        last_inserted_time=last_inserted_time+sampling
        while n_epochs < len(epoch_events):
            if epoch_events[n_epochs] < last_inserted_time:
                n_epochs+=1
                model._add_state(last_inserted_time, 1.0)
            else:
                last_inserted_time=last_inserted_time+sampling
                if epoch_events[n_epochs] < last_inserted_time:
                    n_epochs+=1
                    model._add_state(last_inserted_time, 1.0)
                else:
                    model._add_state(last_inserted_time, 0.0)

        

    def set_model_states(self, model):
        db=self.mongo_client[model.db]
        collection=db[model.collection]
        query = json.loads(model.query)
        available = collection.find(query)
        if available:
            for i in available:
                model._add_entry(i)
        else:
            model._set_unknown(True)

    def graph_model_build_cb(self, req):
        self.graph_model_construction(req)
        return "Done"        

    def detect_anomalies_cb(self, req):
        model_found=False
        with self.srv_lock:
            for i in self.models:
                if i.name == req.model_name:
                    print "Detecting annomalies over: "+str(req.confidence)+" for "+req.model_name
                    epochs, values = i._detect_anomalies(req.order, req.confidence)
                    model_found=True

        if not model_found:
            rospy.logerr("Frongo: Model %s Not Found" %req.model_name)
            return [0],[-1]
            
        return epochs, values


    def graph_model_construction(self, req):
        for i in self.models:
            if req.model_name == i.name:
                preds=[]
                graph = graph_model(i.name)
                ordeps = np.arange(min(i.epochs), max(i.epochs), 3600)
                ordrange = np.arange(req.from_val, req.until_val+1, req.increment)
                for j in ordrange.tolist():
                    preds.append(i._predict_outcome(ordeps.tolist(), order=j))
                graph.graph_model_construction(i.epochs, i.states, preds, ordeps.tolist())


    def _on_node_shutdown(self):
        rospy.loginfo("Shutting Down ...")
        self.mongo_client.close()
        rospy.loginfo("Done... Bye")
            

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-yaml_defs", help="The yaml file", type=str)
    args, unknown = parser.parse_known_args() # Necessary due to roslaunch injecting rubbish arguments

    if args.yaml_defs:
        data = load_yaml(args.yaml_defs)
    else:
        print "starting empty"
        data=''
    
    rospy.init_node('door_prediction')
    server = frongo(data)
    