#!/usr/bin/env python

import rospy
import yaml
import json
import pymongo
import argparse

import std_msgs

from frongo.temporal_models import *
from frongo.fremen_interface import *


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
        self.models=[]
        rospy.on_shutdown(self._on_node_shutdown)
        host = rospy.get_param("mongodb_host")
        port = rospy.get_param("mongodb_port")
        self.mongo_client = pymongo.MongoClient(host, port)

        # Subscribe to fremen server start topic
        rospy.Subscriber('/fremenserver_start', std_msgs.msg.Bool, self.fremen_restart_cb)
        rospy.loginfo("... Done")
        
        self.create_models(data)
        for i in self.models:
            print i

        rospy.loginfo("All Done ...")
        rospy.spin()


    """
     fremen_start_cb
     
     This function creates the models when the fremenserver is started
    """
    def fremen_restart_cb(self, msg):
        if msg.data:
            rospy.logwarn("FREMENSERVER restart detected will generate new models now")
            #self.create_models()


    def create_models(self, data):
        print data
        for i in data:
            val=TModels(i['model']['name'])
            val._set_props_from_dict(i['model'])
            self.models.append(val)

        for i in self.models:
            self.set_model_states(i)

    def set_model_states(self, model):
        db=self.mongo_client[model.db]
        collection=db[model.collection]        
        query = json.loads(model.query)
        available = collection.find(query)        
        for i in available:
            model._add_entry(i)

    def _on_node_shutdown(self):
        rospy.loginfo("Shutting Down ...")
        self.mongo_client.close()
        rospy.loginfo("Done... Bye")
            

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("yaml_defs", help="The yaml file", type=str)
    args, unknown = parser.parse_known_args() # Necessary due to roslaunch injecting rubbish arguments

    data = load_yaml(args.yaml_defs)
    
    rospy.init_node('door_prediction')
    server = frongo(data)
    