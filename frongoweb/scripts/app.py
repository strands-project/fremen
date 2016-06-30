#!/usr/bin/python


import rospy
import roslib


import web
import signal
from json import dumps
from datetime import datetime
from time import mktime, strptime, time
from bson import json_util
from os import _exit
from urllib import urlencode

from os import chdir

from frongo.srv import PredictStateOrder
from frongo.srv import GetInfo



### Templates
TEMPLATE_DIR = roslib.packages.get_pkg_dir('frongoweb') + '/www'
chdir(TEMPLATE_DIR)

DATETIME_PATTERN = '%d.%m.%Y %H:%M'
DATETIME_PATTERN_JS = 'dd.mm.yyyy hh:ii'

renderer = web.template.render(TEMPLATE_DIR, base="base", globals=globals())

urls = (
    '/', 'Index',
    '/query', 'Query'
)


class FrongoApp(web.application):

    def run(self, port, *middleware):
        func = self.wsgifunc(*middleware)
        return web.httpserver.runsimple(func, ('0.0.0.0', port))


class Index:

    def GET(self):
        frongo = FrongoBridge()

        info = frongo.get_info()
        data = {
          'submit_url': '/query',
          'queries': info,
          'datetime_format': DATETIME_PATTERN_JS
        }
        return renderer.index(data)


class FrongoBridge:

    pred_srv_name = '/frongo/predict_models_with_order'
    entr_srv_name = '/frongo/get_entropies_with_order'
    info_srv_name = '/frongo/get_models'

    def __init__(self):
        rospy.loginfo('waiting for services')
        rospy.wait_for_service(self.pred_srv_name)
        rospy.wait_for_service(self.info_srv_name)
        self.pred_srv = rospy.ServiceProxy(self.pred_srv_name,
                                           PredictStateOrder)
        self.entr_srv = rospy.ServiceProxy(self.entr_srv_name,
                                           PredictStateOrder)
        self.info_srv = rospy.ServiceProxy(self.info_srv_name,
                                           GetInfo)
        rospy.loginfo('frongo services ready')

    def get_info(self):
        infos = self.info_srv()

        res = zip(infos.names, infos.info)
        return res

    def query_values(self, model, order, epochs):
        res = self.pred_srv(model, int(order), epochs)
        return res

    def query_entropies(self, model, order, epochs):
        res = self.entr_srv(model, int(order), epochs)
        return res


class Query:

    RESOLUTION = 50
    MIN_STEP = 300

    def dts_to_epoch(self, dts):
        return int(mktime(strptime(dts, DATETIME_PATTERN)))

    def epoch_to_dts(self, epoch):
        return datetime.fromtimestamp(epoch).strftime(DATETIME_PATTERN)

    def query_frongo(self, model, order, epoch_from, epoch_to):
        duration = epoch_to - epoch_from
        steps_from_duration = int(duration / self.RESOLUTION)
        steps = max(steps_from_duration, self.MIN_STEP)

        epochs = range(epoch_from, epoch_to+steps, steps)

        rospy.loginfo(epochs)
        # to be changed into the actual query
        frongo = FrongoBridge()
        fpred = frongo.query_values(model, order, epochs)
        fentr = frongo.query_entropies(model, order, epochs)
        finfo = ''

        for f in frongo.get_info():
            if f[0] == model:
                finfo = f[1]

        res = {
            'epochs': fpred.epochs,
            'values': fpred.predictions,
            'entropies': fentr.predictions,
            'model_info': finfo
        }
        # res = {
        #     'epochs': epochs,
        #     'values': [random() for e in epochs],
        #     'entropies': [random() for e in epochs]
        # }
        return res

    def prepare_plot(self, d):
        dataset_probs = {
            'label': 'Probability',
            'fillColor': "rgba(0,0,220,0.3)",
            'strokeColor': "rgba(0,0,220,1)",
            'pointColor': "rgba(0,0,220,1)",
            'pointStrokeColor': "#fff",
            'pointHighlightFill': "#fff",
            'pointHighlightStroke': "rgba(220,220,220,1)",
            'data': d['values']
        }

        dataset_ent = {
            'label': 'Entropy',
            'fillColor': "rgba(0,220,120,0.3)",
            'strokeColor': "rgba(0,220,120,1)",
            'pointColor': "rgba(0,220,120,1)",
            'pointStrokeColor': "#fff",
            'pointHighlightFill': "#fff",
            'pointHighlightStroke': "rgba(220,220,220,1)",
            'data': d['entropies']
        }

        data = {
            'labels': [self.epoch_to_dts(s)
                       for s in d['epochs']],
            'datasets': [dataset_probs, dataset_ent],
            'model_info': d['model_info']
        }

        # dataset = {
        #     'label': "responses",
        #     'fillColor': "rgba(120,0,0,0.5)",
        #     'data': [results[r] for r in sorted_keys]
        # }

        return dumps(data, default=json_util.default)

    def GET(self):
        user_data = web.input()
        print user_data
        if len(user_data['model']) == 0:
            rospy.logwarn('empty model received from web form')
            return web.BadRequest()
        try:
            epoch_from = self.dts_to_epoch(user_data['epoch_from'])
        except Exception as e:
            rospy.logwarn(e)
            epoch_from = int(time())-3600
        try:
            epoch_to = self.dts_to_epoch(user_data['epoch_to'])
        except Exception as e:
            rospy.logwarn(e)
            epoch_to = int(time())

        d = self.query_frongo(user_data['model'],
                              user_data['order'],
                              epoch_from,
                              epoch_to)
        return self.prepare_plot(d)


def signal_handler(signum, frame):
    _exit(signal.SIGTERM)


if __name__ == '__main__':
    rospy.init_node("frongo_webserver")
    port = rospy.get_param('~port', 8999)
    app = FrongoApp(urls, globals())

    signal.signal(signal.SIGINT, signal_handler)

    app.run(port=port)
