#!/usr/bin/python


import rospy
import roslib


import web
import signal
from json import dumps
from yaml import load
from datetime import datetime
from time import mktime, strptime, time
from bson import json_util
from os import _exit
from urllib import urlencode

from os import chdir

from frongo.srv import PredictStateOrder
from frongo.srv import PredictState
from frongo.srv import GetInfo
from frongo.srv import DetectAnomalies



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
        user_data = web.input()
        frongo = FrongoBridge()


        info = frongo.get_info()
        data = {
          'submit_url': '/query',
          'queries': info,
          'datetime_format': DATETIME_PATTERN_JS,
          'def_from': user_data['from'] if 'from' in user_data else '',
          'def_to': user_data['to'] if 'to' in user_data else '',
          'def_order': user_data['order'] if 'order' in user_data else '0',
          'def_model': user_data['model'] if 'model' in user_data else '',
          'anom_conf': user_data['anom_conf'] if 'anom_conf' in user_data else '1',
        }
        return renderer.index(data)


class FrongoBridge:

    pred_srv_name = '/frongo/predict_models_with_order'
    entr_srv_name = '/frongo/get_entropies_with_order'
    info_srv_name = '/frongo/get_models'
    states_srv_name = '/frongo/get_states'
    anomalies_srv_name = '/frongo/detect_anomalies'

    def __init__(self):
        rospy.loginfo('waiting for services')
        rospy.wait_for_service(self.pred_srv_name)
        rospy.wait_for_service(self.info_srv_name)
        rospy.wait_for_service(self.entr_srv_name)
        rospy.wait_for_service(self.states_srv_name)
        rospy.wait_for_service(self.anomalies_srv_name)
        self.pred_srv = rospy.ServiceProxy(self.pred_srv_name,
                                           PredictStateOrder)
        self.entr_srv = rospy.ServiceProxy(self.entr_srv_name,
                                           PredictStateOrder)
        self.info_srv = rospy.ServiceProxy(self.info_srv_name,
                                           GetInfo)
        self.states_srv = rospy.ServiceProxy(self.states_srv_name,
                                             PredictState)
        self.anomalies_srv = rospy.ServiceProxy(self.anomalies_srv_name,
                                                DetectAnomalies)
        rospy.loginfo('frongo services ready')

    def get_info(self):
        infos = self.info_srv()

        res = zip(infos.names, infos.info)
        return res

    def query_values(self, model, order, epochs):
        res = self.pred_srv(model, int(order), epochs)
        return res

    def query_states(self, model, fr, to):
        res = self.states_srv(model, [fr, to])
        return res

    def query_entropies(self, model, order, epochs):
        res = self.entr_srv(model, int(order), epochs)
        return res

    def query_anomalies(self, model, order, confidence):
        res = self.anomalies_srv(model, int(order), confidence)
        return res


class Query:

    RESOLUTION = 500
    MIN_STEP = 300

    def dts_to_epoch(self, dts):
        return int(mktime(strptime(dts, DATETIME_PATTERN)))

    def epoch_to_dts(self, epoch):
        return datetime.fromtimestamp(epoch).strftime(DATETIME_PATTERN)

    def query_frongo(self, model, order, confidence, epoch_from, epoch_to):
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
        fstates = frongo.query_states(model, epoch_from, epoch_to)
        fanomalies = frongo.query_anomalies(model, order, confidence)

        # we can use these fstates to eventually display the real observations

        for f in frongo.get_info():
            if f[0] == model:
                finfo = f[1]

        res = {
            'epochs': fpred.epochs,
            'values': fpred.predictions,
            'states': fstates.predictions,
            'states_epochs': fstates.epochs,
            'entropies': fentr.predictions,
            'anomalies_epochs': fanomalies.epochs,
            'anomalies_values': fanomalies.values,
            'model_info': finfo
        }
        # res = {
        #     'epochs': epochs,
        #     'values': [random() for e in epochs],
        #     'entropies': [random() for e in epochs]
        # }
        return res

    def prepare_prediction_plot(self, d):
        dataset_probs = {
            'label': 'Probability',
            'backgroundColor': "rgba(0,0,220,0.3)",
            'borderColor': "rgba(0,0,220,1)",
            'pointColor': "rgba(0,0,220,1)",
            'pointStrokeColor': "#fff",
            'pointHighlightFill': "#fff",
            'pointHighlightStroke': "rgba(220,220,220,1)",
            'data': d['values']
        }

        dataset_ent = {
            'label': 'Entropy',
            'backgroundColor': "rgba(0,220,120,0.3)",
            'borderColor': "rgba(0,220,120,1)",
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

        return data


    def prepare_observation_plot(self, d):
        dataset_obs = {
            'label': 'Observations',
            'fill': False,
            'backgroundColor': "rgba(0,220,0,0.3)",
            'borderColor': "rgba(0,0,0,0)",
            'borderWidth': 0,
            'pointStrokeColor': "#fff",
            'pointHighlightFill': "#fff",
            'pointHighlightStroke': "rgba(220,220,220,1)",
            'data': [{'x': p[1], 'y': p[0]} for p in zip(d['states'], d['states_epochs'])]
        }

        dataset_anom = {
            'label': 'Anomalies',
            'fill': False,
            'backgroundColor': "rgba(255,0,0,1)",
            'borderColor': "rgba(0,0,0,0)",
            'pointStyle': "square",
            'radius': 6,
            'borderWidth': 0,
            'pointStrokeColor': "#fff",
            'pointHighlightFill': "#fff",
            'pointHighlightStroke': "rgba(220,220,220,1)",
            'data': [{'x': p[1], 'y': p[0]} for p in zip(d['anomalies_values'], d['anomalies_epochs'])]
        }

        print 'data:', dataset_anom['data']

        data = {
            'type': 'line',
            'labels': [self.epoch_to_dts(s)
                       for s in d['states_epochs']],
            'datasets': [dataset_obs, dataset_anom],
            'model_info': d['model_info']
        }

        return data

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
                              float(user_data['confidence']),
                              epoch_from,
                              epoch_to)

        prediction_chart = self.prepare_prediction_plot(d)
        observation_chart = self.prepare_observation_plot(d)
        query_params = {
            'model':    user_data['model'],
            'order':    str(user_data['order']),
            'from':     self.epoch_to_dts(epoch_from),
            'to':       self.epoch_to_dts(epoch_to),
            'anom_conf':  user_data['confidence']
        }

        model_info = load(prediction_chart['model_info'])
        data = {
            'prediction_chart':     prediction_chart,
            'observation_chart':    observation_chart,
            'url':                  '/?' + urlencode(query_params),
            'min':                  epoch_from,
            'max':                  epoch_to,
            'model_info':			prediction_chart['model_info'],
            'best_order': model_info['order']
        }

        data['url'] = '/?' + urlencode(query_params)
        return dumps(data, default=json_util.default)


def signal_handler(signum, frame):
    _exit(signal.SIGTERM)


if __name__ == '__main__':
    rospy.init_node("frongo_webserver")
    port = rospy.get_param('~port', 8999)
    app = FrongoApp(urls, globals())
    signal.signal(signal.SIGINT, signal_handler)
    app.run(port=port)
