#!/usr/bin/python


import rospy
import roslib


import web
import signal
from random import random
from json import dumps
from datetime import datetime
from time import mktime, strptime, time
from bson import json_util
from os import _exit

from os import chdir



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
        data = {
          'submit_url': '/query',
          'queries': ['test1', 'test2'],
          'datetime_format': DATETIME_PATTERN_JS
        }
        return renderer.index(data)


class Query:

    RESOLUTION = 50
    MIN_STEP = 300

    def dts_to_epoch(self, dts):
        return int(mktime(strptime(dts, DATETIME_PATTERN)))

    def epoch_to_dts(self, epoch):
        return datetime.fromtimestamp(epoch).strftime(DATETIME_PATTERN)

    def query_frongo(self, model, epoch_from, epoch_to):
        duration = epoch_to - epoch_from
        steps_from_duration = int(duration / self.RESOLUTION)
        steps = max(steps_from_duration, self.MIN_STEP)

        epochs = range(epoch_from, epoch_to+steps, steps)

        rospy.loginfo(epochs)
        # to be changed into the actual query
        res = {
            'epochs': epochs,
            'values': [random() for e in epochs],
            'entropies': [random() for e in epochs]
        }
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
            'datasets': [dataset_probs, dataset_ent]
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
                              epoch_from,
                              epoch_to)
        return self.prepare_plot(d)


def signal_handler(signum, frame):
    _exit(signal.SIGTERM)


if __name__ == '__main__':

    app = FrongoApp(urls, globals())

    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node("frongo_webserver")
    port = rospy.get_param('~port', 8999)

    app.run(port=port)
