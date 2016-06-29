#!/usr/bin/python


import rospy
import roslib


import web
import signal
from json import dumps
from datetime import datetime
from bson import json_util
from os import _exit

from os import chdir



### Templates
TEMPLATE_DIR = roslib.packages.get_pkg_dir('frongoweb') + '/www'
chdir(TEMPLATE_DIR)


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
          'queries': ['test1', 'test2']
        }
        return renderer.index(data)


class Query:

    def prepare_plot(self):
        probs_data = [.1, .2, .5, .7, .3, .6]
        ent_data = [.3, .1, .2, .5, .7, .3]
        dataset_probs = {
            'label': 'Probability',
            'fillColor': "rgba(0,0,220,0.3)",
            'strokeColor': "rgba(0,0,220,1)",
            'pointColor': "rgba(0,0,220,1)",
            'pointStrokeColor': "#fff",
            'pointHighlightFill': "#fff",
            'pointHighlightStroke': "rgba(220,220,220,1)",
            'data': probs_data
        }

        dataset_ent = {
            'label': 'Entropy',
            'fillColor': "rgba(0,220,120,0.3)",
            'strokeColor': "rgba(0,220,120,1)",
            'pointColor': "rgba(0,220,120,1)",
            'pointStrokeColor': "#fff",
            'pointHighlightFill': "#fff",
            'pointHighlightStroke': "rgba(220,220,220,1)",
            'data': ent_data
        }

        data = {
            'labels': [str(s) for s in probs_data],
            'datasets': [dataset_probs, dataset_ent]
        }

        # dataset = {
        #     'label': "responses",
        #     'fillColor': "rgba(120,0,0,0.5)",
        #     'data': [results[r] for r in sorted_keys]
        # }

        return dumps(data, default=json_util.default)

    def GET(self):
        return self.prepare_plot()


def signal_handler(signum, frame):
    print "stopped."
    _exit(signal.SIGTERM)


if __name__ == '__main__':

    app = FrongoApp(urls, globals())

    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node("frongo_server")
    port = rospy.get_param('~port', 8999)

    app.run(port=port)
