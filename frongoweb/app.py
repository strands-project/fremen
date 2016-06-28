import sys
import os
abspath = os.path.dirname(__file__)
print abspath

if len(abspath) > 0:
    sys.path.append(abspath)
    os.chdir(abspath)

import web
import signal
from json import dumps
from pymongo import MongoClient
from uuid import uuid4
from datetime import datetime
from bson import json_util
from threading import Condition
import httpagentparser
from os import _exit

import time


from urlparse import urlparse


renderer = web.template.render('templates', base="base", globals=globals())

urls = (
    '/', 'Index',
    '/query', 'Query'
)


listen_port = 8088


class FrongoApp(web.application):

    def run(self, *middleware):
        func = self.wsgifunc(*middleware)
        return web.httpserver.runsimple(func, ('0.0.0.0', listen_port))

if __name__ == '__main__':
    app = FrongoApp(urls, globals())
else:
    app = web.application(urls, globals(), autoreload=False)


class Index:

    def GET(self):
        data = {
          'submit_url': '/query',
          'queries': ['test1', 'test2']}
        return renderer.index(data)


class Query:

    def POST(self):
        return web.ok()


def signal_handler(signum, frame):
    print "stopped."
    _exit(signal.SIGTERM)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    app.run()
else:
    application = app.wsgifunc()
