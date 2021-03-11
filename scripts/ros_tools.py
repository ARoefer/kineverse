#!/usr/bin/env python
import rospy
import sys

import kineverse.network.names as stn

from kineverse.srv import ListPaths as ListPathsSrv
from kineverse.srv import LoadModel as LoadModelSrv
from kineverse.srv import SaveModel as SaveModelSrv

class Tool(object):
    def __init__(self, name, srv_type):
        self.proxy = rospy.ServiceProxy(name, ListPathsSrv)

    @classmethod
    def desc(cls):
        raise NotImplementedError

    @classmethod
    def name(cls):
        raise NotImplementedError

    @classmethod
    def sig_str(cls):
        args     = cls._call.__code__.co_varnames[1:cls._call.__code__.co_argcount]
        defaults = cls._call.func_defaults if cls._call.func_defaults is not None else []
        return '{} {} {}'.format(cls.name(), ' '.join(args[:-len(defaults)]), ' '.join(['[{}]'.format(x) for x in args[-len(defaults):]]))

    def call(self, raw_args):
        if '-h' in raw_args or '--help' in raw_args:
            return 'Usage: {}'.format(self.sig_str())

        args     = self._call.__code__.co_varnames[1:self._call.__code__.co_argcount]
        defaults = self._call.func_defaults
            
        if len(raw_args) < len(args) - len(defaults):
            raise Exception('Missing arguments! Call signature: {}'.format(self.sig_str()))

        assignment = dict(zip(args[-len(self._call.func_defaults):], self._call.func_defaults))

        for x in range(len(raw_args)):
            assignment[args[x]] = raw_args[x] if x < len(args) - len(defaults) else type(defaults[x - len(args)])(raw_args[x])

        return self._call(**assignment)

    def _call(self, **kwargs):
        raise NotImplementedError


class ListPaths(Tool):
    def __init__(self):
        super(ListPaths, self).__init__(stn.srv_list_paths, ListPathsSrv)

    @classmethod
    def desc(cls):
        return 'Lists all path currently present in the model.\nOptionally starting at a specific node up to a specific depth.'

    @classmethod
    def name(cls):
        return 'list'

    def _call(self, root='', depth=0):
        return '\n'.join(self.proxy(root, depth).paths)


class LoadModel(Tool):
    def __init__(self):
        super(LoadModel, self).__init__(stn.srv_load_model, LoadModelSrv)

    @classmethod
    def desc(cls):
        return 'Loads a model to the kineverse server.'

    @classmethod
    def name(cls):
        return 'load'

    def _call(self, filepath):
        return self.proxy(filepath).error_msg


class SaveModel(Tool):
    def __init__(self):
        super(SaveModel, self).__init__(stn.srv_load_model, SaveModelSrv)

    @classmethod
    def desc(cls):
        return 'Saves a model (or a part of it) to a file.'

    @classmethod
    def name(cls):
        return 'save'

    def _call(self, filepath, model_path=''):
        return self.proxy(filepath, model_path).error_msg



if __name__ == '__main__':
    rospy.init_node('kineverse_ros_tools')

    tools = {t.name(): t() for t in [ListPaths, LoadModel, SaveModel]}

    if len(sys.argv) < 2 or sys.argv[1] in {'-h', '--help'}:
        print('\n'.join(['- {}\n    {}'.format(t.sig_str(), t.desc().replace('\n', '\n    ')) for _, t in sorted(tools.items())]))
        exit(0)

    if sys.argv[1] not in tools:
        print('Unknown command "{}" type -h or --help for a list of commands'.format(sys.argv[1]))
        exit(0)

    try:
        print(tools[sys.argv[1]].call(sys.argv[2:]))
    except Exception as e:
        print(str(e))
    