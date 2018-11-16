""" BlueSky plugin for logging ac data. """

from bluesky import stack, traf, sim
import pandas as pd
import pickle as p


class Logger:
    def __init__(self):
        self.acid = []
        self.data = {}
        self.enable = False

    def log(self, acid, flag):
        # find if acid exists
        if acid not in traf.id:
            raise ValueError('acid not found')

        # Enable or disable logging for acid
        if flag is True:
            self.acid.append(acid)
            self.data[acid] = []
        if flag is False:
            if acid in self.acid:
                self.acid.remove(acid)
            else:
                print("Cannot remove acid, not added.")

    def update(self):
        if self.enable:
            for ac in self.acid:
                idx = traf.id2idx(ac)
                self.data[ac].append([sim.simt, traf.lat[idx], traf.lon[idx], traf.alt[idx], traf.hdg[idx],
                                      traf.gs[idx], traf.M[idx], traf.tas[idx], traf.perf.mass])

    def preupdate(self):
        pass

    def reset(self):
        self.acid = []
        self.data = {}
        self.enable = False

    def save(self):
        results = {}
        for ac in self.acid:
            df = pd.DataFrame(self.data[ac], columns=['time', 'lat', 'lon', 'alt', 'heading', 'gs', 'M', 'tas', 'mass'])
            results[ac] = df
        p.dump(results, open("output/data.p", "wb"))


logger = Logger()


# Initialization function of your plugin. Do not change the name of this
# function, as it is the way BlueSky recognises this file as a plugin.
def init_plugin():
    # Additional initialisation code

    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'FLOGG',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim',

        # Update interval in seconds. By default, your plugin's update function(s)
        # are called every timestep of the simulation. If your plugin needs less
        # frequent updates provide an update interval.
        'update_interval': 1,

        # The update function is called after traffic is updated. Use this if you
        # want to do things as a result of what happens in traffic. If you need to
        # something before traffic is updated please use preupdate.
        'update':          logger.update,

        # The preupdate function is called before traffic is updated. Use this
        # function to provide settings that need to be used by traffic in the current
        # timestep. Examples are ASAS, which can give autopilot commands to resolve
        # a conflict.
        'preupdate':       logger.preupdate,

        # If your plugin has a state, you will probably need a reset function to
        # clear the state in between simulations.
        'reset':         logger.reset
        }

    stackfunctions = {
        # The command name for your function
        'FLOGG_ADD': [
            # A short usage string. This will be printed if you type HELP <name> in the BlueSky console
            'FLOGG_ADD [acid] ON/OFF',

            # A list of the argument types your function accepts. For a description of this, see ...
            '[txt, onoff]',

            # The name of your function in this plugin
            add,

            # a longer help text of your function.
            'Print something to the bluesky console based on the flag passed to MYFUN.'],

        'FLOGG_TOGGLE': [
            # A short usage string. This will be printed if you type HELP <name> in the BlueSky console
            'FLOGG_TOGGLE ON/OFF',

            # A list of the argument types your function accepts. For a description of this, see ...
            '[onoff]',

            # The name of your function in this plugin
            toggle,

            # a longer help text of your function.
            'Print something to the bluesky console based on the flag passed to MYFUN.']
    }

    # init_plugin() should always return these two dicts.
    return config, stackfunctions


# Other functions of your plugin
def add(acid, flag=True):
    logger.log(acid, flag)
    return flag, 'logger active'


def toggle(flag):
    if logger.enable is True and flag is False:  # switch off
        logger.save()
    logger.enable = flag
