import numpy

import rospy
import actionlib
import fremenserver.msg
#import std_msgs

class fremen_interface(object):

    def __init__(self) :
        rospy.on_shutdown(self._on_node_shutdown)
#        self.lock = Lock()

        # Creating fremen server client
        rospy.loginfo("Creating fremen server client")
        self.FremenClient= actionlib.SimpleActionClient('fremenserver', fremenserver.msg.FremenAction)
        self.FremenClient.wait_for_server()
        
        rospy.loginfo("Set-Up Fremenserver monitors")
        #Fremen Server Monitor
        self.fremen_monitor = rospy.Timer(rospy.Duration(10), self.monitor_cb)
        rospy.loginfo(" ...done")

        
    def sampling_by_extrapolation(self, nepochs):
        #samples for result sampling
        index_b = range(int(numpy.ceil(nepochs*0.75)))
        index_e = range(int(numpy.ceil(nepochs*0.75)),nepochs)
        return index_b, index_e


    def get_build_and_eval_states(self, epochs, states, sampling_type='extrapolation'):

        if sampling_type == 'extrapolation':
            index_b, index_e = self.sampling_by_extrapolation(len(epochs))            
        else:
            #random sampling
        
           #samples for result sampling        
            index_b = sorted(random.sample(xrange(len(epochs)), int(numpy.ceil(len(epochs)*0.8)))) 
            index_e = []
            for i in range(len(epochs)):
                if i not in index_b:
                    index_e.append(i)

            #samples for time sampling
            tindex_b = sorted(random.sample(xrange(len(tepochs)), int(numpy.ceil(len(tepochs)*0.8)))) 
            tindex_e = []
            for i in range(len(tepochs)):
                if i not in tindex_b:
                    tindex_e.append(i)        
        
        
    """
     create_fremen_model
     
     This function creates the fremen model for each door
    """
    def create_fremen_model(self, name, epochs, states, sampling_type='extrapolation'):
       


   
        if not index_e:
            index_e = random.sample(xrange(len(epochs)), 1)

        if not tindex_e:
            tindex_e = random.sample(xrange(len(tepochs)), 1)
            
        epochs_build = [ epochs[i] for i in index_b]
        epochs_eval = [ epochs[i] for i in index_e]
        res_build = [ res[i] for i in index_b]
        res_eval = [ res[i] for i in index_e]
        
        to_ret['res']=self.add_and_eval_models(door['model_id']['res'], epochs_build, res_build, epochs_eval, res_eval)

        return to_ret


    """
     add_and_eval_models
     
     This function creates and evaluates fremen models for binary states
     it returns the recommended order for the predictions
    """
    def add_and_eval_models(self, model_id, a_epochs, a_states, e_epochs, e_states):
        fremgoal = fremenserver.msg.FremenGoal()
        fremgoal.operation = 'add'
        fremgoal.id = model_id
        fremgoal.times = a_epochs
        fremgoal.states = a_states
        
        # print "--- BUILD ---"
        self.FremenClient.send_goal(fremgoal)
        self.FremenClient.wait_for_result()
        ps = self.FremenClient.get_result()
        #print ps
        
        # print "--- EVALUATE ---"
        frevgoal = fremenserver.msg.FremenGoal()
        frevgoal.operation = 'evaluate'
        frevgoal.id = model_id
        frevgoal.times = e_epochs
        frevgoal.states = e_states
        frevgoal.order = 5
        
        self.FremenClient.send_goal(frevgoal)
        self.FremenClient.wait_for_result()
        pse = self.FremenClient.get_result()  
        #print pse.errors
        #print "chosen order %d" %pse.errors.index(min(pse.errors))
        return pse.errors.index(min(pse.errors))


    """
     add_and_eval_value_models
     
     This function creates and evaluates fremen models for float values
     it returns the recommended order for the predictions
    """
    def add_and_eval_value_models(self, model_id, a_epochs, a_states, e_epochs, e_states):
        #print a_states
        fremgoal = fremenserver.msg.FremenGoal()
        fremgoal.operation = 'addvalues'
        fremgoal.id = model_id
        fremgoal.times = a_epochs
        fremgoal.values = a_states
        
        # Sends the goal to the action server.
        self.FremenClient.send_goal(fremgoal)
        #print "Sending data to fremenserver"
        
        
        # Waits for the server to finish performing the action.
        self.FremenClient.wait_for_result()
        
        #print "fremenserver done"
        
        # Prints out the result of executing the action
        ps = self.FremenClient.get_result()
        #print "fremenserver result:"
        #print ps
        
        # print "--- EVALUATE ---"
        frevgoal = fremenserver.msg.FremenGoal()
        frevgoal.operation = 'evaluate'
        frevgoal.id = model_id
        frevgoal.times = e_epochs
        frevgoal.states = e_states
        frevgoal.order = 5
        
        # Sends the goal to the action server.
        self.FremenClient.send_goal(frevgoal)
        
        # Waits for the server to finish performing the action.
        self.FremenClient.wait_for_result()
        
        # Prints out the result of executing the action
        pse = self.FremenClient.get_result()  # A FibonacciResult
        #print pse.errors
        #print "chosen order %d" %pse.errors.index(min(pse.errors))
        return pse.errors.index(min(pse.errors))

    

    def forecast_outcome(self, epoch, mods, ords):
        #print epoch, mods, ords
        fremgoal = fremenserver.msg.FremenGoal()
        fremgoal.operation = 'forecast'
        fremgoal.ids = mods
        fremgoal.times.append(epoch)
        
        fremgoal.order = -1
        fremgoal.orders = ords
        
        self.FremenClient.send_goal(fremgoal)
        self.FremenClient.wait_for_result(timeout=rospy.Duration(10.0))

#        if self.FremenClient.get_state() == actionlib.GoalStatus.SUCCEEDED:
        ps = self.FremenClient.get_result()
        #print ps
        prob = list(ps.probabilities)
        return prob


    """
     monitor_cb
     
     This function monitors if fremenserver is still active
    """
    def monitor_cb(self, events) :
        if not self.FremenClient.wait_for_server(timeout = rospy.Duration(1)):
            rospy.logerr("NO FREMEN SERVER FOUND. Fremenserver restart might be required")


    def _on_node_shutdown(self):
        self.fremen_monitor.shutdown()
