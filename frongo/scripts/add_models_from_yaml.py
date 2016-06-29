#!/usr/bin/env python
import sys
import rospy


from frongo.srv import AddModel

if __name__ == '__main__':
    filename=str(sys.argv[1])
    rospy.init_node('frongo_add_model_defs')
    rospy.loginfo("loading %s"%filename)
    f= open(filename, 'r').read()
    print f
    rospy.wait_for_service('/frongo/add_model_defs', timeout=10)
    try:
        add_models  = rospy.ServiceProxy('/frongo/add_model_defs', AddModel)
        resp1 = add_models(f)
        print resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    