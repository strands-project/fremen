import matplotlib.pyplot as plt
import numpy as np

class graph_model(object):

    def __init__(self, name) :
        self.name = name


    def graph_model_construction(self, ep, states, preds, oep):
        yp=[]
        pred_colours=['blue','green','cyan','magenta','yellow','black']
        
        x = np.asarray(ep)
        x2 = np.asarray(oep)
        y1= np.asarray(states)
        for i in preds:
            yp.append(np.asarray(i))
        #print x
        #print y1
        #print y2
        
#       plt.plot(x, y1)
#       plt.savefig('trav.png') 
#       
#       plt.plot(x, y2)
#       plt.savefig('times.png')
        
        fig = plt.figure()
        
        ax1 = fig.add_subplot(111)
        ax1.plot(x, y1, color='red', marker='o', linestyle='')#(x, y1, 'r^')
        col=0
        for i in yp:
            ax1.plot(x2, i, color=pred_colours[col], marker='', linestyle='-')
            col+=1

#       ax2 = fig.add_subplot(212)
#       ax2.plot(x, y2, 'k-')

       #plt.tight_layout()
        plt.title(self.name)
        plt.ylim(-0.1,1.1)
        plt.show()