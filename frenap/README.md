# Overview

The FreNaP package contains FreMEn-based prediction framework intended for topological navigation.
It takes the navigation statistics collected in the *mongo_store* and creates a dynamic FreMEn-based model of the navigation result and duration.
These dynamic models can be used to predict if the robot is likely to succeed in reaching its intended destination. 

# Practical 
The FreNap package contains an action server that receives four types of goals specified in the action string.

##The **build** action

retrieves the data from the Mongo and builds the FreMEn models:

###input:
- **mapName** with the topological map name that you want to process,
- **resultOrder** which is FreMEn order for the result prediction - I recommend the values 2 or 1, but you can also let FreNaP to choose the model by entering the value of -1.
- **timeOrder** which is FreMEn order for the duration prediction -  I recommend a static model for that, but again, you can enter -1 and FreNaP will choose the model order with the best estimation (not prediction!) accuracy.

###output:
- **status** - string containing the number of edges,
- **edgeName** - list of edges,
- **probability** - list of the model reconstruction errors (for the individual edges) as defined in the FreMEn [paper](http://labe.felk.cvut.cz/~tkrajnik/papers/fremen_2014_ICRA.pdf),
- **duration** - average duration prediction error (for the individual edges). 

##The **predict** action
performs the prediction for a particular time given in epoch seconds,

###input:
- **predictionTime**: in seconds since epoch,

###output:
- **edgeName**:   list of edges,
- **probabilities**: list of predicted probabilities that the edge will be traversed successfully,
- **durations**: list of predicted durations.

##The **timeline** action 
allows to recover several predictions for a particular edge

###input:
- **mapName**:  name of the edge to get the prediction time-line,
- **startTime**:  the starting time of the predictions,
- **endTime**:  the final time of the predictions,
- **predictionTime**:  the time step between the two previous times.

###output:
- **probabilities**: list of predicted probabilities that the edge will be traversed successfully,
- **durations**: list of predicted durations.

##The **debug** action 
The debug action prints detailed info about the edge given in
-**mapName** with the topological map name that you want to process,
