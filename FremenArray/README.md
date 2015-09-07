# Overview

The FremenArray is a tool for prediction of binary states based on non-uniform, incremental Fourier Transform.
In contrast to the FremenServer, the FremenArray is meant to interface occupancy grids used in the ROS navigation stack.
It uses frequency spectrum analysis to identify reoccuring patterns in temporal sequences of binary states and represents the probability of these states in an analythical form as a combination of harmonic functions. 
This allows to learn long-term dynamics of the environment and use the learned experience to predict states of the world models used in mobile robotics.
This ability is beneficial for tasks like self-localization, object search, path planning, anomaly detection and exploration.

## Practical 

The FremenArray is an action server that maintains a set of state models held in an single-dimensional array which conforms to the ROS occupancy grid format, i.e. -1 means unknown cell and occupancy probability is scaled from 0 to 100. 
Three different operations can be performed with the array of states.
These correspond to three types of goals specified in the **operation** string.

###The 'add' operation

This allows to add a the measured map **states** along with the **time** of its observations.
On the first call of the **add** operation, the size of the **states** array is remembered and used as a number of cells of the map. 
This number cannot be changed later on. 
The **add** operation remembers the last timestamp that was provided and adds only newer observation to the model.

####Inputs
- **states** a sequence of numbers from -1 to 100, where -1 is an unknown cell observed at
- **time** which is in seconds. The length of the fields **states** in all calls has to be the same.

####Outputs
- **success** contains a number of cells updated in the model. Note that this might be lower than the length of the **states** in case that some were unobserved (valued -1). **success** equals to -2 if **states** has a different length from before. 
- **message** contains more detailed report or error message.

###The 'predict' operation
This operation calculates the (scaled by 100, so it conforms to the ROS map format) **probabilities** of the states for the given **time**.

####Inputs
- **time** at which the **probabilities** of the state should be estimated (in seconds).
- **order** of the model used for the estimation. The **order** equals to the number of periodic processes to be modeled. Setting the order to 0 results in a static model with **probabilities** constant in time.

####Outputs
- **success** contains the number of predicted states or -1 if the **order** is out of bounds.
- **message** contains a detailed report or error message.
- **probabilities** is an array of predicted probabilities (scaled by 100) of the states t given **time**.

###The 'entropy' operation 
This operation calculates the **entropies** of the states for the given **time**.

####Inputs
- **time** is the time instant at which the **entropies** of the states should be estimated.
- **order** equals to the number of periodic processes to be modeled, i.e. model order. Setting **order** to 0 results in a static model, so that the **entropies** will be constant in time.

####Outputs
- **success** contains the number of predicted states or -1 if the **order** is out of bounds.
- **message** contains a detailed report or an error message.
- **entropies** is an array of state's predicted entropies at given **time**.

###The 'evaluate' operation 
The **evaluate** operation is meant to support decisions what model order to use for probability and entropy predictions.
It allows to calculate the prediction error of the various model orders of a given (by **id**) state.
The user provides a sequence of observed **states** and **times** of observations and a maximal **order** to be evaluated.
The server performs predictions for the given **times** and **orders**, compares those with the provided **states** and calculates the percentage of unsuccessful predictions for model orders from 0 to **order**.

####Inputs
- **id** identification of the state that is concerned. If the **id** did not exist, an error is reported.
- **states** is a sequence of zeros and ones indicating the observed states at  
- **times** which are in seconds. The length of the fields **times** and **states** has to be the same.
- **order** is the maximal model order to be evaluated. 

####Outputs
- **success** contains the best performing model order, -1 if the state **id** does not exist and -2 if **times** and **states** have different lengths.
- **message** contains a detailed report or an error message.
- **errors** is an array of prediction errors for model orders from 0 to **order**.

###The 'print' operation 
Makes the fremenarray server to print data about the states on the screen.

####Inputs
- **order** number of periodic components to print for each state.

####Outputs
- **success** contains the number of states in the collection before the operation was performed. 
- **message** contains a detailed report or an error message.
