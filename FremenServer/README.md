# Overview

The FremenServer is a tool for prediction of binary states based on non-uniform, incremental Fourier Transform.
It uses frequency spectrum analysis to identify reoccuring patterns in temporal sequences of binary states and represents the probability of these states in an analythical form as a combination of harmonic functions. 
This allows to learn long-term dynamics of the environment and use the learned experience to predict states of the world models used in mobile robotics.
This ability is beneficial for tasks like self-localization, object search, path planning, anomaly detection and exploration.

## Practical 

The FremenServer is an action server that maintains a collection of state models, each with a different **id**.
Six different operations can be performed with each state held in the collection.
These correspond to six types of goals specified in the **operation** string that are related to a state given by its **id**.

###The 'add' operation

This allows to add a sequence of observed **states** along with the **times** of observations to the model of the state **id**.
If the **id** is given for the first time, a new state is created and filled with the values.
The **add** operation remembers the last timestamp that was provided and adds only newer observation to the model.

####Inputs
- **id** identification of the state that is concerned. If the **id** did not exist, a new state will be created. 
- **states** a sequence of zeroes and ones observed at
- **times** which are in seconds. The length of the fields **times** and **states** has to be the same.

####Outputs
- **success** contains a number of observation added to the model. Note that this might be lower than the length of the **states** in case that some of the **times** are older that the **times** of the previous step. Equals to -2 if **times** and **states** have different lengths. 
- **message** contains more detailed report or error message.

###The 'predict' operation
This operation calculates the **probabilities** of the state **id**  for the given **times**.
Can predict the probability of one state for several time instants.

####Inputs
- **id** is identification of the state that is concerned. If the **id** does not exist, an error is reported.
- **times** at which the **probabilities** of the state should be estimated (in seconds).
- **order** of the model used for the estimation. The **order** equals to the number of periodic processes to be modeled. Setting the order to 0 results in a static model with **probabilities** constant in time.

####Outputs
- **success** contains the number of predicted states or -1 if the state **id** does not exist.
- **message** contains a detailed report or error message.
- **probabilities** is an array of predicted probabilities of the state **id** at given **times**.

###The 'forecast' operation
This operation calculates the **probabilities** of the states in the **ids** array for the  **times**.
Can predict the probability of several states for one time instant.

####Inputs
- **ids** is identification of the states concerned. If the **ids** is not filled, an error is reported.
- **times** at which the **probabilities** of the states should be estimated (in seconds). Must have length 1, otherwise an error is reported. 
- **orders** of the model used for the estimation. Each **orders** element equals to the number of periodic processes to be modeled for a given state. Setting the order to 0 results in a static model with **probabilities** constant in time.
- **order** alternatively, the same **order** can be used for all states. The **order** is used instead of **orders** if the number of elements in the **ids** and **orders** differ.

####Outputs
- **success** contains the number of predicted states, or -1 then error.
- **message** contains a detailed report or error message.
- **probabilities** is an array of predicted probabilities of the states **ids** at given **times**.


###The 'entropy' operation 
This operation calculates the **entropies** of the state **id**  for the given **times**.

####Inputs
- **id** is an identification of the state that is concerned. If the **id** did not exist, an error is reported.
- **times** are time instants at which the **entropies** of the state should be estimated.
- **order** equals to the number of periodic processes to be modeled, i.e. model order. Setting **order** to 0 results in a static model, so that the **entropies** will be constant in time.

####Outputs
- **success** is the number of predicted states, -1 if **id** does not exist.
- **message** contains a detailed report or an error message.
- **entropies** is an array of state's **id** predicted entropies at given **times**.

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

###The 'delete' operation 
Deletes a state with a given **id** from the state collection held in the server.

####Inputs
- **id** identification of the state that is concerned. If the **id** did not exist, an error is reported.

####Outputs
- **success** contains the number of states in the collection before the operation was performed. 
- **message** contains a detailed report or an error message.

###The 'update' operation 
Reserved for future use when FreMEn is fused with Gaussian Mixture Models.

###The 'detect' operation (TODO)
The **detect** operation is meant to detect anomalous observations based on their discrepancy with the model of **order** given a **confidence** level. The user provides a sequence of observed **values** and **times** of observations of a state **id**, model **order** and a **confidence** level. The server performs predictions for the given **times** and **orders**, compares those predictions with the provided **values** and returns **anomTimes**, when the observed states did not match the predictions on a given confidence **level**. Anomaly is defined as a situation where |p(t)-s(t)| > c, where *p(t)* is the prediction, *s(t)* is the observation and *c* is the confidence level.

####Inputs
- **id** identification of the state that is concerned. If the **id** did not exist, an error is reported.
- **values** is a sequence of zeros and ones indicating the observed states at  
- **times** which are in seconds. The length of the fields **times** and **values** has to be the same.
- **order** is the model order to be used for predictions, 
- **confidence** is the confidence level considered. 

####Outputs
- **anomTimes** contains the times of detected anomalies,
- **anomValues** contains the values of detected anomalies,
- **values** contains the values predicted for the given **times**,
- **message** contains a detailed report or an error message.
